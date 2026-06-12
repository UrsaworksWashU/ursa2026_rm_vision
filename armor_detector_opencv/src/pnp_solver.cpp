// Copyright 2022 Chen Jun
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#include "armor_detector_opencv/pnp_solver.hpp"

#include <cmath>
#include <vector>

#include "opencv2/calib3d.hpp"
#include "tf2/LinearMath/Matrix3x3.h"
#include "tf2/LinearMath/Quaternion.h"

namespace rm_auto_aim
{
PnPSolver::PnPSolver(
  const std::array<double, 9> & camera_matrix, const std::vector<double> & dist_coeffs)
: camera_matrix_(cv::Mat(3, 3, CV_64F, const_cast<double *>(camera_matrix.data())).clone()),
  dist_coeffs_(cv::Mat(1, 5, CV_64F, const_cast<double *>(dist_coeffs.data())).clone())
{
  // Unit: m
  constexpr double small_half_y = SMALL_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double small_half_z = SMALL_ARMOR_HEIGHT / 2.0 / 1000.0;
  constexpr double large_half_y = LARGE_ARMOR_WIDTH / 2.0 / 1000.0;
  constexpr double large_half_z = LARGE_ARMOR_HEIGHT / 2.0 / 1000.0;

  // Start from bottom left in clockwise order
  // Model coordinate: x forward, y left, z up
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, -small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, small_half_z));
  small_armor_points_.emplace_back(cv::Point3f(0, -small_half_y, -small_half_z));

  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, -large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, large_half_z));
  large_armor_points_.emplace_back(cv::Point3f(0, -large_half_y, -large_half_z));
}

bool PnPSolver::solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec)
{
  // IPPE returns the two candidate solutions for a planar 4-point target,
  // sorted by ascending reprojection error.
  std::vector<cv::Mat> rvecs, tvecs;
  bool success = cv::solvePnPGeneric(
    objectPoints(armor), toImagePoints(armor), camera_matrix_, dist_coeffs_, rvecs, tvecs, false,
    cv::SOLVEPNP_IPPE);

  if (!success || rvecs.empty()) {
    return false;
  }

  // Resolve the yaw ambiguity (left/right mirror) between the two solutions.
  size_t best = rvecs.size() < 2 ? 0 : selectPnPSolution(armor, rvecs, tvecs);
  rvec = rvecs[best];
  tvec = tvecs[best];
  return true;
}

std::vector<cv::Point2f> PnPSolver::toImagePoints(const Armor & armor) const
{
  // Start from bottom left in clockwise order, matching the object points
  std::vector<cv::Point2f> image_armor_points;
  image_armor_points.emplace_back(armor.left_light.bottom);
  image_armor_points.emplace_back(armor.left_light.top);
  image_armor_points.emplace_back(armor.right_light.top);
  image_armor_points.emplace_back(armor.right_light.bottom);
  return image_armor_points;
}

const std::vector<cv::Point3f> & PnPSolver::objectPoints(const Armor & armor) const
{
  return armor.type == ArmorType::SMALL ? small_armor_points_ : large_armor_points_;
}

double PnPSolver::calculateReprojectionError(
  const Armor & armor, const cv::Mat & rvec, const cv::Mat & tvec) const
{
  auto image_points = toImagePoints(armor);
  std::vector<cv::Point2f> reprojected_points;
  cv::projectPoints(
    objectPoints(armor), rvec, tvec, camera_matrix_, dist_coeffs_, reprojected_points);

  double error = 0;
  for (size_t i = 0; i < image_points.size(); ++i) {
    error += cv::norm(image_points[i] - reprojected_points[i]);
  }
  return error;
}

cv::Vec3d PnPSolver::rotationMatrixToRPY(const cv::Matx33d & R)
{
  tf2::Matrix3x3 tf2_rotation(
    R(0, 0), R(0, 1), R(0, 2), R(1, 0), R(1, 1), R(1, 2), R(2, 0), R(2, 1), R(2, 2));
  double roll, pitch, yaw;
  tf2_rotation.getRPY(roll, pitch, yaw);
  return cv::Vec3d(roll, pitch, yaw);
}

size_t PnPSolver::selectPnPSolution(
  const Armor & armor, const std::vector<cv::Mat> & rvecs,
  const std::vector<cv::Mat> & tvecs) const
{
  // If one solution reprojects far better than the other, trust it. IPPE sorts
  // by ascending reprojection error, so index 0 is the lower-error candidate.
  constexpr double PROJECT_ERR_THRES = 3.0;
  constexpr double MAX_ROLL_RAD = 10.0 / 180.0 * CV_PI;

  cv::Mat R1_cv, R2_cv;
  cv::Rodrigues(rvecs[0], R1_cv);
  cv::Rodrigues(rvecs[1], R2_cv);

  // RPY of the armor in the gimbal-like frame (x forward, y left, z up)
  cv::Vec3d rpy1 = rotationMatrixToRPY(R_gimbal_camera_ * cv::Matx33d(R1_cv));
  cv::Vec3d rpy2 = rotationMatrixToRPY(R_gimbal_camera_ * cv::Matx33d(R2_cv));

  double error1 = calculateReprojectionError(armor, rvecs[0], tvecs[0]);
  double error2 = calculateReprojectionError(armor, rvecs[1], tvecs[1]);

  // Don't disambiguate when one solution clearly dominates or the roll is
  // physically implausible; fall back to the lower-error solution.
  if ((error1 > 0 && error2 / error1 > PROJECT_ERR_THRES) ||
      std::abs(rpy1[0]) > MAX_ROLL_RAD || std::abs(rpy2[0]) > MAX_ROLL_RAD) {
    return 0;
  }

  // Light-bar tilt direction in the image determines the physical yaw sign.
  // axis points from bottom to top of each light bar.
  cv::Point2f axis_l = armor.left_light.top - armor.left_light.bottom;
  cv::Point2f axis_r = armor.right_light.top - armor.right_light.bottom;
  double l_angle = std::atan2(axis_l.y, axis_l.x) * 180 / CV_PI;
  double r_angle = std::atan2(axis_r.y, axis_r.x) * 180 / CV_PI;
  double angle = (l_angle + r_angle) / 2 + 90.0;
  if (armor.number == "outpost") {
    angle = -angle;
  }

  // Pick the solution whose yaw sign matches the physical tilt.
  // Armor tilts left (angle > 0)  -> negative yaw solution.
  // Armor tilts right (angle < 0) -> positive yaw solution.
  // NOTE: sign verified against this repo's point ordering; flip if mirrored.
  double yaw1 = rpy1[2];
  double yaw2 = rpy2[2];
  if ((angle > 0 && yaw1 > 0 && yaw2 < 0) || (angle < 0 && yaw1 < 0 && yaw2 > 0)) {
    return 1;
  }
  return 0;
}

float PnPSolver::calculateDistanceToCenter(const cv::Point2f & image_point)
{
  float cx = camera_matrix_.at<double>(0, 2);
  float cy = camera_matrix_.at<double>(1, 2);
  return cv::norm(image_point - cv::Point2f(cx, cy));
}

}  // namespace rm_auto_aim
