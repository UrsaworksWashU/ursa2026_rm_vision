// Copyright 2022 Chen Jun
// Use of this source code is governed by an MIT-style
// license that can be found in the LICENSE file or at
// https://opensource.org/licenses/MIT.

#ifndef ARMOR_DETECTOR_OPENCV__PNP_SOLVER_HPP_
#define ARMOR_DETECTOR_OPENCV__PNP_SOLVER_HPP_

#include <array>
#include <vector>

#include "armor_detector_opencv/armor.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "opencv2/core.hpp"

namespace rm_auto_aim
{
class PnPSolver
{
public:
  PnPSolver(
    const std::array<double, 9> & camera_matrix,
    const std::vector<double> & distortion_coefficients);

  // Get 3d position
  bool solvePnP(const Armor & armor, cv::Mat & rvec, cv::Mat & tvec);

  // Calculate the distance between armor center and image center
  float calculateDistanceToCenter(const cv::Point2f & image_point);

private:
  // Image points in the same order used for the object points
  // (left bottom, left top, right top, right bottom)
  std::vector<cv::Point2f> toImagePoints(const Armor & armor) const;

  // Select the 3d model points matching the armor size
  const std::vector<cv::Point3f> & objectPoints(const Armor & armor) const;

  // Sum of L2 distances between detected and reprojected image points
  double calculateReprojectionError(
    const Armor & armor, const cv::Mat & rvec, const cv::Mat & tvec) const;

  // Resolve the IPPE planar PnP yaw ambiguity by selecting one of the two
  // candidate solutions. Returns the index into rvecs/tvecs to use.
  size_t selectPnPSolution(
    const Armor & armor, const std::vector<cv::Mat> & rvecs,
    const std::vector<cv::Mat> & tvecs) const;

  // Convert a rotation matrix to roll-pitch-yaw (radians)
  static cv::Vec3d rotationMatrixToRPY(const cv::Matx33d & R);

  cv::Mat camera_matrix_;
  cv::Mat dist_coeffs_;

  // Camera-optical -> gimbal-like frame (x forward, y left, z up).
  // Used only to read the yaw/roll sign when disambiguating; needs no IMU.
  const cv::Matx33d R_gimbal_camera_{0, 0, 1, -1, 0, 0, 0, -1, 0};

  // Unit: mm
  static constexpr float SMALL_ARMOR_WIDTH = 135;
  static constexpr float SMALL_ARMOR_HEIGHT = 55;
  static constexpr float LARGE_ARMOR_WIDTH = 225;
  static constexpr float LARGE_ARMOR_HEIGHT = 55;

  // Four vertices of armor in 3d
  std::vector<cv::Point3f> small_armor_points_;
  std::vector<cv::Point3f> large_armor_points_;
};

}  // namespace rm_auto_aim

#endif  // ARMOR_DETECTOR_OPENCV__PNP_SOLVER_HPP_
