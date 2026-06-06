// Copyright 2025 Ursa Works
// Licensed under the Apache-2.0 License.
//
// Simplified serial bridge between the ursa2026_rm_vision ROS stack and the
// Infantry_Bottom C-board via USB-CDC VCP (/dev/ttyACM0 or /dev/gimbal).
//
// Receive (C-board -> Jetson, 34 bytes, 0x5A header):
//   Broadcasts TF: odom -> gimbal_pitch_odom from IMU roll/pitch/yaw.
//   Sets armor_detector_opencv detect_color parameter from flags byte.
//   Calls /tracker/reset service when reset_tracker bit is set.
//
// Send (Jetson -> C-board, 12 bytes, 0xA5 header):
//   Subscribes to cmd_gimbal (pb_rm_interfaces/GimbalCmd) and
//   cmd_shoot (example_interfaces/UInt8) at 50 Hz.

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <atomic>
#include <cstring>
#include <memory>
#include <mutex>
#include <string>
#include <thread>

#include <tf2/LinearMath/Quaternion.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

#include <example_interfaces/msg/u_int8.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <pb_rm_interfaces/msg/gimbal_cmd.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_srvs/srv/trigger.hpp>

#include "ursa_serial_driver/crc.hpp"
#include "ursa_serial_driver/packet.hpp"

namespace ursa_serial_driver
{

class UrsaSerialDriver : public rclcpp::Node
{
public:
  explicit UrsaSerialDriver(const rclcpp::NodeOptions & options)
  : Node("ursa_serial_driver", options)
  {
    device_name_ = declare_parameter<std::string>("device_name", "/dev/ttyACM0");
    timestamp_offset_ = declare_parameter<double>("timestamp_offset", 0.0);

    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    detector_param_client_ = std::make_shared<rclcpp::AsyncParametersClient>(
      this, "armor_detector_opencv");
    reset_tracker_client_ = create_client<std_srvs::srv::Trigger>("/tracker/reset");

    gimbal_cmd_sub_ = create_subscription<pb_rm_interfaces::msg::GimbalCmd>(
      "cmd_gimbal", rclcpp::SensorDataQoS(),
      std::bind(&UrsaSerialDriver::gimbalCmdCb, this, std::placeholders::_1));

    shoot_cmd_sub_ = create_subscription<example_interfaces::msg::UInt8>(
      "cmd_shoot", rclcpp::SensorDataQoS(),
      std::bind(&UrsaSerialDriver::shootCmdCb, this, std::placeholders::_1));

    // 50 Hz send timer
    send_timer_ = create_wall_timer(
      std::chrono::milliseconds(20),
      std::bind(&UrsaSerialDriver::sendTimerCb, this));

    openPort();
    receive_thread_ = std::thread(&UrsaSerialDriver::receiveLoop, this);

    RCLCPP_INFO(get_logger(), "UrsaSerialDriver started on %s", device_name_.c_str());
  }

  ~UrsaSerialDriver() override
  {
    if (receive_thread_.joinable()) {
      receive_thread_.join();
    }
    if (fd_ >= 0) {
      close(fd_);
    }
  }

private:
  void openPort()
  {
    if (fd_ >= 0) {
      close(fd_);
      fd_ = -1;
    }
    fd_ = open(device_name_.c_str(), O_RDWR | O_NOCTTY);
    if (fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Cannot open %s: %s", device_name_.c_str(), strerror(errno));
      return;
    }

    struct termios tty {};
    tcgetattr(fd_, &tty);
    // Raw mode: no echo, no canonical, no signals
    tty.c_iflag = IGNPAR;
    tty.c_oflag = 0;
    tty.c_lflag = 0;
    tty.c_cflag = CS8 | CLOCAL | CREAD;
    tty.c_cc[VMIN] = 1;
    tty.c_cc[VTIME] = 0;
    cfsetspeed(&tty, B115200);
    tcsetattr(fd_, TCSANOW, &tty);
    tcflush(fd_, TCIOFLUSH);

    RCLCPP_INFO(get_logger(), "Opened %s", device_name_.c_str());
  }

  void receiveLoop()
  {
    uint8_t buf[sizeof(ReceivePacket)];

    while (rclcpp::ok()) {
      if (fd_ < 0) {
        rclcpp::sleep_for(std::chrono::seconds(1));
        openPort();
        continue;
      }

      // Scan for header byte
      uint8_t hdr;
      ssize_t n = read(fd_, &hdr, 1);
      if (n <= 0) continue;
      if (hdr != 0x5A) continue;

      // Read remaining bytes
      buf[0] = hdr;
      size_t got = 1;
      while (got < sizeof(ReceivePacket)) {
        n = read(fd_, buf + got, sizeof(ReceivePacket) - got);
        if (n > 0) {
          got += static_cast<size_t>(n);
        } else {
          got = 0;
          break;
        }
      }
      if (got < sizeof(ReceivePacket)) continue;

      if (!crc16::Verify_CRC16_Check_Sum(buf, sizeof(ReceivePacket))) {
        RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "CRC error on receive");
        continue;
      }

      ReceivePacket pkt;
      memcpy(&pkt, buf, sizeof(pkt));
      handlePacket(pkt);
    }
  }

  void handlePacket(const ReceivePacket & pkt)
  {
    // Update detect_color parameter on the detector node
    if (!initial_set_param_ || pkt.detect_color != prev_detect_color_) {
      setDetectColor(pkt.detect_color);
      prev_detect_color_ = pkt.detect_color;
    }

    if (pkt.reset_tracker) {
      resetTracker();
    }

    // Broadcast dynamic TF: odom -> gimbal_pitch_odom
    geometry_msgs::msg::TransformStamped t;
    t.header.stamp = now() - rclcpp::Duration::from_seconds(timestamp_offset_);
    t.header.frame_id = "odom";
    t.child_frame_id = "gimbal_pitch_odom";
    tf2::Quaternion q;
    q.setRPY(pkt.roll, pkt.pitch, pkt.yaw);
    t.transform.rotation = tf2::toMsg(q);
    tf_broadcaster_->sendTransform(t);
  }

  void gimbalCmdCb(const pb_rm_interfaces::msg::GimbalCmd::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lk(send_mutex_);
    last_yaw_ = msg->position.yaw;
    last_pitch_ = msg->position.pitch;
    last_gimbal_stamp_ = now();
    has_target_ = true;
  }

  void shootCmdCb(const example_interfaces::msg::UInt8::SharedPtr msg)
  {
    shoot_cmd_ = msg->data;
    if (msg->data == 0) {
      std::lock_guard<std::mutex> lk(send_mutex_);
      has_target_ = false;
    }
  }

  void sendTimerCb()
  {
    if (fd_ < 0) return;

    SendPacket pkt;
    pkt.header = 0xA5;

    {
      std::lock_guard<std::mutex> lk(send_mutex_);
      // Expire target after 500 ms of no cmd_gimbal
      if (has_target_) {
        auto age = now() - last_gimbal_stamp_;
        if (age > rclcpp::Duration::from_seconds(0.5)) {
          has_target_ = false;
        }
      }

      if (!has_target_) {
        pkt.mode = 0;
        pkt.yaw = 0.0f;
        pkt.pitch = 0.0f;
      } else {
        pkt.mode = (shoot_cmd_ > 0) ? 2 : 1;
        pkt.yaw = last_yaw_;
        pkt.pitch = last_pitch_;
      }
    }

    crc16::Append_CRC16_Check_Sum(reinterpret_cast<uint8_t *>(&pkt), sizeof(pkt));
    if (write(fd_, &pkt, sizeof(pkt)) < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Write error: %s", strerror(errno));
    }
  }

  void setDetectColor(uint8_t color)
  {
    if (!detector_param_client_->service_is_ready()) return;
    if (set_param_future_.valid() &&
        set_param_future_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
      return;
    }
    RCLCPP_INFO(get_logger(), "Setting detect_color -> %d", color);
    set_param_future_ = detector_param_client_->set_parameters(
      {rclcpp::Parameter("detect_color", static_cast<int>(color))},
      [this](const ResultFuturePtr &) { initial_set_param_ = true; });
  }

  void resetTracker()
  {
    if (!reset_tracker_client_->service_is_ready()) return;
    reset_tracker_client_->async_send_request(
      std::make_shared<std_srvs::srv::Trigger::Request>());
    RCLCPP_INFO(get_logger(), "Tracker reset requested");
  }

  // Serial
  std::string device_name_;
  int fd_{-1};
  std::thread receive_thread_;

  // TF
  double timestamp_offset_{0.0};
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;

  // Send state
  std::mutex send_mutex_;
  bool has_target_{false};
  float last_yaw_{0.0f};
  float last_pitch_{0.0f};
  rclcpp::Time last_gimbal_stamp_{0, 0, RCL_ROS_TIME};
  std::atomic<uint8_t> shoot_cmd_{0};
  rclcpp::TimerBase::SharedPtr send_timer_;

  // Subscriptions
  rclcpp::Subscription<pb_rm_interfaces::msg::GimbalCmd>::SharedPtr gimbal_cmd_sub_;
  rclcpp::Subscription<example_interfaces::msg::UInt8>::SharedPtr shoot_cmd_sub_;

  // Detector param client
  using ResultFuturePtr =
    std::shared_future<std::vector<rcl_interfaces::msg::SetParametersResult>>;
  std::shared_ptr<rclcpp::AsyncParametersClient> detector_param_client_;
  ResultFuturePtr set_param_future_;
  bool initial_set_param_{false};
  uint8_t prev_detect_color_{0xFF};

  // Tracker reset service
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr reset_tracker_client_;
};

}  // namespace ursa_serial_driver

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(ursa_serial_driver::UrsaSerialDriver)
