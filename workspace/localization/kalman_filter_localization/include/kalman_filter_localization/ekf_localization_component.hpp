// Copyright (c) 2020, Ryohei Sasaki
// All rights reserved.
//
// Software License Agreement (BSD License 2.0)
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions
// are met:
//
//  * Redistributions of source code must retain the above copyright
//    notice, this list of conditions and the following disclaimer.
//  * Redistributions in binary form must reproduce the above
//    copyright notice, this list of conditions and the following
//    disclaimer in the documentation and/or other materials provided
//    with the distribution.
//  * Neither the name of {copyright_holder} nor the names of its
//    contributors may be used to endorse or promote products derived
//    from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
// FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
// COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
// INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
// BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
// CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
// LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
// ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
// POSSIBILITY OF SUCH DAMAGE.
#ifndef KALMAN_FILTER_LOCALIZATION__EKF_LOCALIZATION_COMPONENT_HPP_
#define KALMAN_FILTER_LOCALIZATION__EKF_LOCALIZATION_COMPONENT_HPP_

#if __cplusplus
extern "C" {
#endif

// The below macros are taken from https://gcc.gnu.org/wiki/Visibility and from
// demos/composition/include/composition/visibility_control.h at https://github.com/ros2/demos
#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define KFL_EKFL_EXPORT __attribute__ ((dllexport))
    #define KFL_EKFL_IMPORT __attribute__ ((dllimport))
  #else
    #define KFL_EKFL_EXPORT __declspec(dllexport)
    #define KFL_EKFL_IMPORT __declspec(dllimport)
  #endif
  #ifdef KFL_EKFL_BUILDING_DLL
    #define KFL_EKFL_PUBLIC KFL_EKFL_EXPORT
  #else
    #define KFL_EKFL_PUBLIC KFL_EKFL_IMPORT
  #endif
  #define KFL_EKFL_PUBLIC_TYPE KFL_EKFL_PUBLIC
  #define KFL_EKFL_LOCAL
#else
  #define KFL_EKFL_EXPORT __attribute__ ((visibility("default")))
  #define KFL_EKFL_IMPORT
  #if __GNUC__ >= 4
    #define KFL_EKFL_PUBLIC __attribute__ ((visibility("default")))
    #define KFL_EKFL_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define KFL_EKFL_PUBLIC
    #define KFL_EKFL_LOCAL
  #endif
  #define KFL_EKFL_PUBLIC_TYPE
#endif

#if __cplusplus
}  // extern "C"
#endif

#include <kalman_filter_localization/ekf.hpp>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/vector3_stamped.hpp>
#include "geometry_msgs/msg/point_stamped.hpp"
#include <sensor_msgs/msg/imu.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <tf2/transform_datatypes.h>
#include <tf2/convert.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_eigen/tf2_eigen.h>
#include <tf2_ros/buffer.h>
#include "tf2_ros/transform_broadcaster.h"
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>

#include <rclcpp_components/register_node_macro.hpp>

#include <Eigen/Core>

#include <string>

namespace kalman_filter_localization
{
class EkfLocalizationComponent : public rclcpp::Node
{
public:
  KFL_EKFL_PUBLIC
  explicit EkfLocalizationComponent(const rclcpp::NodeOptions & options);

private:
  std::string reference_frame_id_;
  std::string robot_frame_id_;
  std::string initial_pose_topic_;
  std::string imu_topic_;
  std::string odom_topic_;
  std::string gnss_pose_topic_;
  int pub_period_;

  double var_imu_w_;
  double var_imu_acc_;
  double var_gnss_xy_;
  double var_gnss_z_;
  Eigen::Vector3d var_gnss_;
  double var_odom_xyz_;
  Eigen::Vector3d var_odom_;
  bool use_gnss_;
  bool use_odom_;

  bool initial_pose_recieved_{false};

  geometry_msgs::msg::PoseStamped current_pose_;
  rclcpp::Time current_stamp_;

  EKFEstimator ekf_;

  rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr sub_initial_pose_;
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr sub_imu_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
  rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr sub_gnss_pose_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr current_pose_pub_;
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Clock clock_;
  tf2_ros::Buffer tfbuffer_;
  tf2_ros::TransformListener listener_;
  std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
  void predictUpdate(const sensor_msgs::msg::Imu imu_msg);
  void measurementUpdate(
    const geometry_msgs::msg::PoseStamped pose_msg,
    const Eigen::Vector3d variance);
  void broadcastPose();

  geometry_msgs::msg::PoseStamped current_pose_odom_;
  Eigen::Matrix4d previous_odom_mat_{Eigen::Matrix4d::Identity()};


  enum STATE
  {
    X  = 0, Y = 1, Z = 2,
    VX = 3, VY = 4, VZ = 5,
    QX = 6, QY = 7, QZ = 8, QW = 9,
  };
};
}  // namespace kalman_filter_localization

#endif  // KALMAN_FILTER_LOCALIZATION__EKF_LOCALIZATION_COMPONENT_HPP_
