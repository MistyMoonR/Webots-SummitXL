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
#include <kalman_filter_localization/ekf_localization_component.hpp>
#include <chrono>
#include <vector>
#include <memory>
#include <string>
using namespace std::chrono_literals;

namespace kalman_filter_localization
{
EkfLocalizationComponent::EkfLocalizationComponent(const rclcpp::NodeOptions & options)
: Node("ekf_localization", options),
  clock_(RCL_ROS_TIME),
  tfbuffer_(std::make_shared<rclcpp::Clock>(clock_)),
  listener_(tfbuffer_)
{
  declare_parameter("reference_frame_id", "map");
  get_parameter("reference_frame_id", reference_frame_id_);
  declare_parameter("robot_frame_id", "base_link");
  get_parameter("robot_frame_id", robot_frame_id_);
  declare_parameter("initial_pose_topic", get_name() + std::string("/initial_pose"));
  get_parameter("initial_pose_topic", initial_pose_topic_);
  declare_parameter("imu_topic", get_name() + std::string("/imu"));
  get_parameter("imu_topic", imu_topic_);
  declare_parameter("odom_topic", get_name() + std::string("/odometry"));
  get_parameter("odom_topic", odom_topic_);
  declare_parameter("gnss_pose_topic", get_name() + std::string("/gnss_pose"));
  get_parameter("gnss_pose_topic", gnss_pose_topic_);

  declare_parameter("pub_period", 10);
  get_parameter("pub_period", pub_period_);
  declare_parameter("var_imu_w", 0.0);
  get_parameter("var_imu_w", var_imu_w_);
  declare_parameter("var_imu_acc", 0.0);
  get_parameter("var_imu_acc", var_imu_acc_);
  declare_parameter("var_gnss_xy", 0.1);
  get_parameter("var_gnss_xy", var_gnss_xy_);
  declare_parameter("var_gnss_z", 0.15);
  get_parameter("var_gnss_z", var_gnss_z_);
  declare_parameter("var_odom_xyz", 0.2);
  get_parameter("var_odom_xyz", var_odom_xyz_);
  declare_parameter("use_gnss", true);
  get_parameter("use_gnss", use_gnss_);
  declare_parameter("use_odom", false);
  get_parameter("use_odom", use_odom_);


  initial_pose_recieved_ = true;

  Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf_.getNumState());
  x(STATE::X) = 0;
  x(STATE::Y) = 0;
  x(STATE::Z) = 0;
  x(STATE::QX) = 0;
  x(STATE::QY) = 0;
  x(STATE::QZ) = 0;
  x(STATE::QW) = 1;
  ekf_.setInitialX(x);

  tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

  set_on_parameters_set_callback(
    [this](const std::vector<rclcpp::Parameter> params)
    -> rcl_interfaces::msg::SetParametersResult
    {
      auto results = std::make_shared<rcl_interfaces::msg::SetParametersResult>();
      for (auto param : params) {
        if (param.get_name() == "var_imu_w") {
          if (var_imu_w_ > 0) {
            var_imu_w_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "var_imu_w must over 0";
          }
        }
        if (param.get_name() == "var_imu_acc") {
          if (var_imu_acc_ > 0) {
            var_imu_acc_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "var_imu_acc must over 0";
          }
        }
        if (param.get_name() == "var_gnss_xy") {
          if (var_gnss_xy_ > 0) {
            var_gnss_xy_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "var_gnss_xy must over 0";
          }
        }
        if (param.get_name() == "var_gnss_z") {
          if (var_gnss_z_ > 0) {
            var_gnss_z_ = param.as_double();
            results->successful = true;
            results->reason = "";
          } else {
            results->successful = false;
            results->reason = "var_gnss_z must over 0";
          }
        }
      }
      if (!results->successful) {
        results->successful = false;
        results->reason = "";
      }
      return *results;
    }
  );

  ekf_.setVarImuGyro(var_imu_w_);
  ekf_.setVarImuAcc(var_imu_acc_);
  var_gnss_ << var_gnss_xy_, var_gnss_xy_, var_gnss_z_;
  var_odom_ << var_odom_xyz_, var_odom_xyz_, var_odom_xyz_;

  // Setup Publisher
  std::string output_pose_name = "/odom";
  current_pose_pub_ = create_publisher<nav_msgs::msg::Odometry>(output_pose_name, 10);

  // Setup Subscriber
  auto initial_pose_callback =
    [this](const typename geometry_msgs::msg::PoseStamped::SharedPtr msg) -> void
    {
      std::cout << "initial pose callback" << std::endl;
      initial_pose_recieved_ = true;
      current_pose_ = *msg;

      Eigen::VectorXd x = Eigen::VectorXd::Zero(ekf_.getNumState());
      x(STATE::X) = current_pose_.pose.position.x;
      x(STATE::Y) = current_pose_.pose.position.y;
      x(STATE::Z) = current_pose_.pose.position.z;
      x(STATE::QX) = current_pose_.pose.orientation.x;
      x(STATE::QY) = current_pose_.pose.orientation.y;
      x(STATE::QZ) = current_pose_.pose.orientation.z;
      x(STATE::QW) = current_pose_.pose.orientation.w;
      ekf_.setInitialX(x);
    };

  auto imu_callback =
    [this](const typename sensor_msgs::msg::Imu::SharedPtr msg) -> void
    {
      if (initial_pose_recieved_) {
        sensor_msgs::msg::Imu transformed_msg;
        try {
          geometry_msgs::msg::Vector3Stamped acc_in, acc_out, w_in, w_out;
          acc_in.vector.x = msg->linear_acceleration.x;
          acc_in.vector.y = msg->linear_acceleration.y;
          acc_in.vector.z = msg->linear_acceleration.z;
          w_in.vector.x = msg->angular_velocity.x;
          w_in.vector.y = msg->angular_velocity.y;
          w_in.vector.z = msg->angular_velocity.z;
          tf2::TimePoint time_point = tf2::TimePoint(
            std::chrono::seconds(msg->header.stamp.sec) +
            std::chrono::nanoseconds(msg->header.stamp.nanosec));
          const geometry_msgs::msg::TransformStamped transform =
            tfbuffer_.lookupTransform(
            robot_frame_id_,
            msg->header.frame_id,
            time_point);
          tf2::doTransform(acc_in, acc_out, transform);
          tf2::doTransform(w_in, w_out, transform);
          transformed_msg.header.stamp = msg->header.stamp;
          transformed_msg.angular_velocity.x = w_out.vector.x;
          transformed_msg.angular_velocity.y = w_out.vector.y;
          transformed_msg.angular_velocity.z = w_out.vector.z;
          transformed_msg.linear_acceleration.x = acc_out.vector.x;
          transformed_msg.linear_acceleration.y = acc_out.vector.y;
          transformed_msg.linear_acceleration.z = acc_out.vector.z;
          predictUpdate(transformed_msg);
        } catch (tf2::TransformException & e) {
          RCLCPP_ERROR(this->get_logger(), "%s", e.what());
          return;
        }
      }
    };

  auto odom_callback =
    [this](const typename nav_msgs::msg::Odometry::SharedPtr msg) -> void
    {
      if (initial_pose_recieved_ && use_odom_) {
        Eigen::Affine3d affine;
        tf2::fromMsg(msg->pose.pose, affine);
        Eigen::Matrix4d odom_mat = affine.matrix();
        if (previous_odom_mat_ == Eigen::Matrix4d::Identity()) {
          current_pose_odom_ = current_pose_;
          previous_odom_mat_ = odom_mat;
          return;
        }

        Eigen::Affine3d current_affine;
        tf2::fromMsg(current_pose_odom_.pose, current_affine);
        Eigen::Matrix4d current_trans = current_affine.matrix();
        current_trans = current_trans * previous_odom_mat_.inverse() * odom_mat;

        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = current_trans(0, 3);
        pose.pose.position.y = current_trans(1, 3);
        pose.pose.position.z = current_trans(2, 3);
        measurementUpdate(pose, var_odom_);

        current_pose_odom_ = current_pose_;
        previous_odom_mat_ = odom_mat;
      }
    };

  auto gnss_pose_callback =
    [this](const typename geometry_msgs::msg::PointStamped::SharedPtr msg) -> void
    {
      if (initial_pose_recieved_ && use_gnss_) {
        geometry_msgs::msg::PoseStamped pose;
        pose.header = msg->header;
        pose.pose.position.x = - msg->point.x;
        pose.pose.position.y = - msg->point.y;
        pose.pose.position.z = 0;
        measurementUpdate(pose, var_gnss_);
      }
    };

  sub_initial_pose_ =
    create_subscription<geometry_msgs::msg::PoseStamped>(initial_pose_topic_, 1,
      initial_pose_callback);
  sub_imu_ =
    create_subscription<sensor_msgs::msg::Imu>("/imu", 1,
      imu_callback);
  sub_odom_ =
    create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1,
      odom_callback);
  sub_gnss_pose_ =
    create_subscription<geometry_msgs::msg::PointStamped>("/Summit_XL_Steel/robot_gps", 1,
      gnss_pose_callback);
  std::chrono::milliseconds period(pub_period_);
  timer_ = create_wall_timer(
    std::chrono::duration_cast<std::chrono::nanoseconds>(period),
    std::bind(&EkfLocalizationComponent::broadcastPose, this));
}

void EkfLocalizationComponent::predictUpdate(const sensor_msgs::msg::Imu imu_msg)
{
  current_stamp_ = imu_msg.header.stamp;

  double current_time_imu = imu_msg.header.stamp.sec +
    imu_msg.header.stamp.nanosec * 1e-9;
  Eigen::Vector3d gyro = Eigen::Vector3d(
    imu_msg.angular_velocity.x,
    imu_msg.angular_velocity.y,
    imu_msg.angular_velocity.z);
  Eigen::Vector3d linear_acceleration = Eigen::Vector3d(
    imu_msg.linear_acceleration.x,
    imu_msg.linear_acceleration.y,
    imu_msg.linear_acceleration.z);

  ekf_.predictionUpdate(current_time_imu, gyro, linear_acceleration);
}


void EkfLocalizationComponent::measurementUpdate(
  const geometry_msgs::msg::PoseStamped pose_msg,
  const Eigen::Vector3d variance)
{
  current_stamp_ = pose_msg.header.stamp;
  Eigen::Vector3d y = Eigen::Vector3d(pose_msg.pose.position.x,
      pose_msg.pose.position.y,
      pose_msg.pose.position.z);

  ekf_.observationUpdate(y, variance);
}

void EkfLocalizationComponent::broadcastPose()
{
  if (initial_pose_recieved_) {
    auto x = ekf_.getX();
    current_pose_.header.stamp = current_stamp_;
    current_pose_.header.frame_id = reference_frame_id_;
    current_pose_.pose.position.x = x(STATE::X);
    current_pose_.pose.position.y = x(STATE::Y);
    current_pose_.pose.position.z = x(STATE::Z);
    current_pose_.pose.orientation.x = x(STATE::QX);
    current_pose_.pose.orientation.y = x(STATE::QY);
    current_pose_.pose.orientation.z = x(STATE::QZ);
    current_pose_.pose.orientation.w = x(STATE::QW);

    nav_msgs::msg::Odometry pose_;
    pose_.header = current_pose_.header;
    pose_.pose.pose.position.x = current_pose_.pose.position.x;
    pose_.pose.pose.position.y = current_pose_.pose.position.y;
    pose_.pose.pose.position.z = current_pose_.pose.position.z;
    pose_.pose.pose.orientation.x = current_pose_.pose.orientation.x;
    pose_.pose.pose.orientation.y = current_pose_.pose.orientation.y;
    pose_.pose.pose.orientation.z = current_pose_.pose.orientation.z;
    pose_.pose.pose.orientation.w = current_pose_.pose.orientation.w;
    current_pose_pub_->publish(pose_);

    geometry_msgs::msg::TransformStamped t;

    t.header.stamp = current_pose_.header.stamp;
    t.header.frame_id = "odom";
    t.child_frame_id = "base_link";

    t.transform.translation.x = current_pose_.pose.position.x;
    t.transform.translation.y = current_pose_.pose.position.y;
    t.transform.translation.z = current_pose_.pose.position.z;
    t.transform.rotation.x = current_pose_.pose.orientation.x;
    t.transform.rotation.y = current_pose_.pose.orientation.y;
    t.transform.rotation.z = current_pose_.pose.orientation.z;
    t.transform.rotation.w = current_pose_.pose.orientation.w;

    tf_broadcaster_->sendTransform(t);
  }
}
}  // namespace kalman_filter_localization

RCLCPP_COMPONENTS_REGISTER_NODE(kalman_filter_localization::EkfLocalizationComponent)
