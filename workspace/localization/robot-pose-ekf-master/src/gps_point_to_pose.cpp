#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "geometry_msgs/msg/pose_with_covariance_stamped.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2_ros/transform_broadcaster.h"
#include <iostream>
#include <memory>
#include <string>

using namespace std;


class TFbroadcast : public rclcpp::Node
{
public:
    TFbroadcast()
    : Node("tf_broadcaster")
    {
        subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
        "/Summit_XL_Steel/gps", 20, std::bind(&TFbroadcast::topic_callback, this, std::placeholders::_1));

        // sub_ekf_ = this->create_subscription<nav_msgs::msg::Odometry>(
        // "/odometry/ekf", 20, std::bind(&TFbroadcast::ekf_callback, this, std::placeholders::_1));

        publisher_ = this->create_publisher<nav_msgs::msg::Odometry>("/gps/odom", 10);

        tf_broadcaster_ = std::make_shared<tf2_ros::TransformBroadcaster>(this);

        tf_publisher_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);

        // Publish static transforms once at startup
        // this->make_transforms();
    }
private:
    rclcpp::Time now;
    void topic_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg)
    {
        nav_msgs::msg::Odometry pose_;
        pose_.header = msg->header;
        pose_.pose.pose.position.x = -msg->point.x;
        pose_.pose.pose.position.y = -msg->point.y;
        pose_.pose.pose.position.z = 0;
        publisher_->publish(pose_);

        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "gps_odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = -msg->point.x;
        t.transform.translation.y = -msg->point.y;
        t.transform.translation.z = 0;

        tf_broadcaster_->sendTransform(t);
    }

    void ekf_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
    {
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = msg->header.stamp;
        t.header.frame_id = "odom";
        t.child_frame_id = "base_link";

        t.transform.translation.x = msg->pose.pose.position.x;
        t.transform.translation.y = msg->pose.pose.position.y;
        t.transform.translation.z = msg->pose.pose.position.z;
        t.transform.rotation.x = msg->pose.pose.orientation.x;
        t.transform.rotation.y = msg->pose.pose.orientation.y;
        t.transform.rotation.z = msg->pose.pose.orientation.z;
        t.transform.rotation.w = msg->pose.pose.orientation.w;

        tf_broadcaster_->sendTransform(t);
    }

    void make_transforms()
    {
        rclcpp::Time now = this->get_clock()->now();
        geometry_msgs::msg::TransformStamped t;

        t.header.stamp = now;
        t.header.frame_id = "base_link";
        t.child_frame_id = "Velodyne_VLP_16";

        t.transform.translation.x = 0;
        t.transform.translation.y = 0;
        t.transform.translation.z = 0.11;
        t.transform.rotation.x = 0;
        t.transform.rotation.y = 0;
        t.transform.rotation.z = 0;
        t.transform.rotation.w = 1;

        tf_publisher_->sendTransform(t);
    }

    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr subscription_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_ekf_;
    rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr publisher_;
    std::shared_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_publisher_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<TFbroadcast>());
    rclcpp::shutdown();
    return 0;
}
