#pragma once

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <ackermann_msgs/msg/ackermann_drive_stamped.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "nmpc_navigation/nmpc_navigation.hpp"

class NmpcNavigationRos : public rclcpp::Node
{
public:
    NmpcNavigationRos();
    ~NmpcNavigationRos();

private:
    void odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg);
    void target_pos_callback(const geometry_msgs::msg::Point::ConstSharedPtr target_pos_msg);

    std::string drive_topic_;
    std::string odom_topic_;
    std::string vis_global_path_topic_;
    std::string vis_local_traj_topic_;
    std::string target_pos_topic_;

    rclcpp::Publisher<ackermann_msgs::msg::AckermannDriveStamped>::SharedPtr pub_drive_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr sub_odom_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_global_path_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr pub_vis_local_traj_;
    rclcpp::Subscription<geometry_msgs::msg::Point>::SharedPtr sub_target_pos_;

    void load_global_path_from_csv(std::string filename);
    void visualize_global_path();
    void visualize_local_traj_ref();
    void visualize_local_traj_lib();

    nmpcnav::Config get_nmpc_config();
    nmpcnav::NmpcNavigation nmpc_navigation_; // ROS-free main code
};