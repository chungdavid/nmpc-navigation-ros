#include "nmpc_navigation/nmpc_navigation_ros.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

using std::placeholders::_1;

NmpcNavigationRos::NmpcNavigationRos()
    : Node("nmpc_navigation_ros")
{
    // Declare parameters
    this->declare_parameter("odom_topic", "/ego_racecar/odom");
    this->declare_parameter("drive_topic", "/drive");
    this->declare_parameter("vis_global_path_topic", "/vis_global_path");

    this->declare_parameter("target_pos_topic", "/target_pos");
    this->declare_parameter("use_csv_global_path", true);
    this->declare_parameter("global_path_csv_filename", "/sim_ws/f1tenth_gym_ros/maps/Budapest_centerline.csv");

    // Get parameters
    drive_topic_ = this->get_parameter("drive_topic").as_string();
    odom_topic_ = this->get_parameter("odom_topic").as_string();
    vis_global_path_topic_ = this->get_parameter("vis_global_path_topic").as_string();

    target_pos_topic_ = this->get_parameter("target_pos_topic").as_string();
    bool use_csv_global_path = this->get_parameter("use_csv_global_path").as_bool();
    std::string global_path_csv_filename = this->get_parameter("global_path_csv_filename").as_string();

    // Create publishers and subscribers
    pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 10, std::bind(&NmpcNavigationRos::odom_callback, this, _1));
    pub_vis_global_path_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(vis_global_path_topic_, 10);

    if(use_csv_global_path) {
        load_global_path_from_csv(global_path_csv_filename);
    } else {
        sub_target_pos_ = this->create_subscription<geometry_msgs::msg::Point>(target_pos_topic_, 10, std::bind(&NmpcNavigationRos::target_pos_callback, this, _1));
    }

    while(pub_vis_global_path_->get_subscription_count() < 1) {
        RCLCPP_WARN_ONCE(this->get_logger(), "pub_vis_global_path has no subscribers. Waiting for a connection before visualizing the global path.");
    }

    visualize_global_path();
}

NmpcNavigationRos::~NmpcNavigationRos() { }

void NmpcNavigationRos::load_global_path_from_csv(std::string filename)
{
    // Create file object
    std::fstream csvfile;
    csvfile.open(filename, std::ios::in);

    if(!csvfile.is_open()) {
        RCLCPP_ERROR(this->get_logger(), "Cannot Open CSV File: %s", filename.c_str());
        return;
    } else {
        RCLCPP_INFO(this->get_logger(), "CSV File Opened");
    }

    Path global_path; //where the CSV data will be loaded
    std::string line, word;

    while (!csvfile.eof()) {
        std::getline(csvfile, line, '\n');
        std::stringstream s(line);

        int j = 0;
        Point point;
        while(std::getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    point.x = std::stod(word);
                } else if (j == 1) {
                    point.y = std::stod(word);
                }
            }
            j++;
        }
        global_path.points.push_back(point);
    }

    csvfile.close();

    global_path.num_points = global_path.points.size();
    nmpc_navigation_.initGlobalPath(global_path);

    RCLCPP_INFO(this->get_logger(), "Finished loading %d points from %s", global_path.num_points, filename.c_str());
}

void NmpcNavigationRos::visualize_global_path()
{
    Path global_path = nmpc_navigation_.getGlobalPath(); // path to visualize

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker dots;
    dots.header.frame_id = "map";
    dots.id = 0;
    dots.ns = "global_path_points";
    dots.type = visualization_msgs::msg::Marker::POINTS;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.header.stamp = rclcpp::Clock().now();    
    dots.scale.x = 0.15;
    dots.scale.y = 0.15;
    dots.scale.z = 0.15;
    dots.color.a = 1.0;
    dots.color.g = 1.0;

    for (int i = 0; i < global_path.num_points; ++i) {
        geometry_msgs::msg::Point p;
        p.x = global_path.points[i].x;
        p.y = global_path.points[i].y;;
        dots.points.push_back(p);
    }

    marker_array.markers.push_back(dots);
    pub_vis_global_path_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Finished visualizing %d points from the global path", dots.points.size());
}

void NmpcNavigationRos::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    pub_drive_->publish(drive_msg);
}

void NmpcNavigationRos::target_pos_callback(const geometry_msgs::msg::Point::ConstSharedPtr target_pos_msg)
{
    // TODO
    // RCLCPP_INFO(this->get_logger(), "A new target position was set: x=%d, y=%d", target_pos_msg->x, target_pos_msg->y);
}
