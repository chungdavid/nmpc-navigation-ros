#include "nmpc_navigation/nmpc_navigation_ros.hpp"

#include <fstream>
#include <sstream>
#include <iostream>

using std::placeholders::_1;

enum class RvizId {
    GLOBAL_PATH,
    TRAJ_REF,
    TRAJ_LIB
};

NmpcNavigationRos::NmpcNavigationRos()
    : Node("nmpc_navigation_ros"),
      nmpc_navigation_(get_nmpc_config())
{
    // Declare and get node specific parameters
    this->declare_parameter("node_config.drive_topic", "/drive");
    this->declare_parameter("node_config.odom_topic", "/ego_racecar/odom");
    this->declare_parameter("node_config.vis_global_path_topic", "/vis_global_path");
    this->declare_parameter("node_config.vis_local_traj_topic", "/vis_local_traj");
    this->declare_parameter("node_config.target_pos_topic", "/target_pos");
    this->declare_parameter("node_config.use_csv_global_path", true);
    this->declare_parameter("node_config.global_path_csv_filename", "/sim_ws/src/f1tenth_gym_ros/maps/Budapest_centerline.csv");

    drive_topic_ = this->get_parameter("node_config.drive_topic").as_string();
    odom_topic_ = this->get_parameter("node_config.odom_topic").as_string();
    vis_global_path_topic_ = this->get_parameter("node_config.vis_global_path_topic").as_string();
    vis_local_traj_topic_ = this->get_parameter("node_config.vis_local_traj_topic").as_string();
    target_pos_topic_ = this->get_parameter("node_config.target_pos_topic").as_string();
    bool use_csv_global_path = this->get_parameter("node_config.use_csv_global_path").as_bool();
    std::string global_path_csv_filename = this->get_parameter("node_config.global_path_csv_filename").as_string();

    // Create publishers and subscribers
    pub_drive_ = this->create_publisher<ackermann_msgs::msg::AckermannDriveStamped>(drive_topic_, 10);
    sub_odom_ = this->create_subscription<nav_msgs::msg::Odometry>(odom_topic_, 1, std::bind(&NmpcNavigationRos::odom_callback, this, _1));
    pub_vis_global_path_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(vis_global_path_topic_, 10);
    pub_vis_local_traj_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(vis_local_traj_topic_, 10);
    sub_target_pos_ = this->create_subscription<geometry_msgs::msg::Point>(target_pos_topic_, 10, std::bind(&NmpcNavigationRos::target_pos_callback, this, _1));
    
    // If user decides to use a global path computed offline, load it from CSV file.
    if(use_csv_global_path) {
        load_global_path_from_csv(global_path_csv_filename);
        while(pub_vis_global_path_->get_subscription_count() < 1) {
            RCLCPP_WARN_ONCE(this->get_logger(), "pub_vis_global_path has no subscribers. Waiting for a connection before visualizing the global path.");
        }
        visualize_global_path();
        // visualize_local_traj_lib();
    }

}

NmpcNavigationRos::~NmpcNavigationRos() { }

nmpcnav::Config NmpcNavigationRos::get_nmpc_config()
{
    // Declare and get nmpc controller specific parameters
    this->declare_parameter("nmpc_config.car_max_speed", 8.0);
    this->declare_parameter("nmpc_config.car_max_steering_angle", 1.25);
    this->declare_parameter("nmpc_config.car_max_accel", 24.0);
    this->declare_parameter("nmpc_config.car_max_deccel", 30.0);
    this->declare_parameter("nmpc_config.car_max_yaw_rate", 5.0);
    this->declare_parameter("nmpc_config.car_length_front", 0.1651);
    this->declare_parameter("nmpc_config.car_length_rear", 0.1651);

    this->declare_parameter("nmpc_config.nmpc_time_step", 0.05);
    this->declare_parameter("nmpc_config.nmpc_control_horizon", 20);
    this->declare_parameter("nmpc_config.traj_lib_num_speed", 30);
    this->declare_parameter("nmpc_config.traj_lib_num_steer", 30);

    double v_max = this->get_parameter("nmpc_config.car_max_speed").as_double();
    double delta_max = this->get_parameter("nmpc_config.car_max_steering_angle").as_double();
    double accel_max = this->get_parameter("nmpc_config.car_max_accel").as_double();
    double deccel_max = this->get_parameter("nmpc_config.car_max_deccel").as_double();
    double yaw_rate_max = this->get_parameter("nmpc_config.car_max_yaw_rate").as_double();
    double l_f = this->get_parameter("nmpc_config.car_length_front").as_double();
    double l_r = this->get_parameter("nmpc_config.car_length_rear").as_double();

    double Ts = this->get_parameter("nmpc_config.nmpc_time_step").as_double();
    int N = this->get_parameter("nmpc_config.nmpc_control_horizon").as_int();
    int lib_num_v = this->get_parameter("nmpc_config.traj_lib_num_speed").as_int();
    int lib_num_delta = this->get_parameter("nmpc_config.traj_lib_num_steer").as_int();

    // Init main program nmpc_navigation_ with params
    nmpcnav::Config nmpc_config {
        v_max,
        delta_max,
        accel_max,
        deccel_max,
        yaw_rate_max,
        l_f,
        l_r,
        Ts,
        N,
        lib_num_v,
        lib_num_delta
    };

    return nmpc_config;
}

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

    std::vector<double> x_in; // where the x values form the CSV will be stored
    std::vector<double> y_in; // where the y values form the CSV will be stored
    std::string line, word;

    while (!csvfile.eof()) {
        std::getline(csvfile, line, '\n');
        std::stringstream s(line);

        int j = 0;
        while(std::getline(s, word, ',')) {
            if (!word.empty()) {
                if (j == 0) {
                    x_in.push_back(std::stod(word));
                } else if (j == 1) {
                    y_in.push_back(std::stod(word));
                }
            }
            j++;
        }
    }

    if(x_in.size() != y_in.size()) {
        RCLCPP_ERROR(this->get_logger(), "CSV file has a different number of X and Y values! Check that your CSV file is correct.");
    }

    nmpc_navigation_.setGlobalPath(x_in, y_in);
    csvfile.close();
    RCLCPP_INFO(this->get_logger(), "Finished loading %d points from %s", x_in.size(), filename.c_str());
}

void NmpcNavigationRos::visualize_global_path()
{
    const nmpcnav::GlobalPath& global_path = nmpc_navigation_.getGlobalPath(); // path to visualize

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker dots;
    dots.header.frame_id = "map";
    dots.id = static_cast<int>(RvizId::GLOBAL_PATH);
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
        p.x = global_path.X(i);
        p.y = global_path.Y(i);
        dots.points.push_back(p);
    }

    marker_array.markers.push_back(dots);
    pub_vis_global_path_->publish(marker_array);

    RCLCPP_INFO(this->get_logger(), "Finished visualizing %d points from the global path", dots.points.size());
}

void NmpcNavigationRos::visualize_local_traj_ref()
{
    const nmpcnav::LocalTrajectory& local_traj = nmpc_navigation_.getLocalTrajectoryRef();

    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker dots;
    dots.header.frame_id = "map";
    dots.id = static_cast<int>(RvizId::TRAJ_REF);
    dots.ns = "local_traj_ref_points";
    dots.type = visualization_msgs::msg::Marker::POINTS;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.header.stamp = rclcpp::Clock().now();    
    dots.scale.x = 0.1;
    dots.scale.y = 0.1;
    dots.scale.z = 0.1;
    dots.color.a = 1.0;
    dots.color.r = 1.0;

    for (int i = 0; i < local_traj.X.size(); ++i) {
        geometry_msgs::msg::Point p;
        p.x = local_traj.X(i);
        p.y = local_traj.Y(i);
        dots.points.push_back(p);
    }

    marker_array.markers.push_back(dots);
    pub_vis_local_traj_->publish(marker_array);
}

void NmpcNavigationRos::visualize_local_traj_lib()
{
    const std::vector<std::vector<nmpcnav::LocalTrajectory>>& local_traj_lib = nmpc_navigation_.getLocalTrajectoryLib();
    
    visualization_msgs::msg::MarkerArray marker_array;
    visualization_msgs::msg::Marker dots;
    dots.header.frame_id = "map";
    dots.id = static_cast<int>(RvizId::TRAJ_LIB);
    dots.ns = "local_traj_lib_points";
    dots.type = visualization_msgs::msg::Marker::POINTS;
    dots.action = visualization_msgs::msg::Marker::ADD;
    dots.header.stamp = rclcpp::Clock().now();
    dots.scale.x = 0.025;
    dots.scale.y = 0.025;
    dots.scale.z = 0.025;
    dots.color.a = 1.0;
    dots.color.b = 1.0;

    for(int i = 0; i < local_traj_lib.size(); ++i) {
        for(int j = 0; j < local_traj_lib[i].size(); ++j) {
            nmpcnav::LocalTrajectory local_traj = local_traj_lib[i][j];
            for (int k = 0; k < local_traj.X.size(); ++k) {
                geometry_msgs::msg::Point p;
                p.x = local_traj.X(k);
                p.y = local_traj.Y(k);
                dots.points.push_back(p);
            }
        }
    }

    marker_array.markers.push_back(dots);
    pub_vis_local_traj_->publish(marker_array);
}

void NmpcNavigationRos::odom_callback(const nav_msgs::msg::Odometry::ConstSharedPtr odom_msg)
{
    nmpcnav::State x0 = {odom_msg->pose.pose.position.x, odom_msg->pose.pose.position.y, 2*acos(odom_msg->pose.pose.orientation.w)};
    nmpcnav::Input nmpc_sol = nmpc_navigation_.runNavigationPipeline(x0);
    
    visualize_local_traj_ref();

    ackermann_msgs::msg::AckermannDriveStamped drive_msg;
    drive_msg.header.stamp = rclcpp::Clock().now();
    drive_msg.drive.speed = nmpc_sol.v;
    drive_msg.drive.steering_angle = nmpc_sol.delta;
    pub_drive_->publish(drive_msg);
}

void NmpcNavigationRos::target_pos_callback(const geometry_msgs::msg::Point::ConstSharedPtr target_pos_msg)
{
    nmpc_navigation_.setTargetPosition(target_pos_msg->x, target_pos_msg->y);
    visualize_global_path();
    RCLCPP_INFO(this->get_logger(), "A new target position was set: x=%d, y=%d", target_pos_msg->x, target_pos_msg->y);
}
