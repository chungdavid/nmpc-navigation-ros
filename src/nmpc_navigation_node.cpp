#include "nmpc_navigation/nmpc_navigation_ros.hpp"

int main(int argc, char * argv[]) {
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<NmpcNavigationRos>());
    rclcpp::shutdown();
    return 0;
}