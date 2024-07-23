#include "nmpc_navigation/nmpc_navigation.hpp"

#include <iostream>

NmpcNavigation::NmpcNavigation()
    : global_path_{}
{
    std::cout << "ROS-free main code init! \n"; 
}

NmpcNavigation::~NmpcNavigation() { }

void NmpcNavigation::setGlobalPath(std::vector<double>& x_in, std::vector<double>& y_in) {
    global_path_.X = Eigen::Map<Eigen::VectorXd>(x_in.data(), x_in.size());
    global_path_.Y = Eigen::Map<Eigen::VectorXd>(y_in.data(), y_in.size());
    global_path_.num_points = x_in.size();
}

const GlobalPath& NmpcNavigation::getGlobalPath() const { return global_path_; }


