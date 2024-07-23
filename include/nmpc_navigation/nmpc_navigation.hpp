#pragma once

#include <vector>
#include <Eigen/Dense>

struct GlobalPath {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    int num_points;
};

class NmpcNavigation
{
public:
    NmpcNavigation();
    ~NmpcNavigation();

    void setGlobalPath(std::vector<double>& x_in, std::vector<double>& y_in);
    const GlobalPath& getGlobalPath() const;

private:
    GlobalPath global_path_;
};