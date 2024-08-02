#pragma once

#include "types.h"
#include <nav_msgs/msg/occupancy_grid.hpp>

namespace nmpcnav {


class NmpcNavigation
{
public:
    NmpcNavigation(Config config);
    ~NmpcNavigation();

    void setGlobalPath(const std::vector<double>& x_in, const std::vector<double>& y_in);
    const GlobalPath& getGlobalPath() const;

    void setTargetPosition(double target_x, double target_y);

    const LocalTrajectory& getLocalTrajectoryRef() const;
    const std::vector<std::vector<LocalTrajectory>>& getLocalTrajectoryLib() const;

    Input runNavigationPipeline(const State& x0);

    void setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map);

private:
    double Ts_;
    int N_;

    double V_MAX;
    double DELTA_MAX;
    double ACCEL_MAX;
    double DECCEL_MAX;
    double YAW_RATE_MAX;
    int LIB_NUM_V;
    int LIB_NUM_DELTA;
    double L_F;
    double L_R;

    State curr_state_;
    Input curr_input_;
    TargetPosition target_pos_;
    GlobalPath global_path_;
    LocalTrajectory local_traj_ref_;
    Model model_;

    std::vector<std::vector<LocalTrajectory>> local_traj_lib_;

    Eigen::DiagonalMatrix<double, NX> QN_;
    Eigen::DiagonalMatrix<double, NX> Qk_;
    Eigen::DiagonalMatrix<double, NU> R_;

    nav_msgs::msg::OccupancyGrid::ConstSharedPtr map_;
    
    bool has_target_pos_;

    void create_local_traj_lib();
    void select_local_traj();
    void run_mpc();
};


} // namespace nmpcnav