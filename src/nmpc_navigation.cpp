#include "nmpc_navigation/nmpc_navigation.hpp"

#include <iostream>

namespace nmpcnav {


NmpcNavigation::NmpcNavigation(Config config)
    : Ts_(config.Ts), N_(config.N),
      V_MAX(config.v_max), DELTA_MAX(config.delta_max), ACCEL_MAX(config.accel_max),
      DECCEL_MAX(config.deccel_max), YAW_RATE_MAX(config.yaw_rate_max),
      LIB_NUM_V(config.lib_num_v), LIB_NUM_DELTA(config.lib_num_delta),
      L_F(config.l_f), L_R(config.l_r),
      model_(config.Ts, config.l_f, config.l_r),
      local_traj_lib_(config.lib_num_v, std::vector<LocalTrajectory>(config.lib_num_delta + 1))
{
    create_local_traj_lib();
    std::cout << "NmpcNavigation fully initialized! \n"; 
}

NmpcNavigation::~NmpcNavigation() { }

void NmpcNavigation::setGlobalPath(std::vector<double>& x_in, std::vector<double>& y_in) {
    global_path_ = GlobalPath(x_in, y_in);
    target_pos_.X = x_in[-1];
    target_pos_.Y = y_in[-1];
}

const GlobalPath& NmpcNavigation::getGlobalPath() const { return global_path_; }

void NmpcNavigation::setTargetPosition(double target_x, double target_y)
{ 
    target_pos_.X = target_x;
    target_pos_.Y = target_y;
    // TODO: generate a global path based on the new position
}

Input NmpcNavigation::runNavigationPipeline(const State& x0)
{
    curr_state_ = x0; // Update the state of the vehicle
    select_local_traj(); // Update the trajectory reference
    // run_mpc();

    
    // Update the current input of the vehicle, then return it
    return curr_input_;
}

void NmpcNavigation::create_local_traj_lib()
{
    std::cout << "Creating local trajectory library...\n";
    double d_v = V_MAX / LIB_NUM_V;
    for(int i = 0; i < LIB_NUM_V; ++i) { // skip v = 0
        double v = d_v + i * d_v;
        double max_delta_at_v = std::min(DELTA_MAX, atan(YAW_RATE_MAX * (L_F + L_R) / v));
        double d_delta = 2 * max_delta_at_v / LIB_NUM_DELTA;
        for(int j = 0; j < LIB_NUM_DELTA + 1; ++j) {
            double delta = -max_delta_at_v + j * d_delta;
            Input input = {v, delta};
            local_traj_lib_[i][j] = model_.simulateLocalTrajectory(N_, input);
            std::cout << "Simulated trajectory for:     v=" << v << "   delta=" << delta << "\n";
        }
    }
}

void NmpcNavigation::select_local_traj()
{
    // Determine which trajectories are feasible
    double dv = V_MAX / LIB_NUM_V;
    double v_lower_bound = std::max(dv, curr_input_.v - Ts_ * DECCEL_MAX);
    double v_upper_bound = std::min(V_MAX, curr_input_.v + Ts_ * ACCEL_MAX);
    int i_lower = static_cast<int>(v_lower_bound / dv) - 1;
    int i_upper = static_cast<int>(v_upper_bound / dv) - 1;
    
    LocalTrajectory best_traj;
    Eigen::Matrix2d rot_mat;
    rot_mat << cos(curr_state_.phi), -sin(curr_state_.phi),
                sin(curr_state_.phi), cos(curr_state_.phi);
    Eigen::Vector2d t_vec(curr_state_.X, curr_state_.Y);
    double s_best = -100;
    for(int i = i_lower; i <= i_upper; ++i) {
        for(LocalTrajectory& traj : local_traj_lib_[i]) {
            // Transform the endpoint of the traj into the map frame
            Eigen::Vector2d traj_endpoint(traj.X(N_- 1), traj.Y(N_-1));
            traj_endpoint = rot_mat * traj_endpoint + t_vec;
            double s = global_path_.find_s(traj_endpoint(0), traj_endpoint(1));
            if(s > s_best && s <= global_path_.L) {
                s_best = s;
                best_traj = traj;
            }
        }
    }

    // Transform the best traj into the map frame
    Eigen::MatrixXd x_y_points(2, best_traj.X.size());
    x_y_points.row(0) = best_traj.X.transpose();
    x_y_points.row(1) = best_traj.Y.transpose();
    Eigen::MatrixXd transformed_x_y_points = (rot_mat * x_y_points).colwise() + t_vec;          
    
    local_traj_ref_.X = transformed_x_y_points.row(0).transpose();
    local_traj_ref_.Y = transformed_x_y_points.row(1).transpose();
    local_traj_ref_.phi = best_traj.phi.array() + curr_state_.phi;
}

const LocalTrajectory& NmpcNavigation::getLocalTrajectoryRef() const { return local_traj_ref_; }

const std::vector<std::vector<LocalTrajectory>>& NmpcNavigation::getLocalTrajectoryLib() const { return local_traj_lib_; }


} // namespace nmpcnav
