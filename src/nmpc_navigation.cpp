#include "nmpc_navigation/nmpc_navigation.hpp"

#include <iostream>
#include "OsqpEigen/OsqpEigen.h"
#include "nmpc_navigation/grid_utils.hpp"

namespace nmpcnav {


NmpcNavigation::NmpcNavigation(Config config)
    : Ts_(config.Ts), N_(config.N),
      V_MAX(config.v_max), DELTA_MAX(config.delta_max), ACCEL_MAX(config.accel_max),
      DECCEL_MAX(config.deccel_max), YAW_RATE_MAX(config.yaw_rate_max),
      LIB_NUM_V(config.lib_num_v), LIB_NUM_DELTA(config.lib_num_delta),
      L_F(config.l_f), L_R(config.l_r), curr_input_{0.0, 0.0},
      model_(config.Ts, config.l_f, config.l_r),
      local_traj_lib_(config.lib_num_v, std::vector<LocalTrajectory>(config.lib_num_delta + 1)),
      map_(nullptr), has_target_pos_(false)
{
    create_local_traj_lib();
    QN_.diagonal() << config.QN_X, config.QN_Y, config.QN_phi, 
    Qk_.diagonal() << config.Qk_X, config.Qk_Y, config.Qk_phi;
    R_.diagonal() << config.R_v, config.R_delta;
    std::cout << "NmpcNavigation initialized! \n"; 
}

NmpcNavigation::~NmpcNavigation() { }

void NmpcNavigation::setGlobalPath(const std::vector<double>& x_in, const std::vector<double>& y_in) {
    global_path_ = GlobalPath(x_in, y_in);
    target_pos_.X = x_in.back();
    target_pos_.Y = y_in.back();
    has_target_pos_ = true;
}

const GlobalPath& NmpcNavigation::getGlobalPath() const { return global_path_; }

void NmpcNavigation::setTargetPosition(double target_x, double target_y) {
    if(!map_) {
        std::cout << "A map (occupancy grid) does not exist. Unable to set target position." << std::endl;
        return;
    }
    target_pos_.X = target_x;
    target_pos_.Y = target_y;

    // TODO: generate a global path based on the new position

    has_target_pos_ = true;
}

Input NmpcNavigation::runNavigationPipeline(const State& x0) {
    curr_state_ = x0; // Update the state of the vehicle
    if(has_target_pos_) {
        select_local_traj(); // Update the trajectory reference
        run_mpc(); // Use mpc to track reference
    }
    return curr_input_;
}

void NmpcNavigation::create_local_traj_lib() {
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
            // std::cout << "Simulated trajectory for:     v=" << v << "   delta=" << delta << "\n";
        }
    }
}

void NmpcNavigation::select_local_traj() {
    // Determine which trajectories are feasible
    double dv = V_MAX / LIB_NUM_V;
    double v_lower_bound = std::max(dv, curr_input_.v - Ts_ * DECCEL_MAX);
    double v_upper_bound = std::min(V_MAX, curr_input_.v + Ts_ * ACCEL_MAX);
    int i_lower = static_cast<int>(v_lower_bound / dv) - 1;
    int i_upper = static_cast<int>(v_upper_bound / dv) - 1;
    i_lower = std::max(0, i_lower);
    i_upper = std::min(LIB_NUM_V - 1, i_upper);
    
    LocalTrajectory best_traj;
    bool found = false;

    Eigen::Matrix2d rot_mat;
    rot_mat << cos(curr_state_.phi), -sin(curr_state_.phi),
                sin(curr_state_.phi), cos(curr_state_.phi);
    Eigen::Vector2d t_vec(curr_state_.X, curr_state_.Y);
    double s_best = -std::numeric_limits<double>::infinity();

    Eigen::Matrix<double, 2, Eigen::Dynamic> x_y_points(2, N_ + 1);
    Eigen::Matrix<double, 2, Eigen::Dynamic> traj_map(2, N_ + 1);
    Eigen::VectorXd traj_map_X(N_ + 1);
    Eigen::VectorXd traj_map_Y(N_ + 1);

    for(int i = i_lower; i <= i_upper; ++i) {
        for(const LocalTrajectory& traj : local_traj_lib_[i]) {
            // Transform each trajectory to map frame
            x_y_points.row(0) = traj.X.transpose();
            x_y_points.row(1) = traj.Y.transpose();

            Eigen::MatrixXd traj_map = (rot_mat * x_y_points).colwise() + t_vec;  
            Eigen::VectorXd traj_map_X = traj_map.row(0).transpose();
            Eigen::VectorXd traj_map_Y = traj_map.row(1).transpose();
            
            if(map_ && !gridutils::isTrajectoryCollisionFree(*map_, traj_map_X, traj_map_Y, 0.20)) {
                continue;
            }

            double s = global_path_.find_s(traj_map_X(N_), traj_map_Y(N_));
            if(s > s_best && s <= global_path_.L) {
                s_best = s;
                best_traj.X = traj_map_X;
                best_traj.Y = traj_map_Y;
                best_traj.phi = traj.phi.array() + curr_state_.phi;
                best_traj.input_ref = traj.input_ref;
                found = true;
            }
        }
    }  
    if(found) {
        local_traj_ref_ = std::move(best_traj);
    } else {
        std::cout << "Local trajectory not found..." << std::endl;

        if(local_traj_ref_.X.size() == N_ + 1
           && local_traj_ref_.Y.size() == N_ + 1
           && local_traj_ref_.phi.size() == N_ + 1) {

            local_traj_ref_.X.head(N_) = local_traj_ref_.X.tail(N_);
            local_traj_ref_.Y.head(N_) = local_traj_ref_.Y.tail(N_);
            local_traj_ref_.phi.head(N_) = local_traj_ref_.phi.tail(N_);

            local_traj_ref_.X(N_) = local_traj_ref_.X(N_ - 1);
            local_traj_ref_.Y(N_) = local_traj_ref_.Y(N_ - 1);
            local_traj_ref_.phi(N_) = local_traj_ref_.phi(N_ - 1);
        } else {
            local_traj_ref_.X = Eigen::VectorXd::Constant(N_ + 1, curr_state_.X);
            local_traj_ref_.Y = Eigen::VectorXd::Constant(N_ + 1, curr_state_.Y);
            local_traj_ref_.phi = Eigen::VectorXd::Constant(N_ + 1, curr_state_.phi);
        }
    }
    
}

const LocalTrajectory& NmpcNavigation::getLocalTrajectoryRef() const { return local_traj_ref_; }

const std::vector<std::vector<LocalTrajectory>>& NmpcNavigation::getLocalTrajectoryLib() const { return local_traj_lib_; }

// TODO synchronize with mutex if setMap and select_local_traj access map_ concurrently
void NmpcNavigation::setMap(const nav_msgs::msg::OccupancyGrid::ConstSharedPtr map) { map_ = map; }

void NmpcNavigation::run_mpc() {
    const int n_state_vars = NX * (N_ + 1);
    const int n_input_vars = NU * N_;
    const int n_decision_vars = n_state_vars + n_input_vars;
    const int n_constraints = NX * (N_ + 1) + NX * (N_ + 1) + NU * N_;

    // Hessian
    Eigen::SparseMatrix<double> hessian_matrix(n_decision_vars, n_decision_vars);
    hessian_matrix.reserve(Eigen::VectorXi::Constant(n_decision_vars, 1));

    for(int i = 0; i < n_state_vars - NX; ++i) {
        int pos_Q = i % NX;
        double val = 2.0 * Qk_.diagonal()[pos_Q];
        if(val != 0) {
            hessian_matrix.insert(i, i) = val;
        }
    }
    // terminal (N + 1) state
    for (int i = 0; i < NX; ++i) {
        double val = 2.0 * QN_.diagonal()(i);
        if (val != 0) {
            hessian_matrix.insert(n_state_vars - NX + i, n_state_vars - NX + i) = val;
        }
    }

    for(int i = 0; i < n_input_vars; ++i) {
        int pos_R = i % NU;
        double val = 2.0 * R_.diagonal()[pos_R];
        if(val != 0) {
            hessian_matrix.insert(i + n_state_vars, i + n_state_vars) = val;
        }
    }

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(n_decision_vars);
    Eigen::SparseMatrix<double> constraints_matrix(n_constraints, n_decision_vars);
    constraints_matrix.reserve(Eigen::VectorXi::Constant(n_constraints, 1 + NX + NX));
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(n_constraints);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(n_constraints);

    Eigen::Matrix<double, NX, NX> A_d;
    Eigen::Matrix<double, NX, NU> B_d;
    Eigen::Matrix<double, NX, 1> g_d;

    Eigen::VectorXd x_k_ref(NX);
    Eigen::VectorXd u_k_ref(NU);
    u_k_ref << local_traj_ref_.input_ref.v, local_traj_ref_.input_ref.delta;

    Eigen::VectorXd u_k_min(NU);
    Eigen::VectorXd u_k_max(NU);
    u_k_min << 0.0, -DELTA_MAX;
    u_k_max << V_MAX, DELTA_MAX;
    
    // First constraint
    for(int i = 0; i < NX; ++i) {
        constraints_matrix.insert(i, i) = 1.0;
    }
    lower_bound.segment<NX>(0) << curr_state_.X, curr_state_.Y, curr_state_.phi;
    upper_bound.segment<NX>(0) << curr_state_.X, curr_state_.Y, curr_state_.phi;

    for(int i = 0; i < N_; ++i) {
        x_k_ref << local_traj_ref_.X(i), local_traj_ref_.Y(i), local_traj_ref_.phi(i);
        model_.getLinearModel(A_d, B_d, g_d, x_k_ref, u_k_ref);

        // Populate constraints matrix with A_d
        for (int j = 0; j < NX; ++j) { 
            for (int k = 0; k < NX; ++k) {
                const double val = -A_d(j, k);
                if (val != 0) {
                    constraints_matrix.insert(NX * (i + 1) + j, NX * i + k) = val;
                }
            }
        }
        // Populate constrainsts matrix with B_d
        for (int j = 0; j < NX; ++j) { 
            for (int k = 0; k < NU; ++k) {
                double val = -B_d(j, k);
                if (val != 0) {
                    constraints_matrix.insert(NX * (i + 1) + j, NU * i + k + n_state_vars) = val;
                }
            }
        }

        // 1s along the diagonal
        for (int j = 0; j < NX; ++j) {
            constraints_matrix.insert(NX * (i + 1) + j, NX * (i + 1) + j) = 1.0;
        }

        // Equality bounds for dynamics constraints
        lower_bound.segment<NX>(NX * (i + 1)) = g_d;
        upper_bound.segment<NX>(NX * (i + 1)) = g_d;

        // First N entries of gradient
        gradient.segment<NX>(i * NX) = - 2.0 * Qk_ * x_k_ref;
    }
    // (N + 1) entry of gradient (for terminal state) 
    x_k_ref << local_traj_ref_.X(N_), local_traj_ref_.Y(N_), local_traj_ref_.phi(N_);
    gradient.segment<NX>(n_state_vars - NX) = - 2.0 * QN_ * x_k_ref;

    // State and input bounds
    Eigen::VectorXd x_k_min(NX);
    Eigen::VectorXd x_k_max(NX);
    x_k_min << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
    x_k_max << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;

    for(int i = 0; i < N_ + 1; ++i) {
        lower_bound.segment<NX>(n_state_vars + NX * i) = x_k_min;
        upper_bound.segment<NX>(n_state_vars + NX * i) = x_k_max;
    }

    Eigen::VectorXd u0_min(NU);
    Eigen::VectorXd u0_max(NU);
    double v0_min = std::max(V_MAX / LIB_NUM_V, curr_input_.v - Ts_ * DECCEL_MAX);
    double v0_max = std::min(V_MAX, curr_input_.v + Ts_ * ACCEL_MAX);
    v0_max = std::max(v0_max, 1e-3);
    v0_min = std::min(v0_min, v0_max);
    double delta_max = std::atan(YAW_RATE_MAX * (L_F + L_R) / v0_max);

    u0_min << v0_min, std::max(-DELTA_MAX, -delta_max);
    u0_max << v0_max, std::min(DELTA_MAX, delta_max);
    lower_bound.segment<NU>(n_state_vars + n_state_vars) = u0_min;
    upper_bound.segment<NU>(n_state_vars + n_state_vars) = u0_max;

    for(int i = 1; i < N_; ++i) {
        lower_bound.segment<NU>(n_state_vars + n_state_vars + NU * i) = u_k_min;
        upper_bound.segment<NU>(n_state_vars + n_state_vars + NU * i) = u_k_max;
    }

    for(int i = 0; i < n_decision_vars; ++i) {
        constraints_matrix.insert(n_state_vars + i, i) = 1.0;
    }

    // std::cout << "Dimensions of Hessian: " << hessian_matrix.rows() << "x" << hessian_matrix.cols() << std::endl;  
    // std::cout << "Dimensions of Constraints Matrix: " << constraints_matrix.rows() << "x" << constraints_matrix.cols() << std::endl; 
    // std::cout << "Dimensions of Gradient: " << gradient.rows() << "x" << gradient.cols() << std::endl;
    // std::cout << "Dimensions of lower bound: " << lower_bound.rows() << "x" << lower_bound.cols() << std::endl;
    // std::cout << "Dimensions of upper bound: " << upper_bound.rows() << "x" << upper_bound.cols() << std::endl;

    constraints_matrix.makeCompressed();
    hessian_matrix.makeCompressed();

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(n_decision_vars);
    solver.data()->setNumberOfConstraints(n_constraints);
    if (!solver.data()->setHessianMatrix(hessian_matrix))
        throw "Failed set Hessian";
    if (!solver.data()->setGradient(gradient))
        throw "Failed to set Gradient";
    if (!solver.data()->setLinearConstraintsMatrix(constraints_matrix))
        throw "Failed to set Constraints Matrix";
    if (!solver.data()->setLowerBound(lower_bound))
        throw "Failed to set Lower Bounds";
    if (!solver.data()->setUpperBound(upper_bound))
        throw "Failed to set Upper Bounds";
    
    solver.settings()->setVerbosity(false);

    if (!solver.initSolver())
        throw "Failed to initiate solver";

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        throw "Error trying to solve QP";

    Eigen::VectorXd QP_solution;
    QP_solution = solver.getSolution();

    curr_input_.v = QP_solution(NX * (N_ + 1));
    curr_input_.delta = QP_solution(NX * (N_ + 1) + 1);

    // std::cout << "Optimal v: " << curr_input_.v << "   Optimal delta: " << curr_input_.delta << std::endl;
}


} // namespace nmpcnav
