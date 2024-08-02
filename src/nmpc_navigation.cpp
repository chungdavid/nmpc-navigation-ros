#include "nmpc_navigation/nmpc_navigation.hpp"

#include <iostream>
#include "OsqpEigen/OsqpEigen.h"

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
    Q_.diagonal() << config.Q_X, config.Q_Y, config.Q_phi;
    R_.diagonal() << config.R_v, config.R_delta;
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
    run_mpc(); // Use mpc to track reference
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
    local_traj_ref_.input_ref.v = best_traj.input_ref.v;
    local_traj_ref_.input_ref.delta = best_traj.input_ref.delta;
}

const LocalTrajectory& NmpcNavigation::getLocalTrajectoryRef() const { return local_traj_ref_; }

const std::vector<std::vector<LocalTrajectory>>& NmpcNavigation::getLocalTrajectoryLib() const { return local_traj_lib_; }

void NmpcNavigation::run_mpc() {
    // Set hessian matrix
    Eigen::SparseMatrix<double> hessian_matrix(NX * (N_ + 1) + NU * N_, NX * (N_ + 1) + NU * N_);
    for(int i = 0; i < NX * (N_ + 1) + NU * N_; ++i) {
        if(i < NX * (N_ + 1)) {
            int pos_Q = i % NX;
            double val = Q_.diagonal()[pos_Q];
            if(val != 0) {
                hessian_matrix.insert(i, i) = val;
            } 
        } else {
            int pos_R = i % NU;
            double val = R_.diagonal()[pos_R];
            if(val != 0) {
                hessian_matrix.insert(i, i) = val;
            }
        }
    }
    
    // Populate constraints matrix with -1.0 and 1.0
    Eigen::SparseMatrix<double> constraints_matrix(NX * (N_ + 1) + NX * (N_ + 1) + NU * N_, NX * (N_ + 1) + NU * N_);
    for (int i = 0; i < NX * (N_ + 1); ++i) {
        constraints_matrix.insert(i, i) = -1.0;
    }
    for (int i = 0; i < NX * (N_ + 1) + NU * N_; ++i) {
        constraints_matrix.insert(i + NX * (N_ + 1), i) = 1;
    }

    Eigen::VectorXd gradient = Eigen::VectorXd::Zero(NX * (N_ + 1) + NU * N_);
    Eigen::VectorXd lower_bound = Eigen::VectorXd::Zero(NX * (N_ + 1) + NX * (N_ + 1) + NU * N_);
    Eigen::VectorXd upper_bound = Eigen::VectorXd::Zero(NX * (N_ + 1) + NX * (N_ + 1) + NU * N_);

    Eigen::Matrix<double, NX, NX> A_d;
    Eigen::Matrix<double, NX, NU> B_d;
    Eigen::Matrix<double, NX, 1> g_d;

    Eigen::VectorXd u_k_ref(NU);
    u_k_ref << local_traj_ref_.input_ref.v, local_traj_ref_.input_ref.delta;
    Eigen::VectorXd x_k_ref(NX);
    
    for(int i = 0; i < N_ + 1; ++i) {
        x_k_ref << local_traj_ref_.X(i), local_traj_ref_.Y(i), local_traj_ref_.phi(i);
        model_.getLinearModel(A_d, B_d, g_d, x_k_ref, u_k_ref);
        
        // Set gradient vector
        gradient.segment<NX>(i*NX) << Q_ * (-x_k_ref);

        // Populate constraints matrix with A_d
        for (int j = 0; j < NX; ++j) { 
            for (int k = 0; k < NX; ++k) {
                double val = A_d(j, k);
                if (val != 0) {
                    constraints_matrix.insert(NX * (i + 1) + j, NX * i + k) = val;
                }
            }
        }
        // Populate constrainsts matrix with B_d
        if (i < N_) {
            for (int j = 0; j < NX; ++j) { 
                for (int k = 0; k < NU; ++k) {
                    double val = B_d(j, k);
                    if (val != 0) {
                        constraints_matrix.insert(NX * (i + 1) + j, NU * i + k + NX * (N_ + 1)) = val;
                    }
                }
            }
        }

        // TO DO: Calculate lower & upper inequalities for x
        // I just set them to arbitrary values for now
        Eigen::VectorXd x_k_min(NX);
        Eigen::VectorXd x_k_max(NX);
        x_k_min << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY;
        x_k_max << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY;
        lower_bound.segment<NX>(NX * (N_ + 1) + NX * i) << x_k_min;
        upper_bound.segment<NX>(NX * (N_ + 1) + NX * i) << x_k_max;
    }

    // Constraint on initial conditions
    Eigen::VectorXd x0(NX);
    x0 << local_traj_ref_.X(0), local_traj_ref_.Y(0), local_traj_ref_.phi(0);
    lower_bound.head(NX) = -x0;
    upper_bound.head(NX) = -x0;

    Eigen::VectorXd u0_min(NU);
    Eigen::VectorXd u0_max(NU);
    u0_min << std::max(V_MAX / LIB_NUM_V, curr_input_.v - Ts_ * DECCEL_MAX),
             std::max(-DELTA_MAX, -atan(YAW_RATE_MAX * (L_F + L_R) / curr_input_.v));
    u0_max << std::min(V_MAX, curr_input_.v + Ts_ * ACCEL_MAX),
              std::min(DELTA_MAX, atan(YAW_RATE_MAX * (L_F + L_R) / curr_input_.v));

    lower_bound.segment<NU>(NX * (N_ + 1) + NX * (N_ + 1)) << u0_min;
    upper_bound.segment<NU>(NX * (N_ + 1) + NX * (N_ + 1)) << u0_max;

    // Upper and lower input constraints
    Eigen::VectorXd u_k_min(NU);
    Eigen::VectorXd u_k_max(NU);
    u_k_min << 0.0, -DELTA_MAX;
    u_k_max << V_MAX, DELTA_MAX;
    for(int i = 1; i < N_; ++i) {
        lower_bound.segment<NU>(NX * (N_ + 1) + NX * (N_ + 1) + i * NU) << u_k_min;
        upper_bound.segment<NU>(NX * (N_ + 1) + NX * (N_ + 1) + i * NU) << u_k_max;
    }

    // std::cout << "Dimensions of Hessian: " << hessian_matrix.rows() << "x" << hessian_matrix.cols() << std::endl;  
    // std::cout << "Dimensions of Constraints Matrix: " << constraints_matrix.rows() << "x" << constraints_matrix.cols() << std::endl; 
    // std::cout << "Dimensions of Gradient: " << gradient.rows() << "x" << gradient.cols() << std::endl;
    // std::cout << "Dimensions of lower bound: " << lower_bound.rows() << "x" << lower_bound.cols() << std::endl;
    // std::cout << "Dimensions of upper bound: " << upper_bound.rows() << "x" << upper_bound.cols() << std::endl;

    OsqpEigen::Solver solver;
    solver.settings()->setWarmStart(true);
    solver.data()->setNumberOfVariables(NX * (N_ + 1) + NU * N_);
    solver.data()->setNumberOfConstraints(2 * NX * (N_ + 1) + NU * N_);
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

    if (!solver.initSolver())
        throw "Failed to initiate solver";

    if (solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError)
        throw "Error trying to solve QP";

    Eigen::VectorXd QP_solution;
    QP_solution = solver.getSolution();

    curr_input_.v = QP_solution(NX * (N_ + 1));
    curr_input_.delta = QP_solution(NX * (N_ + 1) + 1);

    std::cout << "Optimal v: " << curr_input_.v << "   Optimal delta: " << curr_input_.delta << std::endl;
}


} // namespace nmpcnav
