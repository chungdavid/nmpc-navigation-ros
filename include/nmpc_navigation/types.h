#pragma once

#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>
#include <vector>
#include <iostream>

namespace nmpcnav {


constexpr int NX = 3; // number of states
constexpr int NU = 2; // number of inputs

struct State {
    double X; // x position in map frame
    double Y; // y position in map frame
    double phi; // angle relative to x-axis in map frame

    void unwrap_phi() {
        if (phi > M_PI) {
            phi -= 2.0 * M_PI;
        } else if (phi < -M_PI) {
            phi += 2.0 * M_PI;
        }
    }
};

struct Input {
    double v; // speed command
    double delta; // steering angle command
};

struct TargetPosition {
    double X; // x position in map frame
    double Y; // y position in map frame
};

struct LocalTrajectory {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd phi;
    Input input_ref;

    LocalTrajectory() : X(0), Y(0), phi(0) {};
    LocalTrajectory(int N) : X(N), Y(N), phi(N) {};
};

struct GlobalPath {
    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    int num_points;
    Eigen::VectorXd s;
    double L;
    
    GlobalPath() : X(0), Y(0), num_points(0), s(0), L(0) { }

    GlobalPath(std::vector<double> x_in, std::vector<double> y_in) {
        X = Eigen::Map<Eigen::VectorXd>(x_in.data(), x_in.size());
        Y = Eigen::Map<Eigen::VectorXd>(y_in.data(), y_in.size());
        num_points = x_in.size();
        s = Eigen::VectorXd::Zero(num_points);
        for(int i = 0; i < num_points - 1; ++i) {
            double dx = x_in[i+1] - x_in[i];
            double dy = y_in[i+1] - y_in[i];
            s(i + 1) = s(i) + std::sqrt(dx*dx + dy*dy);
        }
        L = s(num_points - 1);
    }

    double find_s(double pX_in, double pY_in) {
        Eigen::ArrayXd dx_all = X.array() - pX_in;
        Eigen::ArrayXd dy_all = Y.array() - pY_in;
        Eigen::ArrayXd dist_squared_all = dx_all.square() + dy_all.square();
        std::vector<double> dist_squared_all_vec(dist_squared_all.data(), dist_squared_all.data() + dist_squared_all.size());
        auto min_iter = std::min_element(dist_squared_all_vec.begin(), dist_squared_all_vec.end());
        int p_closest_idx = std::distance(dist_squared_all_vec.begin(), min_iter);

        double s_closest = s(p_closest_idx);
        Eigen::Vector2d p_in(pX_in, pY_in);
        Eigen::Vector2d p_closest(X(p_closest_idx), Y(p_closest_idx));

        int p_next_closest_idx;
        if(p_closest_idx == 0) { // If p(X,Y) is closest to the first point in the global path
            p_next_closest_idx = p_closest_idx + 1;
        } else if(p_closest_idx == num_points - 1) {  // If p(X,Y) is closest to the last point in the global path
            p_next_closest_idx = p_closest_idx - 1;
        } else if(dist_squared_all(p_closest_idx + 1) <= dist_squared_all(p_closest_idx - 1)) {
            p_next_closest_idx = p_closest_idx + 1;
        } else {
            p_next_closest_idx = p_closest_idx - 1;
        }

        Eigen::Vector2d p_next_closest(X(p_next_closest_idx), Y(p_next_closest_idx));
        double proj = (p_next_closest - p_closest).dot(p_in - p_closest) / (p_next_closest - p_closest).norm();
        
        if(p_next_closest_idx > p_closest_idx) {
            return s_closest + proj;
        } else {
            return s_closest - proj;
        }
    }
};

struct Model {
    double Ts;
    double L_F;
    double L_R;

    Model(double Ts, double l_f, double l_r)
        : Ts(Ts), L_F(l_f), L_R(l_r) {}

    // Simulate a trajectory in the vehicle's coordinate frame
    LocalTrajectory simulateLocalTrajectory(int N, Input input) {
        LocalTrajectory traj(N + 1);
        double dx, dy;
        double dphi = input.v * tan(input.delta) / (L_F + L_R);

        // Populate the first point in the trajectory
        traj.X(0) = 0;
        traj.Y(0) = 0;
        traj.phi(0) = 0;

        // Populate the rest of the trajectory from second point to N
        for(int i = 0; i < N; ++i) {
            dx = input.v * cos(traj.phi(i));
            dy = input.v * sin(traj.phi(i));
            traj.X(i+1) = traj.X(i) + Ts*dx;
            traj.Y(i+1) = traj.Y(i) + Ts*dy;
            traj.phi(i+1) = traj.phi(i) + Ts*dphi;
        }

        traj.input_ref = input;
        return traj;
    }

    void getLinearModel(Eigen::Matrix<double,NX,NX>& A_d, Eigen::Matrix<double,NX,NU>& B_d, Eigen::Matrix<double,NX,1>& g_d, const Eigen::Matrix<double,NX,1>& x_bar, const Eigen::Matrix<double,NU,1>& u_bar) {
        // Linearized kinematic bicycle model
        // Get matrices A, B, g where x_dot = Ax + Bu + g
        Eigen::MatrixXd A(NX, NX);
        Eigen::MatrixXd B(NX, NU);
        A << 0, 0, -u_bar(0) * sin(x_bar(2)),
             0, 0, u_bar(0) * cos(x_bar(2)),
             0, 0, 0;
        B << cos(x_bar(2)), 0,
             sin(x_bar(2)), 0,
             tan(u_bar(1)) / (L_F + L_R), u_bar(0) / (cos(u_bar(1)) * cos(u_bar(1)) * (L_F + L_R));

        // Eigen::MatrixXd x_bar_vec(NX, 1);
        // Eigen::MatrixXd u_bar_vec(NU, 1);
        Eigen::MatrixXd f(NX, 1);
        Eigen::MatrixXd g(NX, 1);
        // x_bar_vec << x_bar(0), x_bar(1), x_bar.phi;
        // u_bar_vec << u_bar.v, u_bar.delta;
        f << u_bar(0) * cos(x_bar(2)), u_bar(0) * sin(x_bar(2)), u_bar(0) * tan(u_bar(1)) / (L_F + L_R);
        g << f - (A * x_bar + B * u_bar);

        // Discretize
        Eigen::MatrixXd aux(NX + NU + 1, NX + NU + 1);
        aux.setZero();

        aux.block(0, 0, NX, NX) = A;
        aux.block(0, NX, NX, NU) = B;
        aux.block(0, NX + NU, NX, 1) = g;

        aux *= Ts;
        Eigen::MatrixXd exp_aux = aux.exp();

        A_d = exp_aux.block(0, 0, NX, NX);
        B_d = exp_aux.block(0, NX, NX, NU);
        g_d = exp_aux.block(0, NX + NU, NX, 1);
    } 
};

struct Config {
    double v_max;
    double delta_max;
    double accel_max;
    double deccel_max;
    double yaw_rate_max;
    double l_f;
    double l_r;
    double Ts;
    int N;
    int lib_num_v;
    int lib_num_delta;
    double Q_X;
    double Q_Y;
    double Q_phi; 
    double R_v;
    double R_delta; 
};


} // namespace nmpcnav