#pragma once

#include <nav_msgs/msg/occupancy_grid.hpp>

#include <Eigen/Dense>

#include <cmath>

namespace gridutils {

inline bool worldToGrid(const nav_msgs::msg::OccupancyGrid& grid,
                 double x_w, double y_w,
                 int& x_g, int& y_g) {

    const double res = grid.info.resolution;
    if(res <= 0.0) {
        return false;
    }

    const double ox = grid.info.origin.position.x;
    const double oy = grid.info.origin.position.y;
    
    x_g = static_cast<int>(std::floor((x_w - ox) / res));
    y_g = static_cast<int>(std::floor((y_w - oy) / res));

    if(x_g < 0 || y_g < 0) {
        return false;
    }
    if(x_g >= static_cast<int>(grid.info.width) || y_g >= static_cast<int>(grid.info.height)) {
        return false;
    }
    return true;
}

inline bool isCellOccupied(const nav_msgs::msg::OccupancyGrid& grid,
                    int x_g, int y_g, int threshold) {
    
    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);

    if(w <= 0 || h <= 0) {
        return true;
    }
    if(x_g < 0 || y_g < 0 || x_g >= w || y_g >= h) {
        return true;
    }
    if(static_cast<size_t>(w) * static_cast<size_t>(h) > grid.data.size()) {
        return true;
    }

    const int idx = y_g * w + x_g;
    const int val = static_cast<int>(grid.data[static_cast<size_t>(idx)]);
    return val >= threshold || val < 0;
}

inline bool isCircleCollisionFree(const nav_msgs::msg::OccupancyGrid& grid,
                           double x_w, double y_w, double radius,
                           int threshold = 50) {

    const double res = grid.info.resolution;
    if(res <= 0.0) {
        return false;
    }

    int cx_g, cy_g; // index of the circle center in grid
    if(!worldToGrid(grid, x_w, y_w, cx_g, cy_g)) { // outside of map is unsafe
        return false;
    }

    const int radius_cells = static_cast<int>(std::ceil(radius / res));
    const double radius_sq = radius * radius;

    for(int dy = -radius_cells; dy <= radius_cells; ++dy) {
        for(int dx = -radius_cells; dx <= radius_cells; ++dx) {

            const double dist_sq = (dx * res) * (dx * res) + (dy * res) * (dy * res);
            if(dist_sq > radius_sq) {
                continue;
            }

            const int x_g = cx_g + dx;
            const int y_g = cy_g + dy;
            
            if(isCellOccupied(grid, x_g, y_g, threshold)) {
                return false;
            }
        }
    }
    return true;
}

inline bool isTrajectoryCollisionFree(const nav_msgs::msg::OccupancyGrid& grid,
                               const Eigen::VectorXd& X,
                               const Eigen::VectorXd& Y,
                               double radius = 1.0,
                               int threshold = 50) {
    
    const int n = static_cast<int>(X.size());
    if(n <= 0 || Y.size() != n) {
        return false;
    }

    const double res = grid.info.resolution;
    if(res <= 0.0) {
        return false;
    }

    const double ds = std::max(0.5 * res, 0.25 * std::max(radius, 0.0));
    if(!isCircleCollisionFree(grid, X(0), Y(0), radius, threshold)) {
        return false;
    }

    for(int i = 1; i < n; ++i) {
        const double x0 = X(i - 1);
        const double y0 = Y(i - 1);
        const double x1 = X(i);
        const double y1 = Y(i);

        const double dx = x1 - x0;
        const double dy = y1 - y0;
        const double L = std::sqrt(dx * dx + dy * dy);

        if(L <= 0.0) {
            if(!isCircleCollisionFree(grid, x1, y1, radius, threshold)) {
                return false;
            }
            continue;
        }

        const int steps = std::max(1, static_cast<int>(std::ceil(L / ds)));
        for(int k = 1; k <= steps; ++k) {
            const double t = static_cast<double>(k) / static_cast<double>(steps);
            const double x_w = x0 + t * dx;
            const double y_w = y0 + t * dy;

            if(!isCircleCollisionFree(grid, x_w, y_w, radius, threshold)) {
                return false;
            }
        }
    }
    return true;
}

inline void inflateGrid(nav_msgs::msg::OccupancyGrid& grid,
                 double inflation_radius_m,
                 int threshold = 50) {

    const int w = static_cast<int>(grid.info.width);
    const int h = static_cast<int>(grid.info.height);
    const double res = grid.info.resolution;

    if(w <= 0 || h <= 0) {
        return;
    }
    if(res <= 0.0) {
        return;
    }

    const size_t num_cells = static_cast<size_t>(w) * static_cast<size_t>(h);
    if(num_cells > grid.data.size()) {
        return;
    }
    if(inflation_radius_m <= 0.0) {
        return;
    }

    const int r_cells = static_cast<int>(std::ceil(inflation_radius_m / res));
    const double r2 = inflation_radius_m * inflation_radius_m;

    auto idx = [w](int x, int y) -> std::size_t {
        return static_cast<std::size_t>(y * w + x);
    };

    std::vector<std::pair<int,int>> offsets;
    offsets.reserve(static_cast<std::size_t>((2 * r_cells + 1) * (2 * r_cells + 1)));

    for(int dy = -r_cells; dy <= r_cells; ++dy) {
        for(int dx = -r_cells; dx <= r_cells; ++dx) {
            const double x_m = static_cast<double>(dx) * res;
            const double y_m = static_cast<double>(dy) * res;
            if(x_m * x_m + y_m * y_m <= r2) {
                offsets.emplace_back(dx, dy);
            }
        }
    }

    std::vector<int8_t> out(grid.data.begin(), grid.data.begin() + num_cells);

    for(int y = 0; y < h; ++y) {
        for(int x = 0; x < w; ++x) {

            const int8_t v = grid.data[idx(x, y)];
            if(v < threshold) {
                continue;
            }

            for(const auto& off : offsets) {
                const int nx = x + off.first;
                const int ny = y + off.second;
                if(nx < 0 || ny < 0 || nx >= w || ny >= h) continue;

                out[idx(nx, ny)] = 100;
            }
        }
    }

    grid.data.assign(out.begin(), out.end());
}


} // namespace gridutils