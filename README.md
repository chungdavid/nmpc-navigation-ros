# nmpc_navigation package
A planning pipeline inspired by the following [paper](https://arxiv.org/pdf/1711.07300).

![Demo video](assets/gif.gif)

## The Idea
Given the following inputs:
- Occupancy Grid
- Current State (X, Y, $\phi$)
- Target Positon (x, y)

Generate a global path from the current position to the target position using RRT*. The global path does not need to be optimal, but rather give a general idea of where the robot should go.

Then, given the global path `[(x0, y0), (x1, y1), ..., (xn, yn)]`, select a local trajectory from a library of precomputed trajectories that maximizes progress along that path. The local trajectory should be feasible by considering the dynamics of the robot and map boundaries.

The local trajectory is a sequence of states (X, Y, $\phi$) along a horizon N + 1, and is the input to the MPC. The MPC finds the best control input $u$ (consisting of speed and steering commands $v$ and $\delta$) that tracks the trajectory by minimizing the following cost function...
```math
\begin{equation}
\begin{aligned}

\min_{u, x} \;
  & (x_N - x_N^{\text{ref}})^{T} Q_N (x_N - x_N^{\text{ref}}) +
  \sum_{k=0}^{N-1} (x_k - x_k^{\text{ref}})^{T} Q (x_k - x_k^{\text{ref}}) + u_k^{T} R u_k \\

& \text{s.t.} \quad \\
& x_0 = x(0) \\
& x_{k+1} = A_k x_k + B_k u_k, \quad k = 0, \dots, N-1, \\
& x \in X_{map}, \quad k = 0, \dots, N, \\
& u_{\min} \le u_k \le u_{\max}, \quad k = 0, \dots, N-1.

\end{aligned}
\end{equation}
```

which creates a standard quadratic program:
```math
\begin{aligned}
\min_{\mathbf{z}} \quad & \frac{1}{2} \mathbf{z}^T H \mathbf{z} + \mathbf{g}^T \mathbf{z} \\
\text{s.t.} \quad & \mathbf{l}_\mathrm{b} \le A \mathbf{z} \le \mathbf{u}_\mathrm{b}
\end{aligned}
```

The robot's behaviour is governed by a state machine:
1. Idle/Waiting
    - Stationary, no target set
    - Wait for new goal
2. Follow the global path
    - Generate local trajectory and execute
3. At target position
    - Transition to state 1
4. Local trajectory recovery
    - Triggered when no feasible local trajectory found
    - Reverse, rotate in place, etc. 
5. Recompute global path
    - Triggered if recovery fails
    - Transition to state 2
6. Abort
    - No global path can be recomputed
7. Emergency stop
    - Triggered by safety events

## Implementation
The idea was to plan a global path when a target position is published to the `/target_pos` topic. However, the global planner was not implemented and a precomputed global path was loaded by setting `use_csv_global_path: true` and `global_path_csv_filename: path/to/csv` in the `config.yaml`. Drive commands are published to the `/drive` topic. The state machine is also not implemented.  

## Environment Setup
**ROS Version**: Foxy <br>
**System Requirements**: Ubuntu 20.04 (or with Docker)

Clone and set up the F1Tenth Gym ROS environment.
```bash
git clone https://github.com/f1tenth/f1tenth_gym_ros.git
```
If using their Docker setup, make sure to mount this repository to the src/ folder of your ROS2 workspace inside the container. Your folder structure should look like this:
```
sim_ws/
└─ src/
   ├─ f1_tenth_gym_ros/     # F1Tenth Gym repo
   └─ nmpc_navigation/      # this repo
```

Make sure you activate the ROS2 environment with `source /opt/ros/foxy/setup.bash`, then build the packages as usual:
```bash
cd sim_ws/
colcon build
source install/setup.bash
```

## Running the Packages
Launch both the F1Tenth simulator and this package:
```bash
ros2 launch nmpc_navigation nmpc_navigation_launch.py
ros2 launch f1tenth_gym_ros gym_bridge_launch.py
```
The parameters in the `config.yaml` file of this package can be modified.

