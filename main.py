#!/usr/bin/env python3
import torch
import time
import numpy                as np
import matplotlib.pyplot    as plt

from matplotlib             import animation
from scipy.linalg           import solve_continuous_are
from gbp.core               import GBPSettings
from controller             import GBPController
from trajectory             import generate_circle_trajectory, generate_line_trajectory
from plotting               import TrajectoryPlotter, plot_trajectories, animate_trajectories, analyze_collisions, analyze_tracking

# ---------------------------------------------------------------------------------- 
#                                   SIMULATION AND MODEL
# ----------------------------------------------------------------------------------
# ----------------------------------------
# Simulation parameters
# ----------------------------------------
dt = 0.01
T = 20.0
ts = np.arange(0, T, dt)
horizon = len(ts) - 1

# ----------------------------------------
# Double-integrator dynamics
# State     : x = [x_pos, y_pos, x_vel, y_vel]
# Input     : u = [u_x, u_y]
# Dynamics  : x_dot = Ax + Bu
# ----------------------------------------
A = np.array([[0, 0, 1, 0],
              [0, 0, 0, 1],
              [0, 0, 0, 0],
              [0, 0, 0, 0]])
B = np.array([[0, 0],
              [0, 0],
              [1, 0],
              [0, 1]])

# ----------------------------------------
# Circular reference trajectory generation
# ----------------------------------------
r_1, omega_1, center_1 = 5.0, 0.2, np.array([0.0, 0.0])
ref_traj_1 = generate_circle_trajectory(r_1, omega_1, ts, center_1)

r_2, omega_2, center_2 = 4.0, -0.15, np.array([-2.0, 2.0])
ref_traj_2 = generate_circle_trajectory(r_2, omega_2, ts, center_2)

x0_1 = np.array([center_1[0] + r_1, 0, 0, 0])
x0_2 = np.array([center_2[0] + r_2, center_2[0] + r_2, 0, 0])

# ----------------------------------------
# Line reference trajectory generation
# ----------------------------------------

ref_traj_1 = generate_line_trajectory(
    start_pos=np.array([5.0, 0.1]),
    end_pos=np.array([-5.0, 0.1]),
    ts=ts
)
ref_traj_2 = generate_line_trajectory(
    start_pos=np.array([-5.0,  -0.1]),
    end_pos=np.array([5.0, -0.1]),
    ts=ts
)

x0_1 = np.array([5.0+1.0, 0.1+0.6, 0.0, 0.0])
x0_2 = np.array([-5.0-0.4, -0.1-1.0, 0.0, 0.0])

# ----------------------------------------
# GBP controller parameters
# ----------------------------------------
Q_gbp = np.diag([10, 10, 1, 1])
R_gbp = np.eye(2)
S_gbp = solve_continuous_are(A, B, Q_gbp, R_gbp)

settings = GBPSettings(
    damping=0.1,            # Reduced damping for faster convergence after avoidance
    beta=0.05,              # Increased threshold to reduce excessive relinearization
    num_undamped_iters=3,   # More undamped iterations for better convergence
    min_linear_iters=5,     # Wait longer before relinearizing to avoid oscillations
    dropout=0.0,            # no message dropout
    reset_iters_since_relin=[],
    type=torch.float
)

robot_radius = 0.6              
safety_eps = 0.2*robot_radius   # Reduced safety margin to make avoidance less aggressive         

sigma_0                 = 1e-15 # Initial state uncertainty: Small to give initial state a strong prior
sigma_pos               = 1e-2  # Reduced: Stronger position tracking (was 1e2)
sigma_vel               = 1e2   # Reduced: Stronger velocity tracking (was 1e2)
sigma_u                 = 1e2   # Input uncertainty: Large to allow flexibility in control inputs
sigma_dynamics          = 1e-6  # Dynamics uncertainty: Small to prioritize model accuracy
sigma_collision         = 1e-4  # Increased: Make collision avoidance less rigid (was 1e-4)

# ----------------------------------------
# Create GBP controller
# ----------------------------------------
Controller = GBPController(A=A,
                           B=B,
                           horizon=horizon,
                           dt=dt,
                           sigma_collision=sigma_collision,
                           robot_radius=robot_radius,
                           safety_eps=safety_eps,
                           settings=settings)

# Add agents to the controller
Controller.add_agent(Q=Q_gbp,
                     R=R_gbp,
                     S=Q_gbp,
                     ref_traj=ref_traj_1,
                     x0=x0_1,
                     sigma_0=sigma_0,
                     sigma_pos=sigma_pos,
                     sigma_vel=sigma_vel,
                     sigma_u=sigma_u,
                     sigma_dynamics=sigma_dynamics)

Controller.add_agent(Q=Q_gbp,
                     R=R_gbp,
                     S=Q_gbp,
                     ref_traj=ref_traj_2,
                     x0=x0_2,
                     sigma_0=sigma_0,
                     sigma_pos=sigma_pos,
                     sigma_vel=sigma_vel,
                     sigma_u=sigma_u,
                     sigma_dynamics=sigma_dynamics)

# Add the inter-agent collision factor
Controller.add_inter_agent_collision()

# Solve the GBP controller
x_list, u_list = Controller.solve(n_iters=10, converged_threshold=1e-3)

x_1, x_2 = x_list
u_1, u_2 = u_list


# ----------------------------------------------------------------------------------
#                                   PLOTTING RESULTS                                  
# ----------------------------------------------------------------------------------

# Add collision analysis using the new plotting module
collision_info = analyze_collisions(x_list, ts, robot_radius, safety_eps)

# Create plot data structure for the new plotting module
plot_data = {
    'times': ts,
    'trajectories': [x_1[:, :2], x_2[:, :2]],  # Extract position data
    'targets': [ref_traj_1, ref_traj_2],
    'obstacles': [],  # No obstacles in this scenario
    'velocities': [x_1[:, 2:], x_2[:, 2:]],  # Extract velocity data
    'controls': [u_1, u_2],
    'costs': [],  # No cost tracking in this version
    'robot_radius': robot_radius,
    'collision_info': collision_info
}

# Use the new generalized plotting functions
fig_static = plot_trajectories(
    ref_trajs=[ref_traj_1, ref_traj_2],
    actual_trajs=[x_1[:, :2], x_2[:, :2]],  # Extract position data
    robot_radius=robot_radius,
    title="Multi-Robot Trajectory Tracking with Collision Avoidance",
    save_plots=True,
    save_dir='plots/'
)

# Create animation
anim = animate_trajectories(
    ref_trajs=[ref_traj_1, ref_traj_2],
    actual_trajs=[x_1[:, :2], x_2[:, :2]],  # Extract position data
    ts=ts,
    dt=dt,
    robot_radius=robot_radius,
    title="Multi-Robot Animation with Collision Avoidance",
    save_plots=False,
    save_dir='plots/'
)

# Analyze tracking performance (create this using the comprehensive method)
tracking_analysis = analyze_tracking(
    ts=ts,
    trajectories=[x_1[:, :2], x_2[:, :2]],  # Extract position data
    ref_trajectories=[ref_traj_1, ref_traj_2],
    controls=[u_1, u_2],
    dt=dt,
    save_plots=True,
    save_dir='plots/'
)
