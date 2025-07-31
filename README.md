# Unified Autonomy Stack via Gaussian Belief Propagation (GBP)

A multi-robot trajectory tracking and collision avoidance system using Gaussian Belief Propagation (GBP) for distributed optimization and control.

## Overview

This project implements a unified framework for multi-robot trajectory tracking with collision avoidance using Gaussian Belief Propagation. The system supports both traditional tracking and sensing-based adaptive control, making it suitable for real-time multi-robot coordination scenarios.

### Key Features

- **Multi-Robot Trajectory Tracking**: Simultaneous control of multiple robots with reference trajectory following
- **Collision Avoidance**: Inter-robot collision avoidance with configurable safety margins
- **Gaussian Belief Propagation**: Distributed optimization using GBP for scalable multi-robot coordination
- **Sensing Integration**: Optional sensor-based adaptive control for dynamic environments
- **Comprehensive Visualization**: Advanced plotting and animation tools for trajectory analysis
- **Flexible Trajectory Generation**: Support for circular and linear reference trajectories

## System Architecture

### Core Components

1. **GBP Controller** (`controller.py`): Main control logic implementing GBP-based optimization
2. **Trajectory Generator** (`trajectory.py`): Reference trajectory generation utilities
3. **Visualization Suite** (`plotting.py`): Comprehensive plotting and animation tools
4. **GBP Core** (`gbp/core.py`): Low-level Gaussian belief propagation implementation

### Control Methods

- **Standard GBP Control**: Traditional trajectory tracking with collision avoidance
- **Sensing-Enhanced GBP**: Adaptive control incorporating real-time sensor feedback

## Installation

### Prerequisites

- Python 3.8 or higher
- conda or pip package manager

### Setup

1. Clone the repository:
```bash
git clone https://github.com/VikramIyr/UniGBP.git
cd UniGBP
```

2. Install dependencies:
```bash
pip install -r requirements.txt
```

3. (Optional) For animation export, install ffmpeg:
```bash
# Using conda
conda install ffmpeg

# Using system package manager (Ubuntu/Debian)
sudo apt-get install ffmpeg

# Using Homebrew (macOS)
brew install ffmpeg
```

## Usage

### Basic Multi-Robot Simulation

Run the main simulation with default parameters:

```bash
python main.py
```

This will:
- Generate circular reference trajectories for two robots
- Apply GBP-based collision avoidance
- Create visualization plots and animations
- Save results to the `plots/` directory

### Configuration

#### Robot Dynamics
The system uses double-integrator dynamics:
```python
# State: [x_pos, y_pos, x_vel, y_vel]
# Input: [u_x, u_y]
A = [[0, 0, 1, 0],
     [0, 0, 0, 1], 
     [0, 0, 0, 0],
     [0, 0, 0, 0]]
     
B = [[0, 0],
     [0, 0],
     [1, 0],
     [0, 1]]
```

#### Key Parameters

**Control Parameters:**
- `Q_gbp`: State weighting matrix for trajectory tracking
- `R_gbp`: Control effort weighting matrix
- `S_gbp`: Terminal state weighting matrix

**GBP Parameters:**
```python
settings = GBPSettings(
    damping=0.1,            # Convergence damping factor
    beta=0.05,              # Relinearization threshold
    num_undamped_iters=3,   # Undamped iterations per solve
    min_linear_iters=5,     # Minimum iterations before relinearization
    dropout=0.0,            # Message dropout probability
)
```

**Safety Parameters:**
```python
robot_radius = 0.6        # Robot physical radius
safety_eps = 0.2          # Additional safety margin
sigma_collision = 1e-4    # Collision avoidance stiffness
```

#### Trajectory Types

**Circular Trajectories:**
```python
ref_traj = generate_circle_trajectory(
    radius=5.0,           # Circle radius
    omega=0.2,            # Angular velocity
    ts=time_array,        # Time points
    center=[0.0, 0.0],    # Circle center
    phase=0.0             # Initial phase offset
)
```

**Linear Trajectories:**
```python
ref_traj = generate_line_trajectory(
    start_pos=[5.0, 0.0], # Starting position
    end_pos=[-5.0, 0.0],  # Ending position  
    ts=time_array         # Time points
)
```

### Advanced Usage

#### Multi-Agent Configuration

```python
# Create controller
controller = GBPController(A=A, B=B, horizon=horizon, dt=dt,
                          sigma_collision=sigma_collision,
                          robot_radius=robot_radius,
                          safety_eps=safety_eps, 
                          settings=settings)

# Add multiple agents
for i, (ref_traj, x0) in enumerate(zip(ref_trajectories, initial_states)):
    controller.add_agent(Q=Q, R=R, S=S, ref_traj=ref_traj, x0=x0,
                        sigma_0=sigma_0, sigma_pos=sigma_pos,
                        sigma_vel=sigma_vel, sigma_u=sigma_u,
                        sigma_dynamics=sigma_dynamics)

# Enable collision avoidance
controller.add_inter_agent_collision()

# Solve optimization
x_list, u_list = controller.solve(n_iters=100, converged_threshold=1e-3)
```

#### Sensing-Based Control

```python
# Add agents with sensing capability
controller.add_agent_with_sensing(Q=Q, R=R, S=S, ref_traj=ref_traj, x0=x0, ts=ts,
                                 sigma_sensor=sigma_sensor,
                                 sigma_meas=sigma_meas, ...)

# Solve with real-time sensing
x_list, u_list = controller.solve_with_sensing(sigma_meas=sigma_meas)
```

### Visualization

The system provides comprehensive visualization tools:

#### Static Plots
```python
from plotting import plot_trajectories

plot_trajectories(ref_trajs=[ref1, ref2], 
                 actual_trajs=[actual1, actual2],
                 robot_radius=robot_radius,
                 title="Multi-Robot Trajectory Comparison")
```

#### Animations
```python
from plotting import animate_trajectories

anim = animate_trajectories(ref_trajs=[ref1, ref2],
                           actual_trajs=[actual1, actual2], 
                           ts=time_array, dt=dt,
                           robot_radius=robot_radius)
```

#### Analysis Tools
```python
from plotting import analyze_collisions, analyze_tracking

# Collision analysis
analyze_collisions(trajectories=[traj1, traj2], ts=time_array,
                  robot_radius=robot_radius, safety_eps=safety_eps)

# Tracking performance analysis  
analyze_tracking(ts=time_array, trajectories=[traj1, traj2],
                ref_trajectories=[ref1, ref2], controls=[u1, u2], dt=dt)
```

## File Structure

```
UniGBP/
├── main.py              # Main simulation script
├── controller.py        # GBP controller implementation
├── trajectory.py        # Trajectory generation utilities
├── plotting.py         # Visualization and analysis tools
├── requirements.txt    # Package dependencies
├── README.md          # This file
├── LICENSE           # License information
└── gbp/
    └── core.py       # Low-level GBP implementation
```

## Citation

If you use this code in your research, please cite:

```bibtex
@misc{unigbp2024,
  title={UniGBP: Multi-Robot Gaussian Belief Propagation Controller},
  author={[Kerem Kilic, Oguz Gursoy, Vikram Iyer]},
  year={2025},
  url={https://github.com/VikramIyr/UniGBP}
}
```

## License

This project is licensed under the MIT License - see the LICENSE file for details.

## Contributing

1. Fork the repository
2. Create a feature branch (`git checkout -b feature/AmazingFeature`)
3. Commit your changes (`git commit -m 'Add some AmazingFeature'`)
4. Push to the branch (`git push origin feature/AmazingFeature`)
5. Open a Pull Request
