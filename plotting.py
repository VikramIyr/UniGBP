#!/usr/bin/env python3
"""
Generalized plotting utilities for multi-robot trajectory visualization and analysis.

This module provides comprehensive plotting functions for:
- Static trajectory comparison plots
- Animated trajectory visualization
- Inter-robot collision analysis
- Tracking error and control input analysis

Author: Multi-Robot GBP Controller Project
"""

import os
import numpy as np
import matplotlib.pyplot as plt
from matplotlib import animation
from typing import List, Optional, Tuple, Dict, Union


class TrajectoryPlotter:
    """
    A comprehensive plotting class for multi-robot trajectory visualization and analysis.
    """
    
    def __init__(self, 
                 save_plots: bool = True,
                 save_dir: str = 'plots/',
                 dpi: int = 300,
                 figsize: Tuple[int, int] = (10, 8)):
        """
        Initialize the TrajectoryPlotter.
        
        Parameters
        ----------
        save_plots : bool
            Whether to save plots to files
        save_dir : str
            Directory to save plots
        dpi : int
            DPI for high resolution plots
        figsize : tuple
            Default figure size for plots
        """
        self.save_plots = save_plots
        self.save_dir = save_dir
        self.dpi = dpi
        self.figsize = figsize
        
        # Create save directory if it doesn't exist
        if self.save_plots and not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
    
    def plot_static_trajectories(self,
                                ref_trajs: List[np.ndarray],
                                actual_trajs: List[np.ndarray],
                                ref_labels: Optional[List[str]] = None,
                                actual_labels: Optional[List[str]] = None,
                                colors_ref: Optional[List[str]] = None,
                                colors_actual: Optional[List[str]] = None,
                                robot_radius: float = 1.0,
                                x_label: str = 'X Position [m]',
                                y_label: str = 'Y Position [m]',
                                title: str = 'Multi-Robot Trajectory Comparison',
                                filename: str = 'trajectory_static') -> None:
        """
        Create a static plot comparing reference and actual trajectories.
        
        Parameters
        ----------
        ref_trajs : list of np.ndarray
            List of reference trajectories, each of shape (N, >=2)
        actual_trajs : list of np.ndarray
            List of actual trajectories, each of shape (N, >=2)
        ref_labels : list of str, optional
            Labels for reference trajectories
        actual_labels : list of str, optional
            Labels for actual trajectories
        colors_ref : list of str, optional
            Colors/styles for reference trajectories
        colors_actual : list of str, optional
            Colors/styles for actual trajectories
        robot_radius : float
            Robot radius for visualization circles
        x_label, y_label : str
            Axis labels
        title : str
            Plot title
        filename : str
            Filename for saving (without extension)
        """
        # Set defaults
        n_ref = len(ref_trajs)
        n_act = len(actual_trajs)
        ref_labels = ref_labels or [f'Ref {i+1}' for i in range(n_ref)]
        actual_labels = actual_labels or [f'Actual {i+1}' for i in range(n_act)]
        colors_ref = colors_ref or ['--b', '--g', '--c', '--y', '--k', '--orange'][:n_ref]
        colors_actual = colors_actual or ['-r', '-m', '-g', '-b', '-c', '-y'][:n_act]
        
        # Compute global axis limits
        all_x = np.hstack([traj[:, 0] for traj in ref_trajs + actual_trajs])
        all_y = np.hstack([traj[:, 1] for traj in ref_trajs + actual_trajs])
        limit = max(all_x.max(), -all_x.min(), all_y.max(), -all_y.min()) * 1.3
        
        # Create static plot
        plt.figure(figsize=self.figsize, dpi=self.dpi)
        
        # Plot reference trajectories
        for traj, label, style in zip(ref_trajs, ref_labels, colors_ref):
            plt.plot(traj[:, 0], traj[:, 1], style, label=label, linewidth=2)
        
        # Plot actual trajectories with robot circles
        for traj, label, style in zip(actual_trajs, actual_labels, colors_actual):
            plt.plot(traj[:, 0], traj[:, 1], style, label=label, linewidth=2)
            
            # Add robot circles at start and end positions
            color = style[1:] if style.startswith('-') else style
            start_circle = plt.Circle((traj[0, 0], traj[0, 1]), robot_radius,
                                    fill=False, edgecolor=color, linestyle='--', alpha=0.5)
            end_circle = plt.Circle((traj[-1, 0], traj[-1, 1]), robot_radius,
                                  fill=False, edgecolor=color, linestyle='--', alpha=0.5)
            plt.gca().add_patch(start_circle)
            plt.gca().add_patch(end_circle)
        
        plt.xlabel(x_label)
        plt.ylabel(y_label)
        plt.title(title)
        plt.axis('equal')
        plt.grid(True, alpha=0.3)
        plt.legend()
        plt.tight_layout()
        
        # Save plot
        if self.save_plots:
            png_file = os.path.join(self.save_dir, f'{filename}.png')
            plt.savefig(png_file, dpi=self.dpi, bbox_inches='tight', facecolor='white')
            print(f"Static plot saved: {png_file}")
        
        plt.show()
    
    def animate_trajectories(self,
                           ref_trajs: List[np.ndarray],
                           actual_trajs: List[np.ndarray],
                           ts: np.ndarray,
                           dt: float,
                           ref_labels: Optional[List[str]] = None,
                           actual_labels: Optional[List[str]] = None,
                           colors_ref: Optional[List[str]] = None,
                           colors_actual: Optional[List[str]] = None,
                           robot_radius: float = 1.0,
                           x_label: str = 'X Position [m]',
                           y_label: str = 'Y Position [m]',
                           title: str = 'Multi-Robot Animation',
                           filename: str = 'trajectory_animation') -> animation.FuncAnimation:
        """
        Create an animated visualization of trajectories.
        
        Parameters
        ----------
        ref_trajs : list of np.ndarray
            List of reference trajectories
        actual_trajs : list of np.ndarray
            List of actual trajectories
        ts : np.ndarray
            Time array
        dt : float
            Time step for animation speed
        Other parameters : same as plot_static_trajectories
        filename : str
            Filename for saving animation
            
        Returns
        -------
        animation.FuncAnimation
            The animation object
        """
        # Set defaults
        n_ref = len(ref_trajs)
        n_act = len(actual_trajs)
        ref_labels = ref_labels or [f'Ref {i+1}' for i in range(n_ref)]
        actual_labels = actual_labels or [f'Actual {i+1}' for i in range(n_act)]
        colors_ref = colors_ref or ['--b', '--g', '--c', '--y', '--k', '--orange'][:n_ref]
        colors_actual = colors_actual or ['-r', '-m', '-g', '-b', '-c', '-y'][:n_act]
        
        # Compute global axis limits
        all_x = np.hstack([traj[:, 0] for traj in ref_trajs + actual_trajs])
        all_y = np.hstack([traj[:, 1] for traj in ref_trajs + actual_trajs])
        limit = max(all_x.max(), -all_x.min(), all_y.max(), -all_y.min()) * 1.3
        
        # Setup animation
        fig, ax = plt.subplots(figsize=(8, 8), dpi=self.dpi)
        ax.set_xlim(-limit, limit)
        ax.set_ylim(-limit, limit)
        ax.set_xlabel(x_label)
        ax.set_ylabel(y_label)
        ax.set_title(title)
        ax.set_aspect('equal')
        ax.grid(True, alpha=0.3)
        
        # Static reference lines
        ref_lines = []
        for traj, label, style in zip(ref_trajs, ref_labels, colors_ref):
            line, = ax.plot(traj[:, 0], traj[:, 1], style, label=label, alpha=0.7)
            ref_lines.append(line)
        
        # Dynamic actual lines and robot circles
        act_lines = []
        act_circles = []
        for label, style in zip(actual_labels, colors_actual):
            line, = ax.plot([], [], style, label=label, linewidth=2)
            color = style[1:] if style.startswith('-') else style
            circle = plt.Circle((0, 0), robot_radius, fill=False,
                              edgecolor=color, linewidth=2, alpha=0.8)
            ax.add_patch(circle)
            act_lines.append(line)
            act_circles.append(circle)
        
        def init():
            for line in act_lines:
                line.set_data([], [])
            for circle in act_circles:
                circle.center = (0, 0)
                circle.set_visible(False)
            return (*ref_lines, *act_lines, *act_circles)
        
        def animate_frame(i):
            for traj, line, circle in zip(actual_trajs, act_lines, act_circles):
                # Update trajectory line
                line.set_data(traj[:i+1, 0], traj[:i+1, 1])
                # Update robot circle position
                circle.center = (traj[i, 0], traj[i, 1])
                circle.set_visible(True)
            return (*ref_lines, *act_lines, *act_circles)
        
        # Create animation
        anim = animation.FuncAnimation(
            fig, animate_frame, init_func=init,
            frames=len(ts), interval=dt*1000, blit=True, repeat=True
        )
        
        plt.legend()
        
        # Save animation
        if self.save_plots:
            # Save as MP4 (requires ffmpeg)
            try:
                mp4_file = os.path.join(self.save_dir, f'{filename}.mp4')
                anim.save(mp4_file, writer='ffmpeg', fps=30, dpi=self.dpi//2)
                print(f"Animation saved as MP4: {mp4_file}")
            except Exception as e:
                print(f"Could not save MP4 (ffmpeg required): {e}")
        
        plt.show()
        return anim
    
    def plot_collision_analysis(self,
                              trajectories: List[np.ndarray],
                              ts: np.ndarray,
                              robot_radius: float,
                              safety_eps: float,
                              robot_labels: Optional[List[str]] = None,
                              colors: Optional[List[str]] = None,
                              filename: str = 'collision_analysis') -> None:
        """
        Plot inter-robot distance analysis and collision constraints.
        
        Parameters
        ----------
        trajectories : list of np.ndarray
            List of robot trajectories, each of shape (N, >=2)
        ts : np.ndarray
            Time array
        robot_radius : float
            Robot radius
        safety_eps : float
            Safety margin
        robot_labels : list of str, optional
            Labels for robots
        colors : list of str, optional
            Colors for robots
        filename : str
            Filename for saving
        """
        n_robots = len(trajectories)
        robot_labels = robot_labels or [f'Robot {i+1}' for i in range(n_robots)]
        colors = colors or ['r', 'm', 'g', 'b', 'c', 'y'][:n_robots]
        
        # Calculate pairwise distances
        distances = {}
        min_distances = {}
        
        for i in range(n_robots):
            for j in range(i + 1, n_robots):
                pair_name = f'{robot_labels[i]} - {robot_labels[j]}'
                dists = np.linalg.norm(trajectories[i][:, :2] - trajectories[j][:, :2], axis=1)
                distances[pair_name] = dists
                min_distances[pair_name] = dists.min()
        
        # Safety thresholds
        collision_threshold = 2 * robot_radius
        safety_threshold = 2 * robot_radius + safety_eps
        
        # Create subplots
        fig, axes = plt.subplots(1, 2, figsize=(15, 6), dpi=self.dpi)
        
        # Left plot: Distance over time
        ax1 = axes[0]
        for pair_name, dists in distances.items():
            ax1.plot(ts, dists, '-', linewidth=2, label=f'{pair_name} distance')
        
        ax1.axhline(y=collision_threshold, color='r', linestyle='--',
                   label=f'Collision threshold ({collision_threshold:.2f}m)')
        ax1.axhline(y=safety_threshold, color='orange', linestyle='--',
                   label=f'Safety threshold ({safety_threshold:.2f}m)')
        
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Distance [m]')
        ax1.set_title('Inter-Robot Distance Analysis')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Right plot: Trajectory with collision zones at closest approach
        ax2 = axes[1]
        
        # Plot all trajectories
        for i, (traj, label, color) in enumerate(zip(trajectories, robot_labels, colors)):
            ax2.plot(traj[:, 0], traj[:, 1], '-', color=color, label=label, linewidth=2)
        
        # Find and visualize closest approach for the most critical pair
        closest_pair = min(min_distances.keys(), key=lambda k: min_distances[k])
        closest_distance = min_distances[closest_pair]
        
        # Extract robot indices from pair name
        robot_names = closest_pair.split(' - ')
        idx1 = robot_labels.index(robot_names[0])
        idx2 = robot_labels.index(robot_names[1])
        
        # Find time of closest approach
        pair_dists = distances[closest_pair]
        min_dist_idx = np.argmin(pair_dists)
        pos1 = trajectories[idx1][min_dist_idx, :2]
        pos2 = trajectories[idx2][min_dist_idx, :2]
        
        # Draw robots at closest approach
        circle1 = plt.Circle(pos1, robot_radius, fill=True, alpha=0.3, color=colors[idx1])
        circle2 = plt.Circle(pos2, robot_radius, fill=True, alpha=0.3, color=colors[idx2])
        ax2.add_patch(circle1)
        ax2.add_patch(circle2)
        
        # Draw safety zones
        safety_circle1 = plt.Circle(pos1, safety_threshold/2, fill=False,
                                   edgecolor='orange', linestyle='--', alpha=0.7)
        safety_circle2 = plt.Circle(pos2, safety_threshold/2, fill=False,
                                   edgecolor='orange', linestyle='--', alpha=0.7)
        ax2.add_patch(safety_circle1)
        ax2.add_patch(safety_circle2)
        
        ax2.set_xlabel('X Position [m]')
        ax2.set_ylabel('Y Position [m]')
        ax2.set_title(f'Closest Approach: {closest_pair}\n(d={closest_distance:.3f}m at t={ts[min_dist_idx]:.2f}s)')
        ax2.axis('equal')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        if self.save_plots:
            png_file = os.path.join(self.save_dir, f'{filename}.png')
            plt.savefig(png_file, dpi=self.dpi, bbox_inches='tight', facecolor='white')
            print(f"Collision analysis saved: {png_file}")
        
        plt.show()
        
        # Print summary
        print("\n" + "="*60)
        print("COLLISION ANALYSIS SUMMARY")
        print("="*60)
        for pair_name, min_dist in min_distances.items():
            collision_avoided = "✓" if min_dist > collision_threshold else "✗"
            safety_maintained = "✓" if min_dist > safety_threshold else "✗"
            print(f"{pair_name}:")
            print(f"  Minimum separation: {min_dist:.4f} m")
            print(f"  Collision avoided: {collision_avoided}")
            print(f"  Safety maintained: {safety_maintained}")
        print(f"\nSafety threshold: {safety_threshold:.4f} m")
        print(f"Collision threshold: {collision_threshold:.4f} m")
        print("="*60)
    
    def plot_tracking_and_control(self,
                                ts: np.ndarray,
                                trajectories: List[np.ndarray],
                                ref_trajectories: List[np.ndarray],
                                controls: List[np.ndarray],
                                dt: float,
                                robot_labels: Optional[List[str]] = None,
                                control_labels: Tuple[str, str] = ("u_x", "u_y"),
                                colors: Optional[List[str]] = None,
                                filename: str = 'tracking_and_control') -> None:
        """
        Plot tracking errors and control inputs for multiple robots.
        
        Parameters
        ----------
        ts : np.ndarray
            Time vector
        trajectories : list of np.ndarray
            Actual trajectories, each of shape (N, >=2)
        ref_trajectories : list of np.ndarray
            Reference trajectories, each of shape (N, >=2)
        controls : list of np.ndarray
            Control inputs, each of shape (N-1, 2)
        dt : float
            Time step
        robot_labels : list of str, optional
            Labels for robots
        control_labels : tuple of str
            Labels for control inputs
        colors : list of str, optional
            Colors for robots
        filename : str
            Filename for saving
        """
        n_robots = len(trajectories)
        robot_labels = robot_labels or [f'Robot {i+1}' for i in range(n_robots)]
        colors = colors or ['r', 'm', 'g', 'b', 'c', 'y'][:n_robots]
        
        # Compute tracking errors
        errors = []
        for traj, ref_traj in zip(trajectories, ref_trajectories):
            error = np.linalg.norm(traj[:, :2] - ref_traj[:, :2], axis=1)
            errors.append(error)
        
        # Create subplots
        fig, axes = plt.subplots(2, 2, figsize=(15, 10), dpi=self.dpi)
        
        # Position tracking errors
        ax1 = axes[0, 0]
        for error, label, color in zip(errors, robot_labels, colors):
            ax1.plot(ts, error, color=color, label=label, linewidth=2)
        ax1.set_xlabel('Time [s]')
        ax1.set_ylabel('Position Error [m]')
        ax1.set_title('Position Tracking Errors')
        ax1.legend()
        ax1.grid(True, alpha=0.3)
        
        # Cumulative tracking errors
        ax2 = axes[0, 1]
        for error, label, color in zip(errors, robot_labels, colors):
            cumulative_error = np.cumsum(error) * dt
            ax2.plot(ts, cumulative_error, color=color, label=label, linewidth=2)
        ax2.set_xlabel('Time [s]')
        ax2.set_ylabel('Cumulative Error [m·s]')
        ax2.set_title('Cumulative Tracking Errors')
        ax2.legend()
        ax2.grid(True, alpha=0.3)
        
        # Control inputs components (for first robot as example)
        ax3 = axes[1, 0]
        if len(controls) > 0:
            control = controls[0]
            ax3.plot(ts[:-1], control[:, 0], 'r-', label=control_labels[0], linewidth=2)
            ax3.plot(ts[:-1], control[:, 1], 'b-', label=control_labels[1], linewidth=2)
            ax3.set_xlabel('Time [s]')
            ax3.set_ylabel('Control Input [m/s²]')
            ax3.set_title(f'{robot_labels[0]} Control Components')
            ax3.legend()
            ax3.grid(True, alpha=0.3)

        # Control inputs components (for second robot as example)
        ax4 = axes[1, 1]
        if len(controls) > 0:
            control = controls[1]
            ax4.plot(ts[:-1], control[:, 0], 'r-', label=control_labels[0], linewidth=2)
            ax4.plot(ts[:-1], control[:, 1], 'b-', label=control_labels[1], linewidth=2)
            ax4.set_xlabel('Time [s]')
            ax4.set_ylabel('Control Input [m/s²]')
            ax4.set_title(f'{robot_labels[1]} Control Components')
            ax4.legend()
            ax4.grid(True, alpha=0.3)
        
        plt.tight_layout()
        
        # Save plot
        if self.save_plots:
            png_file = os.path.join(self.save_dir, f'{filename}.png')
            plt.savefig(png_file, dpi=self.dpi, bbox_inches='tight', facecolor='white')
            print(f"Tracking analysis saved: {png_file}")
        
        plt.show()


# Convenience functions for backward compatibility and ease of use
def plot_trajectories(ref_trajs: List[np.ndarray],
                     actual_trajs: List[np.ndarray],
                     robot_radius: float = 1.0,
                     save_plots: bool = True,
                     save_dir: str = 'plots/',
                     **kwargs) -> None:
    """Convenience function for static trajectory plotting."""
    plotter = TrajectoryPlotter(save_plots=save_plots, save_dir=save_dir)
    plotter.plot_static_trajectories(ref_trajs, actual_trajs, robot_radius=robot_radius, **kwargs)


def animate_trajectories(ref_trajs: List[np.ndarray],
                        actual_trajs: List[np.ndarray],
                        ts: np.ndarray,
                        dt: float,
                        robot_radius: float = 1.0,
                        save_plots: bool = True,
                        save_dir: str = 'plots/',
                        **kwargs) -> animation.FuncAnimation:
    """Convenience function for trajectory animation."""
    plotter = TrajectoryPlotter(save_plots=save_plots, save_dir=save_dir)
    return plotter.animate_trajectories(ref_trajs, actual_trajs, ts, dt, robot_radius=robot_radius, **kwargs)


def analyze_collisions(trajectories: List[np.ndarray],
                      ts: np.ndarray,
                      robot_radius: float,
                      safety_eps: float,
                      save_plots: bool = True,
                      save_dir: str = 'plots/',
                      **kwargs) -> None:
    """Convenience function for collision analysis."""
    plotter = TrajectoryPlotter(save_plots=save_plots, save_dir=save_dir)
    plotter.plot_collision_analysis(trajectories, ts, robot_radius, safety_eps, **kwargs)


def analyze_tracking(ts: np.ndarray,
                    trajectories: List[np.ndarray],
                    ref_trajectories: List[np.ndarray],
                    controls: List[np.ndarray],
                    dt: float,
                    save_plots: bool = True,
                    save_dir: str = 'plots/',
                    **kwargs) -> None:
    """Convenience function for tracking and control analysis."""
    plotter = TrajectoryPlotter(save_plots=save_plots, save_dir=save_dir)
    plotter.plot_tracking_and_control(ts, trajectories, ref_trajectories, controls, dt, **kwargs)
