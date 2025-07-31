import numpy as np

def generate_circle_trajectory(radius, 
                               omega, 
                               ts,
                               center=np.zeros(2),
                               phase=0.0):
    """
    Circular trajectory (pos+vel) around an arbitrary center and phase.

    Args:
        radius: circle radius
        omega:  angular speed
        ts:     time array, shape (N,)
        center: 2-vector (cx,cy)
        phase:  initial phase offset φ

    Returns:
        traj: array (N,4) with [x,y, ẋ,ẏ] at each t
    """
    # positions
    angles = omega*ts + phase
    x = center[0] + radius * np.cos(angles)
    y = center[1] + radius * np.sin(angles)

    # velocities = derivative wrt t
    dx = -radius * omega * np.sin(angles)
    dy =  radius * omega * np.cos(angles)

    traj = np.stack([x, y, dx, dy], axis=1)
    return traj

def generate_line_trajectory(start_pos, end_pos, ts):
    """
    Linear trajectory from start_pos to end_pos over time array ts.
    Returns (N, 4) array: [x, y, vx, vy] at each time step.
    """
    duration = ts[-1] - ts[0]
    v = (end_pos - start_pos) / duration
    x = np.outer(ts, v) + start_pos
    vx = np.tile(v, (len(ts), 1))
    return np.hstack((x, vx))