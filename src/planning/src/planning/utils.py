"""Utils

General utilities for planning code.

"""

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint


def check_obs_collision(positions, obs, r_collision):
    """Check a sequence of positions against a single obstacle for collision.

    Obstacles are cylinders represented as (center, radius)

    Parameters
    ----------
    positions : np.array
    obs : tuple

    Returns
    -------
    bool
        True if the plan is safe, False is there is a collision

    """
    c_obs, r_obs = obs
    d_vec = np.linalg.norm(positions - c_obs, axis=1)
    if any(d_vec <= r_collision + r_obs):
        return False
    else:
        return True


def prune_vel_samples(V, v_0, max_norm, max_delta):
    """Prune Velocity Samples
    
    """
    V_mag = np.linalg.norm(V, axis=1)
    delta_V = np.linalg.norm(V - v_0, axis=1)
    keep_idx = np.logical_and(V_mag < max_norm, delta_V < max_delta)
    return V[keep_idx]


