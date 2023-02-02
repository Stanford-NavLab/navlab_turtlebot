"""Utils

General utilities for planning code.

"""

import rospy
import numpy as np
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint

from common.trajectory import Trajectory


def wrap_2D_traj_msg(traj, t2start):
    """Wraps a 2-D trajectory in a JointTrajectory message.

    Parameters
    ----------
    traj : tuple (p,v,a) of np.array (2 x N)
        Trajectory containing position, velocity, and acceleration
    t2start : float
        Time to start in seconds

    Returns
    -------
    JointTrajectory 
        Wrapped message.

    """
    p,v,a = traj
    traj_msg = JointTrajectory()

    jtp_x = JointTrajectoryPoint()
    jtp_x.positions = p[:,0]
    jtp_x.velocities = v[:,0]
    jtp_x.accelerations = a[:,0]
    jtp_x.time_from_start = rospy.Duration(t2start)

    jtp_y = JointTrajectoryPoint()
    jtp_y.positions = p[:,1]
    jtp_y.velocities = v[:,1]
    jtp_y.accelerations = a[:,1]
    jtp_y.time_from_start = rospy.Duration(t2start)

    traj_msg.points = [jtp_x, jtp_y]
    traj_msg.joint_names = ['x','y']

    return traj_msg


def unwrap_2D_traj_msg(msg, time):
    """Convert JointTrajectory message to Trajectory class

    Parameters
    ----------
    msg : JointTrajectory 
        JointTrajectory message
    time : np.array (1 x N)
        Time vector
    Returns

    -------
    Trajectory
        Trajectory wrapped in class

    """
    px = np.array(msg.points[0].positions)
    py = np.array(msg.points[1].positions)
    vx = np.array(msg.points[0].velocities)
    vy = np.array(msg.points[1].velocities)
    ax = np.array(msg.points[0].accelerations)
    ay = np.array(msg.points[1].accelerations)

    pos = np.hstack((px[:,None], py[:,None]))
    vel = np.hstack((vx[:,None], vy[:,None]))
    acc = np.hstack((ax[:,None], ay[:,None]))

    values = np.stack((pos, vel, acc))

    return Trajectory(time, values=values)


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


