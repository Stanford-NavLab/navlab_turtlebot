"""Utils

General utilities.

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

    print(px)
    print(px.shape)

    pos = np.hstack((px[:,None], py[:,None]))
    vel = np.hstack((vx[:,None], vy[:,None]))
    acc = np.hstack((ax[:,None], ay[:,None]))

    values = np.stack((pos, vel, acc))

    return Trajectory(time, values=values)


def rand_in_bounds(bounds, n):
    """Generate random samples within specified bounds

    Parameters
    ----------
    bounds : list
        List of min and max values for each dimension.
    n : int
        Number of points to generate.

    Returns
    -------
    np.array 
        Random samples

    """
    x_pts = np.random.uniform(bounds[0], bounds[1], n)
    y_pts = np.random.uniform(bounds[2], bounds[3], n)
    # 2D 
    if len(bounds) == 4:
        return np.hstack((x_pts[:,None], y_pts[:,None]))
    # 3D
    elif len(bounds) == 6:
        z_pts = np.random.uniform(bounds[4], bounds[5], n)
        return np.hstack((x_pts[:,None], y_pts[:,None], z_pts[:,None]))
    else:
        raise ValueError('Please pass in bounds as either [xmin xmax ymin ymax] '
                            'or [xmin xmax ymin ymax zmin zmax] ')


def normalize(v):
    """Normalize a vector

    Parameters
    ----------
    v : np.array
        Vector to normalize

    Returns
    -------
    np.array
        Normalized vector

    """
    if np.linalg.norm(v) == 0:
        return v
    return v / np.linalg.norm(v)



def signed_angle_btwn_vectors(v1, v2):
    """Signed angle between two 2D vectors

    Counter-clockwise is positive, clockwise is negative.

    Parameters
    ----------
    v1 : np.array
        Vector 1
    v2 : np.array
        Vector 2

    Returns
    -------
    float
        Angle between vectors in radians

    """
    v1_ = normalize(v1.flatten())
    v2_ = normalize(v2.flatten())
    return np.sign(np.cross(v1_, v2_)) * np.arccos(np.dot(v1_, v2_))