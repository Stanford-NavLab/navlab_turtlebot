"""Utils

General utilities.

"""

import numpy as np


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


def wrap_angle(a):
    """Wrap angle to [-pi, pi]

    Parameters
    ----------
    a : float
        Angle in radians
    
    Returns
    -------
    float
        Wrapped angle in radians

    """
    return (a + np.pi) % (2 * np.pi) - np.pi