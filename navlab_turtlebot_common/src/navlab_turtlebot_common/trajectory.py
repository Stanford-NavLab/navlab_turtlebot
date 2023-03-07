"""Trajectory class

"""

import numpy as np

from navlab_turtlebot_common.utils import wrap_angle
import navlab_turtlebot_common.params as params


class Trajectory:
    """Trajectory class

    Planned trajectory

    Attributes
    ----------
    length : int
        Length of trajectory (i.e. number of timesteps), abbreviated as N below
    n_dim : int
        State dimension of the trajectory (i.e. 2D or 3D)
    time : np.array (N)
        Time array
    positions : np.ndarray (N x N_dim)
        Positions
    velocities : np.ndarray (N x N_dim)
        Velocities
    accelerations : np.ndarray (N x N_dim)
        Accelerations

    """
    def __init__(self, time, n_dim=None, values=None):
        """Initialize trajectory

        Parameters
        ----------
        time : np.array (N)
            Time array
        n_dim : int
            State dimension of the trajectory (i.e. 2D or 3D)
        values : np.ndarray (3 x N x n_dim)
            Values array containing positions, velocities, and accelerations

        """
        self.time = time
        self.length = len(time)
        if values is None:
            if n_dim is None:
                raise ValueError("Must specify N_dim or values")
            else:
                self.n_dim = n_dim
                self.positions = np.zeros((self.length, self.n_dim))
                self.velocities = np.zeros((self.length, self.n_dim))
                self.accelerations = np.zeros((self.length, self.n_dim))
        else:
            self.n_dim = values.shape[2]
            self.positions = values[0]
            self.velocities = values[1]
            self.accelerations = values[2]

        self.thetas = None


    def compute_thetas(self):
        """Compute thetas from trajectory

        """
        self.thetas = np.arctan2(self.velocities[:,1], self.velocities[:,0])


    def compute_twist_controls(self):
        """Compute twists from trajectory

        Returns
        -------
        np.ndarray (N x 6)
            Twist array

        """
        # Array of (v, w) pairs where v is linear velocity and w is angular velocity
        twists = np.zeros((self.length, 2))
        twists[:,0] = np.linalg.norm(self.velocities, axis=1)

        # Compute angular velocities
        if self.thetas is not None:
            thetas = self.thetas
        else:
            thetas = np.arctan2(self.velocities[:,1], self.velocities[:,0])
        dthetas = wrap_angle(np.diff(thetas))  # TODO: final thetas sometimes doesn't seem to get wrapped properly
        twists[:-1,1] = dthetas / params.DT
        # Final angular twist is left as 0

        # FIXME: this is a hack to set first angular twist to 0
        twists[0,1] = 0

        return twists
