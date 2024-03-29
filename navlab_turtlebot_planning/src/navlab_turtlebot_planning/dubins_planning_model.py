"""Dubins Planning Model

"""

import numpy as np
from sympy import symbols, lambdify, Array, sin, cos, diff

import navlab_turtlebot_common.params as params

# Generate symbolic dynamics
x, y, th, v, w = symbols('x y th v w')

x0 = Array([x, y, th])
expr = x0
for i in range(params.TRAJ_IDX_LEN):
    expr = expr + params.DT * Array([v*cos(expr[2]), v*sin(expr[2]), w])

dubins = lambdify([x, y, th, v, w], expr)



def dubins_step(x, u, dt):
    """Run one step of dynamics

    Parameters
    ----------
    x : np.array
        State vector (x, y, theta)
    u : np.array
        Control vector (v, w)
    dt : float
        Time step

    Returns
    -------
    np.array
        Updated state vector (x, y, theta)

    """
    x_dot = u[0] * np.cos(x[2])
    y_dot = u[0] * np.sin(x[2])
    theta_dot = u[1]
    x_new = x + np.array([x_dot, y_dot, theta_dot]) * dt
    return x_new


def dubins_traj(x0, u, N, dt):
    """Compute dubins trajectory for given initial pose and control input
    
    Parameters
    ----------
    x0 : np.array (3)
        Initial state vector (x, y, theta)
    u : np.array (2)
        Control input (v, w)
    N : int
        Number of steps
    dt : float
        Time step
    
    Returns
    -------
    np.array (N, 3)
        Trajectory (x, y, theta)
    
    """
    traj = np.zeros((N, 3))
    traj[0] = x0
    for i in range(1, N):
        traj[i] = dubins_step(traj[i-1], u, dt)
    return traj