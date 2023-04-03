# Utilities for controller nodes

import rospy
import numpy as np

from navlab_turtlebot_common.utils import wrap_angle
import navlab_turtlebot_common.params as params


def compute_control(x_nom, u_nom, x_hat, K):
    """Compute total control input vector
    Parameters
    ----------
    x_nom : np.array (4x1)
        nominal state
    u_nom : np.array (4x1)
        nominal control input
    x_hat : np.array (4x1)
        estimated state
    K : np.array (2x4)
        control feedback gain matrix
    Returns
    -------
    u : np.array (2x1)
        total control input
    """
    # Get error between estimated and nominal states
    err = x_hat - x_nom
    err[2] = wrap_angle(err[2])  # Wrap theta

    # Compute total control input
    u = u_nom - K @ err

    return u


def EKF_prediction_step(x_hat, u, P, A, Q, dt):
    """Perform EKF prediction step

    Parameters
    ----------
    x_hat : np.array (4x1)
        estimated state
    u : np.array (2x1)
        total control input
    P : np.array (4x4)
        state estimation covariance matrix
    A : np.array (4x4)
        linearized motion model matrix
    Q : np.array (4x4)
        motion model covariance
    dt : float
        discrete time-step
    Returns
    -------
    x_pred : np.array (4x1)
        predicted state
    P_pred : np.array (4x4)
        predicted state estimation covariance matrix
    """
    # Compute predicted state
    x_pred = x_hat + np.array([u[0] * np.cos(x_hat[2]), u[0] * np.sin(x_hat[2]), u[1]]) * dt
    # Compute predicted state estimation covariance matrix
    P_pred = A @ P @ A.T + Q

    return x_pred, P_pred


def EKF_correction_step(x_pred, P_pred, z, C, R):
    """
    Perform EKF correction step

    Parameters
    ----------
    x_pred : np.array (4x1)
        predicted state
    P_pred : np.array (4x4)
        predicted state estimation covariance matrix
    z : np.array (4x1)
        received measurement
    C : np.array (3x4)
        measurement matrix
    R : np.array (3x3)
        sensing model covariance
    Returns
    -------
    x_hat : np.array (4x1)
        corrected state estimate
    P : np.array (4x4)
        corrected state estimation covariance matrix
    L : np.array (4x3)
        Kalman gain matrix
    """
    # Compute Kalman gain
    L = P_pred @ C.T @ np.linalg.inv(C @ P_pred @ C.T + R)
    # Compute corrected state estimate
    x_hat = x_pred + L @ (z - C @ x_pred)
    # Compute corrected state estimation covariance matrix
    P = P_pred - L @ C @ P_pred

    return x_hat, P


def dlqr_calculate(G, H, Q, R):
    """
    Discrete-time Linear Quadratic Regulator calculation.
    State-feedback control  u[k] = -K*x[k]
    Implementation from  https://github.com/python-control/python-control/issues/359#issuecomment-759423706
    How to apply the function:    
        K = dlqr_calculate(G,H,Q,R)
        K, P, E = dlqr_calculate(G,H,Q,R, return_solution_eigs=True)
    Inputs:
      G, H, Q, R  -> all numpy arrays  (simple float number not allowed)
      returnPE: define as True to return Ricatti solution and final eigenvalues
    Returns:
      K: state feedback gain
      P: Ricatti equation solution
      E: eigenvalues of (G-HK)  (closed loop z-domain poles)
      
    """
    from scipy.linalg import solve_discrete_are, inv
    P = solve_discrete_are(G, H, Q, R)  #Solução Ricatti
    K = inv(H.T@P@H + R)@H.T@P@G    #K = (B^T P B + R)^-1 B^T P A 

    return K


def generate_robot_matrices(x, u):
    """Generate A, B, C and K matrices for robot based on given nominal state and input vector.

    TODO: replace with linearized dynamics from multirobot-planning

    Parameters
    ----------
    x_nom : np.array (4x1)
        nominal state
    u_nom : np.array (2x1)
        nominal input

    Returns
    -------
    A : np.array (4x4)
        Linearized motion model matrix.
    B : np.array (4x2)
        Linearized control input matrix.
    C : np.array (3x4)
        Measurement matrix.
    K : np.array (2x4)
        Control feedback gain matrix.

    """
    A = np.array([[1, 0, -u[0] * np.sin(x[2]) * params.DT],
                  [0, 1,  u[0] * np.cos(x[2]) * params.DT],
                  [0, 0, 1]])
    B = np.array([[np.cos(x[2]) * params.DT, -0.5 * u[0] * params.DT**2 * np.sin(x[2])],
                  [np.sin(x[2]) * params.DT,  0.5 * u[0] * params.DT**2 * np.cos(x[2])],
                  [0, params.DT]])

    # Form measurement matrix
    C = np.eye(3)  # mocap gives (x, y, theta)

    # Compute control feedback gain matrix
    K = dlqr_calculate(A, B, params.Q_LQR, params.R_LQR)
    # if np.abs(x_nom[3,0]) > 0.01:  # if there is sufficient speed for controllability
    #     K = dlqr_calculate(A, B, params.Q_LQR, params.R_LQR)
    # else:
    #     K = np.array([[0, 0, 1.0, 0], 
    #                   [0, 0, 0, 1.0]])  # tuned from flight room tests

    return A, B, C, K