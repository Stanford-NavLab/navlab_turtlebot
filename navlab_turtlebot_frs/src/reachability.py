"""Reachability functions

"""

import numpy as np

from zonotope import Zonotope
from probzonotope import probZonotope
from utils import remove_zero_columns

import rospy


def compute_PRS(p_0, traj=None, N=50, split=None):
    """Compute Planning Reachable Set (PRS)
    
    PRS desribes the reachable positions of planned trajectories over a 
    space of chosen trajectory parameters (v_peak). These sets do not account  
    for any deviations from the planned trajectories to the actual trajectory
    executed by the robot (this is handled by the Error Reachable Set or ERS).
    They can however, model uncertainty in initial conditions v_0 and a_0 
    (e.g. provided by state estimator covariance).

    PRS is of dimension 2*N_DIM. It has N_DIM dimensions for position, and N_DIM 
    dimensions for peak velocities, so that it can later be sliced in the peak 
    velocity dimensions for trajectory planning.

    NOTE: for now, only use position as state. Need to investigate whether velocity 
          as state is beneficial

    PRS dimensions:
      0 - x  
      1 - y  
      2 - z
      3 - v_pk_x
      4 - v_pk_y
      5 - v_pk_z


    Parameters
    ----------
    LPM : LPM 
        Linear planning model object
    p_0 : np.array (N_DIM x 1)
        Initial position
    v_0 : np.array (N_DIM x 1)
        Initial velocity
    a_0 : np.array (N_DIM x 1)
        Initial acceleration
    V_pk : Zonotope
        Zonotope of candidate peak velocities which parameterize possible planned trajectories

    Returns
    -------
    PRS : list
        List of Zonotope objects describing reachable positions from planned trajectories

    """
    PRS = N * [None]
    V_pk = Zonotope(np.zeros((2,1)), .22 * np.eye(2))
    
    if traj is None:
        tf = N
    else:
        tf = len(traj[0])
        PRS = PRS[:tf]
    
    for t in range(tf):
        # The default is .3 for teb local planner, and I'm not about to figure out how to change that
        t_sim = t * .3
        # Ignoring acceleration and orientation, just gliding at top speed
        if traj is None:
            pos = t_sim * V_pk
            PRS[t] = pos.augment(V_pk) + np.vstack((p_0[:,None], np.zeros((2,1))))
        else:
            center = np.zeros((4,1))
            print(center[:2])
            print(traj[:,t].reshape(2,1))
            center[:2] = traj[:,t].reshape(2,1)
            # Covariance increases as driving away, decreases upon nearing goal, forming a parabola with time
            if t <= split:
                PRS[t] = probZonotope(center, np.zeros((4,2)), np.array([[1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                         [1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                         [0,0,0,0],[0,0,0,0]]))
            else:
                PRS[t] = probZonotope(center, np.zeros((4,2)), np.array([[1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                         [1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                         [0,0,0,0],[0,0,0,0]]))
    return PRS

def compute_FRS(p_0, traj=None, N=50, goal=None, args=5):
    """Compute FRS
    
    FRS = PRS + ERS

    For, we use a constant zonotope for ERS. ERS includes robot body
    
    """
    FRS = N * [None]
    
    # If the trajectory is shorter than the provided N, then connect end of trajectory to goal with straight line
    if not traj is None:
        split=None
        if len(traj[0]) < N:
            split=len(traj[0])
            # stuff for logging only on turtlebot1
            if args==0:
                rospy.set_param("split",split)
            x = goal[0] - traj[0,-1]
            y = goal[1] - traj[1,-1]
            hyp = (x**2 + y**2)**.5
            scale = 2/hyp
            x_pts = np.arange(traj[0,-1], goal[0], x*scale)
            y_pts = np.arange(traj[1,-1], goal[1], y*scale)
            extension = np.hstack((x_pts.reshape((2,len(x_pts))), y_pts.reshape((2,len(y_pts)))))
            extension = extension[:,:N-len(traj[0])]
            traj = np.hstack((traj,extension))
            FRS = FRS[:len(traj[0])]
    
    PRS = compute_PRS(p_0, traj=traj, N=N, split=None)
    
    # If we are computing just a normal total FRS
    if traj is None:
        ERS = Zonotope(np.zeros((4,1)), np.vstack(((0 + .178/2) * np.eye(2), np.zeros((2, 2)))))
    else:
        ERS = probZonotope(np.zeros((4,1)), \
                           np.vstack((.178/2 * np.eye(2), np.zeros((2, 2)))), \
                           np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]))
        
    # Add ERS
    for i, zono in enumerate(PRS):
        FRS[i] = zono + ERS
    return FRS

'''
def generate_collision_constraints_FRS(FRS, obs_map):
    """Generate collision constraints FRS

    Given an FRS and a set of obstacles, generate halfspace constraints
    in trajectory parameter space that constrain safe trajectory parameters.

    Parameters
    ----------
    FRS : list
        List of zonotopes representing forward reachable set
    obs_map : list
        List of zonotopes representing obstacles
    k_dim : np.array (1D)
        Dimensions of trajectory parameter 
    obs_dim : np.array (1D)
        Dimensions of obstacle

    Returns
    -------
    A_con : list
        List of constraint matrices
    b_con : list 
        List of constraint vectors 

    """
    A_con = []
    b_con = []

    # Skip first time step since it is not sliceable
    for z in FRS[1:]:
        # Extract center and generators of FRS
        c = z.c[params.OBS_DIM]
        G = z.G

        # Find columns of G which are nonzero in k_dim ("k-sliceable")
        # - this forms a linear map from the parameter space to workspace
        k_col = list(set(np.nonzero(G[params.K_DIM,:])[1]))
        k_slc_G = G[params.OBS_DIM][:,k_col]

        # "non-k-sliceable" generators - have constant contribution regardless of chosen trajectory parameter
        k_no_slc_G = G[params.OBS_DIM]
        k_no_slc_G = np.delete(k_no_slc_G, k_col, axis=1)

        # For each obstacle
        for obs in obs_map:
            # Get current obstacle
            obs = obs.Z

            # Obstacle is "buffered" by non-k-sliceable part of FRS
            buff_obs_c = obs[:,0][:,None] - c
            buff_obs_G = np.hstack((obs[:,1:], k_no_slc_G))
            buff_obs_G = remove_zero_columns(buff_obs_G)
            buff_obs = Zonotope(buff_obs_c, buff_obs_G)

            A_obs, b_obs = buff_obs.halfspace()
            A_con.append(A_obs @ k_slc_G)  # map constraints to be over coefficients of k_slc_G generators
            b_con.append(b_obs)
    
    return A_con, b_con

def generate_collision_constraints_agents(FRS, ag_map):
    """Generate collision constraints for single index of FRS

    Parameters
    ----------
    z : Zonotope
        Zonotope representing forward reachable set
    obs_map : list
        List of zonotopes representing obstacles

    Returns
    -------
    A_con : list
        List of constraint matrices
    b_con : list
        List of constraint vectors
    
    """
    A_con = []
    b_con = []

    # Track the current time step so it can be matched in the agent FRS
    t = 1
    # Skip first time step since it is not sliceable
    for z in FRS[1:]:
        # Extract center and generators of FRS
        c = z.c[params.OBS_DIM]
        G = z.G

        # Find columns of G which are nonzero in k_dim ("k-sliceable")
        # - this forms a linear map from the parameter space to workspace
        k_col = list(set(np.nonzero(G[params.K_DIM,:])[1]))
        k_slc_G = G[params.OBS_DIM][:,k_col]

        # "non-k-sliceable" generators - have constant contribution regardless of chosen trajectory parameter
        k_no_slc_G = G[params.OBS_DIM]
        k_no_slc_G = np.delete(k_no_slc_G, k_col, axis=1)

        # For each agent, generate constraints with the FRS at the same time point
        for ag in ag_map:
            # If we finished checking the agent FRS, stop
            if t>=len(ag):
                break
            current = ag[t]
            # Get current obstacle
            obs = current.view([0,1]).Z

            # Obstacle is "buffered" by non-k-sliceable part of FRS
            buff_obs_c = obs[:,0][:,None] - c
            buff_obs_G = np.hstack((obs[:,1:], k_no_slc_G))
            buff_obs_G = remove_zero_columns(buff_obs_G)
            buff_obs = Zonotope(buff_obs_c, buff_obs_G)

            A_obs, b_obs = buff_obs.halfspace()
            A_con.append(A_obs @ k_slc_G)  # map constraints to be over coefficients of k_slc_G generators
            b_con.append(b_obs)
        t+=1
    
    return A_con, b_con


def generate_collision_constraints(z, obs_map):
    """Generate collision constraints for single index of FRS

    Parameters
    ----------
    z : Zonotope
        Zonotope representing forward reachable set
    obs_map : list
        List of zonotopes representing obstacles

    Returns
    -------
    A_con : list
        List of constraint matrices
    b_con : list
        List of constraint vectors
    
    """
    A_con = []
    b_con = []
    # Extract center and generators of FRS
    c = z.c[params.OBS_DIM]
    G = z.G

    # Find columns of G which are nonzero in k_dim ("k-sliceable")
    # - this forms a linear map from the parameter space to workspace
    k_col = list(set(np.nonzero(G[params.K_DIM,:])[1]))
    k_slc_G = G[params.OBS_DIM][:,k_col]

    # "non-k-sliceable" generators - have constant contribution regardless of chosen trajectory parameter
    k_no_slc_G = G[params.OBS_DIM]
    k_no_slc_G = np.delete(k_no_slc_G, k_col, axis=1)

    # For each obstacle
    for obs in obs_map:
        # Get current obstacle
        obs = obs.Z

        # Obstacle is "buffered" by non-k-sliceable part of FRS
        buff_obs_c = obs[:,0][:,None] - c
        buff_obs_G = np.hstack((obs[:,1:], k_no_slc_G))
        buff_obs_G = remove_zero_columns(buff_obs_G)
        buff_obs = Zonotope(buff_obs_c, buff_obs_G)

        A_obs, b_obs = buff_obs.halfspace()
        A_con.append(A_obs @ k_slc_G)  # map constraints to be over coefficients of k_slc_G generators
        b_con.append(b_obs)
    
    return A_con, b_con

def check_collision_constraints(A_con, b_con, v_peak):
    """Check a trajectory parameter against halfspace collision constraints.
    
    Parameters
    ----------
    A_con : list
        List of halfspace constraint matrices
    b_con : list
        List of halfspace constraint vectors
    v_peak : np.array (N_DIM x 1)
        Trajectory parameter

    Returns
    -------
    bool
        True if the trajectory is safe, False if it results in collision

    """
    c = np.inf

    # Get the coefficients of the parameter space zonotope for this parameter
    # Assumes parameter space zonotope is centered at 0, v_max generators
    lambdas = v_peak / params.V_MAX

    for (A, b) in zip(A_con, b_con):
        c_tmp = A @ lambdas - b  # A*lambda - b <= 0 means inside unsafe set
        c_tmp = c_tmp.max()  # Max of this <= 0 means inside unsafe set
        c = min(c, c_tmp)  # Find smallest max. If it's <= 0, then parameter is unsafe
    
    return c > 0

# TODO: stack the As and bs
def check_collision_constraints_vectorized(A_con, b_con, v_peak):
    """Check a trajectory parameter against halfspace collision constraints.
    
    Parameters
    ----------
    A_con : list
        List of halfspace constraint matrices
    b_con : list
        List of halfspace constraint vectors
    v_peak : np.array (N_DIM x 1)
        Trajectory parameter

    Returns
    -------
    bool
        True if the trajectory is safe, False if it results in collision

    """
    N = len(b_con[0])
    A = np.vstack(A_con)
    b = np.vstack(b_con)

    # Get the coefficients of the parameter space zonotope for this parameter
    # Assumes parameter space zonotope is centered at 0, v_max generators
    lambdas = v_peak / params.V_MAX

    c = A @ lambdas - b
    c = c.reshape((-1,N))
    c = np.max(c, axis=1)
    c = np.min(c)
    
    return c > 0 
'''