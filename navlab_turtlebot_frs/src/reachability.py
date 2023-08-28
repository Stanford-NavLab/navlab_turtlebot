"""Reachability functions

"""

import numpy as np

from zonotope import Zonotope
from probzonotope import probZonotope
from utils import remove_zero_columns


def compute_PRS(p_0, traj=None, N=50):
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
    """
    k_0 = np.hstack((v_0, a_0)).T
    # Form trajectory parameter space zonotope
    # V_pk_c = np.vstack((k_0, np.zeros(params.N_DIM)))
    # V_pk_c = V_pk_c.reshape((-1,1), order='F')
    # V_pk_G = np.kron(np.eye(params.N_DIM), np.array([0,0,params.V_MAX])[:,None])
    #print(V_pk_c, V_pk_G)
    V_pk = Zonotope(np.zeros((params.N_DIM,1)), params.V_MAX * np.eye(params.N_DIM))
    #print(V_pk)
    """
    PRS = N * [None]
    V_pk = Zonotope(np.zeros((2,1)), .22 * np.eye(2))
    
    for t in range(N):
        t_sim = t * .1
        # Ignoring acceleration and orientation, just gliding at top speed
        if traj is None:
            pos = t_sim * V_pk
            PRS[t] = pos.augment(V_pk) + np.vstack((p_0[:,None], np.zeros((2,1))))
        else:
            # If the trajectory is shorter than the provided N, then cut off the PRS early
            PRS = PRS[:len(traj[0])]
            center = np.zeros((4,1))
            center[:2] = traj[:,t].reshape(2,1)
            # Covariance increases as driving away, decreases upon nearing goal, forming a parabola with time
            PRS[t] = probZonotope(center, np.zeros((4,2)), np.array([[1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                     [1-(t_sim-5)**2,1-(t_sim-5)**2,0,0], \
                                                                     [0,0,0,0],[0,0,0,0]]))
    """
    if N is None:
        # Initialize
        N = len(LPM.time)
        PRS = N * [None]

        # Equation 6.10 from Shreyas' dissertation
        init_traj = LPM.P_mat.T[:,:2] @ k_0 + p_0.flatten()  # trajectory due to initial conditions only

        #PRS[0] = Zonotope(np.zeros((2*params.N_DIM,1)), np.zeros((2*params.N_DIM,1))) << WTF DID THIS DO

        # For now, we only consider fixed v_0 and a_0
        # TODO: handle zonotope v_0 and a_0
        for i in range(N): # range(1,N): << PART TWO OF WHY, JUST WHY
            pos = LPM.P_mat[2,i] * V_pk  # position zonotope
            PRS[i] = pos.augment(V_pk) + np.vstack((init_traj[i][:,None], np.zeros((params.N_DIM,1))))
    else:  
        PRS = N * [None]
        init_p = p_0
        init_v = v_0
        init_a = a_0
        k_0 = np.hstack((v_0, a_0)).T
        bonus = np.zeros((4,1))
        speed_boost = np.zeros((4,1))
        init_traj = LPM.P_mat.T[:,:2] @ k_0 + init_p.flatten()
        hz = 16 # Replans at this rate result in largest starting velocity
        k_boost = np.array([[2,2],[0,0]])
        new_traj = LPM.P_mat.T[:,:2] @ k_boost
        for i in range(N):
            # If on a new planning cycle, add a speed boost to account for max velocity and add onto the last zonotope
            if i % hz == 0 and i!=0:
                bonus = Zonotope(np.zeros((4,1)), PRS[i-1].G)
            if i >= hz:
                generators = np.vstack((new_traj[i%hz,:]*np.eye(2),np.zeros((2,2))))
                speed_boost = Zonotope(np.zeros((4,1)), generators)
            # Update the state zonotopes
            pos = LPM.P_mat[2,i%hz] * V_pk
            PRS[i] = pos.augment(V_pk) + np.vstack((init_traj[i%hz][:,None], np.zeros((params.N_DIM,1)))) + speed_boost + bonus"""
    return PRS

def compute_FRS(p_0, traj=None, N=50):
    """Compute FRS
    
    FRS = PRS + ERS

    For, we use a constant zonotope for ERS. ERS includes robot body
    
    """
    FRS = N * [None]
    PRS = compute_PRS(p_0, traj=traj, N=N)
    
    # If we are computing just a normal total FRS
    if traj is None:
        ERS = Zonotope(np.zeros((4,1)), np.vstack(((0 + .178) * np.eye(2), np.zeros((2, 2)))))
    else:
        """
        PRS = []
        trajectory = traj
        for i in range(N):
            center = np.zeros((4,1))
            center[:2] = trajectory[0][:,i].reshape(2,1)
            PRS.append(probZonotope(center, np.zeros((4,2)), np.array([[.028*np.exp(.05*(i+t)),-.096*np.exp(.017*(i+t)),0,0], \
                                                                       [-.096*np.exp(.017*(i+t)),.066*np.exp(.042*(i+t)),0,0], \
                                                                       [0,0,0,0],[0,0,0,0]])))"""
        FRS = FRS[:len(traj[0])] # Same thing was with PRS
        ERS = probZonotope(np.zeros((4,1)), np.zeros((4,2)), np.array([[0,0,0,0],[0,0,0,0],[0,0,0,0],[0,0,0,0]]))
        
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