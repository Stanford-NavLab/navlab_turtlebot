import numpy as np

# Timing parameters
DT = 0.1  # [s] trajectory time discretization
T_REPLAN = 0.5  # [s] amount of time between replans
T_PLAN = 0.5  # [s] amount of time allotted for planning itself 
              #     (allow buffer for actual tranmission of plan)
TRAJ_TIME_LEN = 3.0  # [s] Trajectory total duration
TRAJ_IDX_LEN = int(TRAJ_TIME_LEN / DT) + 1  # Trajectory length in timestep
T_VEC = np.linspace(0, TRAJ_TIME_LEN, TRAJ_IDX_LEN)  # Time vector

NEXT_IC_IDX = int(T_REPLAN / DT)  # Index in trajectory for next set of replanning initial conditions

# Robot parameters
N_DIM = 2  # workspace dimension (i.e. 2D or 3D)
R_BOT = 0.4  # [m]

# Max velocity constraints [m/s]
V_MAX = 0.22  # L1 velocity constraints
V_BOUNDS = np.tile(np.array([-V_MAX, V_MAX]), (1,N_DIM))[0]
V_MAX_NORM = 0.22  # L2 velocity constraints
DELTA_V_PEAK_MAX = 0.7  # Delta from initial velocity constraint

W_MAX = 1.0  # [rad/s] angular velocity constraint

# Planning parameters
R_GOAL_REACHED = 0.2  # [m] stop planning when within this dist of goal
N_PLAN_MAX = 5000  # Max number of plans to evaluate
MODEL_NAME = 'quadrotor_linear_planning_model.mat'

# Scenario parameters
P_0 = np.array([0, 0])  # Initial position
P_GOAL = np.array([5, 0])  # Goal position

OBSTACLES = [(np.array([3, 0]), 0.25),
             (np.array([-4, 0]), 0.5)]