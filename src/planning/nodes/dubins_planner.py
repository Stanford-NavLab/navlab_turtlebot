#!/usr/bin/env python

"""Linear Planner

ROS Linear Planner node.

"""

import rospy
import rospkg
import sys
import numpy as np
import time
import argparse
from scipy.optimize import minimize, NonlinearConstraint
from scipy.spatial.transform import Rotation as R

from nav_msgs.msg import Odometry

import common.params as params
from common.utils import rand_in_bounds, wrap_states, unwrap_states, rot_mat_2D
from common.msg import Control, NominalTrajectory
from planning.dubins_planning_model import dubins_traj


class DubinsPlanner:
    """Dubins Planner node

    Dubins trajectory planner which recomputes collision-free trajectories to follow 
    in a receding-horizon fashion.

    Trajectory: time-indexed positions, velocities, accelerations
    Plan: trajectory and associated reachable set(s)
    
    Attributes
    ----------

    Methods
    -------

    """
    def __init__(self, name='/'):
        self.name = name

        # Initialize node
        rospy.init_node(self.name + 'dubins_planner', disable_signals=True)
        self.rate = rospy.Rate(10)

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_REPLAN), self.replan)

        # Publishers
        self.traj_pub = rospy.Publisher(self.name + '/planner/traj', NominalTrajectory, queue_size=10)
        self.traj_msg = None

        # Subscribers
        odom_sub = rospy.Subscriber(self.name + '/odom', Odometry, self.odom_callback)
        self.odom = None  # [x, y, theta]

        # Initial state (x, y, theta) and goal position (x, y)
        if self.name == 'turtlebot1':
            self.p_0 = np.array([0, 0, 0])
            self.p_goal = np.array([5, 0])
            self.peers = ['turtlebot2']
        elif self.name == 'turtlebot2':
            self.p_0 = np.array([5, 0, np.pi])
            self.p_goal = np.array([0, 0])
            self.peers = ['turtlebot1']
        else:
            self.p_0 = np.array([0, 0, 0])
            self.p_goal = np.array([5, 0])
            self.peers = []

        # Subscribe to peer trajectories
        for peer in self.peers:
            traj_sub = rospy.Subscriber(peer + '/planner/traj', NominalTrajectory, self.traj_callback, callback_args=peer)
        self.peer_traj = {}

        # Obstacles
        self.obstacles = params.OBSTACLES

        self.done = False  # Done planning flag

        # Pre-sample trajectories
        self.u_samples = rand_in_bounds([0, params.V_MAX, -params.W_MAX, params.W_MAX], params.N_PLAN_MAX)
        self.traj_samples = np.zeros((params.N_PLAN_MAX, params.TRAJ_IDX_LEN, 3))

        for i, u in enumerate(self.u_samples):
            self.traj_samples[i,:,:] = dubins_traj(np.zeros(3), u, params.TRAJ_IDX_LEN, params.DT)



    def traj_callback(self, msg, topic):
        """Trajectory subscriber callback

        Parameters
        ----------
        msg : NominalTrajectory
            Trajectory message

        """
        self.peer_traj[topic] = unwrap_states(msg.states)


    def odom_callback(self, msg):
        """Odometry callback function

        Parameters
        ----------
        msg : Odometry
            Odometry message

        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        r = R.from_quat(q)
        theta = r.as_euler('xyz')[2]
        self.odom = np.array([x, y, theta])


    def traj_opt(self, init_pose, t_start_plan):
        """Trajectory Optimization

        Attempt to find a collision-free plan (v_peak) which brings the agent 
        closest to its goal.

        Parameters
        ----------
        t_start_plan : float
            Time at which planning started for this iteration

        Returns
        -------
        np.array or None
            Optimal v_peak or None if failed to find one
        
        """
        def cost(u):
            traj = dubins_traj(init_pose, u, params.TRAJ_IDX_LEN, params.DT)
            dist = np.linalg.norm(traj[-1,:-1] - self.p_goal)
            return dist

        def constraint(u):
            traj = dubins_traj(init_pose, u, params.TRAJ_IDX_LEN, params.DT)
            dists = []
            for obs_c, obs_r in self.obstacles:
                dist = np.linalg.norm(traj[:,:-1] - obs_c, axis=1) - (obs_r + params.R_BOT)
                dists.append(dist)
            # for peer in self.peer_traj:
            #     dist = np.linalg.norm(traj[:,:-1] - self.peer_traj[peer][:,:-1], axis=1) - (2 * params.R_BOT)
            #     dists.append(dist)
            return np.hstack(dists)

        start_time = time.time()
        cons = NonlinearConstraint(constraint, 0, np.inf)
        u0 = rand_in_bounds([-params.V_MAX, params.V_MAX, -params.W_MAX, params.W_MAX], 1)[0]
        res = minimize(cost, u0, method='SLSQP', bounds=[(-params.V_MAX, params.V_MAX), (-params.W_MAX, params.W_MAX)], constraints=cons, 
                    options={'disp': False,
                             'ftol': 1e-6})
        print("Time elapsed: {:.3f} s".format(time.time() - start_time))
        print(res.x)
        mean = [0,0]                                                               # test line
        covariance = [[0,0], [0,.01]]                                              # test line
        return res.x + np.random.multivariate_normal(mean,covariance)              # test line
        #return res.x                                                              # original line
    

    def check_collisions(self, traj):
        """Check if the trajectory collides with any obstacles or peer trajectories."""
        for obs_c, obs_r in self.obstacles:
            dist = np.linalg.norm(traj[:,:-1] - obs_c, axis=1)
            if np.any(dist < obs_r + params.R_BOT):
                return True
        for peer in self.peer_traj:
            dist = np.linalg.norm(traj[:,:-1] - self.peer_traj[peer][:,:-1], axis=1)
            if np.any(dist < 2 * params.R_BOT):
                return True
        return False 


    def traj_opt_sample(self, init_pose, t_start_plan):
        """Sampling-based Trajectory Optimization

        Attempt to find a collision-free plan (v_peak) which brings the agent 
        closest to its goal.

        Parameters
        ----------
        t_start_plan : float
            Time at which planning started for this iteration

        Returns
        -------
        np.array or None
            Optimal v_peak or None if failed to find one
        
        """
        start_time = time.time()
        # u_samples = rand_in_bounds([0, params.V_MAX, -params.W_MAX, params.W_MAX], params.N_PLAN_MAX)
        # traj_samples = np.zeros((params.N_PLAN_MAX, params.TRAJ_IDX_LEN, 3))
        # for i, u in enumerate(u_samples):
        #     traj_samples[i,:,:] = dubins_traj(self.p_0, u, params.TRAJ_IDX_LEN, params.DT)
        #     endpoints[i] = traj[-1,:-1]

        # endpoints = traj_samples[:,-1,:-1]
        # dists = np.linalg.norm(endpoints - self.p_goal, axis=1)
        # sort_idxs = np.argsort(dists)
        # u_samples_sorted = u_samples[sort_idxs]
        # traj_samples_sorted = traj_samples[sort_idxs]

        # Transform samples to global frame using init_pose
        traj_samples_global = self.traj_samples.copy()
        traj_samples_global[:,:,0:2] = traj_samples_global[:,:,0:2] @ rot_mat_2D(init_pose[2]).T  # rotate
        traj_samples_global += init_pose  # translate

        endpoints = traj_samples_global[:,-1,:-1]
        dists = np.linalg.norm(endpoints - self.p_goal, axis=1)
        sort_idxs = np.argsort(dists)
        u_samples_sorted = self.u_samples[sort_idxs]
        traj_samples_sorted = traj_samples_global[sort_idxs]

        # Check collisions
        for i, u in enumerate(u_samples_sorted):
            traj = traj_samples_sorted[i]
            if self.check_collisions(traj):
                continue
            else:
                print("found plan ", u)
                print("Time elapsed: {:.3f} s".format(time.time() - start_time))
                return u
        print("No feasible plan found")
        return None
                


    def replan(self, event):
        """Replan
        
        Periodically called to perform trajectory optimization.
        
        """
        t_start_plan = time.time()

        # Find a new v_peak
        init_pose = self.odom
        #u = self.traj_opt(init_pose, t_start_plan)
        u = self.traj_opt_sample(init_pose, t_start_plan)

        if u is None:
            # Failed to find new plan
            print("Failed to find new plan")
        else:
            # Generate new trajectory
            traj = dubins_traj(init_pose, u, params.TRAJ_IDX_LEN, params.DT)

            # Update initial conditions for next replanning instance
            self.p_0 = traj[params.NEXT_IC_IDX]

            print("Found new trajectory, u = " + str(np.round(u, 2)))
            print(" Start point: " + str(np.round(traj[0], 2)))
            print(" End point: " + str(np.round(traj[-1], 2)))

            # Create and send trajectory msg
            self.traj_msg = NominalTrajectory()
            self.traj_msg.states = wrap_states(traj)
            self.traj_msg.control = Control(u[0], u[1])
            self.traj_pub.publish(self.traj_msg)
        
            # Check for goal-reached
            if np.linalg.norm(traj[-1,:-1] - self.p_goal) < params.R_GOAL_REACHED:
                print("Goal reached")
                self.done = True

    
    def run(self):
        """Run node
        """
        rospy.loginfo("Running Dubins Planner for %s", self.name)

        while not rospy.is_shutdown() and not self.done:
            # connections = self.traj_pub.get_num_connections()
            # rospy.loginfo("Waiting for tracker, connections: %d", connections)

            # if connections > 0:
            
            # loop while replan is periodically called by Timer

            self.rate.sleep()

        rospy.loginfo("Exiting node")


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args = argParser.parse_args()

    dp = DubinsPlanner(args.name)
    try:
        dp.run()
    except rospy.ROSInterruptException:
        pass
