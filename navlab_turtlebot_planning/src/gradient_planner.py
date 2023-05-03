#!/usr/bin/env python

"""Linear Planner

ROS Linear Planner node.

"""

import rospy
import rospkg
import sys
import os
import numpy as np
import time
import argparse
from scipy.optimize import minimize
#from scipy.spatial.transform import Rotation as R

from tf.transformations import euler_from_quaternion
from nav_msgs.msg import Odometry

import navlab_turtlebot_common.params as params
from navlab_turtlebot_common.utils import rand_in_bounds, wrap_states, unwrap_states, rot_mat_2D
from navlab_turtlebot_common.msg import State, Control, NominalTrajectory
from navlab_turtlebot_planning.dubins_planning_model import dubins_traj


class GradientPlanner:
    """Gradient Planner node

    Dubins trajectory planner which recomputes collision-free trajectories to follow
    in a receding-horizon fashion.

    Trajectory: time-indexed positions, velocities, accelerations
    Plan: trajectory and associated reachable set(s)

    Attributes
    ----------

    Methods
    -------

    """
    def __init__(self, name=''):
        self.name = name

        # Print python version
        print("Using Python ", sys.version)

        # Initialize node 
        rospy.init_node(name + '_gradient_planner', disable_signals=True)
        self.rate = rospy.Rate(10)

        # Replan timer
        rospy.Timer(rospy.Duration(params.T_REPLAN), self.replan)

        # Publishers
        self.traj_pub = rospy.Publisher('/' + name + '/planner/traj', NominalTrajectory, queue_size=10)
        self.traj_msg = None

        # Subscribers
        #odom_sub = rospy.Subscriber(self.name + '/odom', Odometry, self.odom_callback)
        mocap_sub = rospy.Subscriber('/' + name + '/sensing/mocap', State, self.mocap_callback)
        self.odom = np.zeros(3)  # [x, y, theta]

        # Initial state (x, y, theta) and goal position (x, y)
        if self.name == 'turtlebot1':
            self.p_0 = np.array([0, 0, 0])
            self.p_goal = np.array([5, 0])
            self.peers = ['turtlebot2']
        elif self.name == 'turtlebot2':
            self.p_0 = np.array([5, 0, np.pi])
            self.p_goal = np.array([0, 0])
            self.peers = ['turtlebot1']
        elif self.name == 'turtlebot4':
            self.p_goal = np.array([1, 0])
            self.peers = []
        else:
            self.p_0 = np.array([0, 0, 0])
            self.p_goal = np.array([5, 0])
            self.peers = []

        print("GOAL: ", str(self.p_goal))

        # Subscribe to peer trajectories
        for peer in self.peers:
            traj_sub = rospy.Subscriber('/' + peer + '/planner/traj', NominalTrajectory, self.traj_callback, callback_args=peer)
        self.peer_traj = {}

        # Obstacles
        self.obstacles = params.OBSTACLES

        self.done = False  # Done planning flag


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
        angles = euler_from_quaternion(q)
        theta = angles[2]
        self.odom = np.array([x, y, theta])


    def mocap_callback(self, msg):
        """
        """
        self.odom = np.array([msg.x, msg.y, msg.theta])


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
            goal_cost = np.linalg.norm(traj[-1,:-1] - self.p_goal)

            # Obstacle collision cost
            obs_cost = 0
            for obs_c, obs_r in self.obstacles:
                dist = np.linalg.norm(traj[:,:-1] - obs_c, axis=1)
                obs_cost += 1 / np.min(dist)

            return goal_cost + obs_cost

        start_time = time.time()
        # u0 = rand_in_bounds([0, params.V_MAX, -params.W_MAX, params.W_MAX], 1)[0]
        # res = minimize(cost, u0, method='L-BFGS-B', bounds=[(0, params.V_MAX), (-params.W_MAX, params.W_MAX)],
        #             options={'disp': False,
        #                      #'ftol': 1e-6
        #                      })
        # print("Time elapsed: {:.3f} s".format(time.time() - start_time))
        # return res.x                                                           
        u_samples = rand_in_bounds([0, params.V_MAX, -params.W_MAX, params.W_MAX], params.N_PLAN_MAX)
        costs = np.array([cost(u) for u in u_samples])
        min_idx = np.argmin(costs)
        opt_cost, opt_u = costs[min_idx], u_samples[min_idx]
        print("Time elapsed: {:.3f} s".format(time.time() - start_time))
        return opt_u



    def replan(self, event):
        """Replan

        Periodically called to perform trajectory optimization.

        """
        t_start_plan = time.time()

        # Find a new v_peak
        init_pose = self.odom
        #u = self.traj_opt(init_pose, t_start_plan)
        u = self.traj_opt(init_pose, t_start_plan)

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

    # Auto-get turtlebot name
    if args.name is None:
        args.name = os.environ['USER']

    gp = GradientPlanner(args.name)
    try:
        gp.run()
    except rospy.ROSInterruptException:
        pass
