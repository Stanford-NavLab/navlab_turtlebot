#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from trajectory_msgs.msg import JointTrajectory

import common.params as params
from planning.utils import unwrap_2D_traj_msg
from common.utils import wrap_angle


class Tracker():
    """Trajectory tracker node

    Tracks double integrator trajectories with twist commands.
    
    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('tracker')
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.trajectory = None
        self.twists = None

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)
        # Twist angular z: positive is CCW, negative is CW (TODO: Check this)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', JointTrajectory, self.traj_callback)
        odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.odom = None  # [x, y, theta]

        # # Logging
        # path = '/home/navlab-nuc/Rover/flightroom_data/4_12_2022/debug/'
        # filename = 'track_'+str(rospy.get_time())+'.csv'
        # self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        # self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta', 
        #                       'x_nom', 'y_nom', 'theta_nom', 'v_nom'])


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


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save received trajectory.

        """
        # Unwrap trajectory message
        time = params.T_VEC  # TODO: add t2start from message
        traj = unwrap_2D_traj_msg(data, time)
        #print("Received trajectory: ", traj.positions)

        # Compute twist controls
        traj.compute_thetas()
        twists = traj.compute_twist_controls()
        #print("Twists: ", twists)
        self.twists = twists

        self.trajectory = traj
        self.twists = twists
        self.idx = 0

        rospy.loginfo("Received trajectory of length %d", traj.length)


    def track(self):
        """Track next point in the current trajectory.
        TODO
        """
        print("idx ", self.idx, " ----------------------------------------")
        # TODO: add feedback based on odometry
        theta_hat = self.odom[2]
        theta_nom = self.trajectory.thetas[self.idx]
        theta_err = wrap_angle(theta_nom - theta_hat)
        K_theta = 0.5

        twist = Twist()
        twist.linear.x = self.twists[self.idx, 0]
        twist.angular.z = self.twists[self.idx, 1] + K_theta * theta_err
        print("     cmd: v = ", twist.linear.x, ", w = ", twist.angular.z)
        self.cmd_pub.publish(twist)
        self.idx += 1


        if self.idx >= self.trajectory.length:
            self.trajectory = None
            self.twists = None
            self.idx = 0
            self.stop_motors()

    
    def stop_motors(self):
        """Send stop command to all motors
        """
        rospy.loginfo("Stopping motors")
        motor_cmd = Twist()
        motor_cmd.linear.x = 0.0
        motor_cmd.angular.z = 0.0
        self.cmd_pub.publish(motor_cmd)


    def run(self):
        """Run node
        """
        rospy.loginfo("Running tracker")
        while not rospy.is_shutdown():
            
            if self.trajectory is not None:
                self.track()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    tracker = Tracker()
    try:
        tracker.run()
    except rospy.ROSInterruptException:
        pass