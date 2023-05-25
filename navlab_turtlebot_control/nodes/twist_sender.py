#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os
import argparse

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64, Int16MultiArray

from navlab_turtlebot_common.msg import State, Control, NominalTrajectory
import navlab_turtlebot_common.params as params

class TwistSender():
    """Twist Sender

    """
    def __init__(self, name=''):
        self.name = name

        # Initialize node
        rospy.init_node(self.name + 'twist_sender', anonymous=True)
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.X_nom = None
        self.u_nom = None
        #self.no_comms = [0,0]
        self.comms_list = [0,0,0,0]

        # Publishers
        self.cmd_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        # Subscribers
        traj_sub = rospy.Subscriber(self.name + '/planner/traj', NominalTrajectory, self.traj_callback)
        comms_sub = rospy.Subscriber('/comms_list',Int16MultiArray,self.comms_cb)

    def traj_callback(self, data):
        """Trajectory subscriber callback.
        Save nominal states and controls from received trajectory.
        """
        self.X_nom = data.states
        self.u_nom = data.control
        self.idx = 0
        rospy.loginfo("Received trajectory of length %d", len(data.states))

    def comms_cb(self,multiarr):
        """
        Saves if communications have been lost between agents.
        """
        self.comms_list = multiarr.data

    def track(self):
        """Track next point in the current trajectory, and run the EKF for estimation.
        TODO
        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom_msg = self.X_nom[self.idx]
        x_nom = np.array([x_nom_msg.x, x_nom_msg.y, x_nom_msg.theta])
        u_nom_msg = self.u_nom
        u_nom = np.array([u_nom_msg.v, u_nom_msg.omega])

        # Create cmd_vel msg
        # First check if bot in main group with priority to move
        main = self.name=="turtlebot1" or \
               (self.name=="turtlebot2" and self.comms_list[1]==self.comms_list[0]) or \
               (self.name=="turtlebot3" and self.comms_list[2]==self.comms_list[0]) or \
               (self.name=="turtlebot4" and self.comms_list[3]==self.comms_list[0])
        if sum(self.comms_list)==0 or (sum(self.comms_list)!=0 and main):
            cmd = Twist()
            cmd.linear.x = u_nom[0]
            cmd.angular.z = u_nom[1]
            self.cmd_pub.publish(cmd)
            self.idx += 1
        else:
            self.stop_motors()

        if self.idx >= len(self.X_nom):
            self.X_nom = None
            self.u_nom = None
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
        rospy.loginfo("Running Twist Sender for %s", self.name)
        while not rospy.is_shutdown():

            if self.X_nom is not None:
                self.track()

            self.rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args = argParser.parse_args()

    ts = TwistSender(args.name)
    try:
        ts.run()
    except rospy.ROSInterruptException:
        pass
