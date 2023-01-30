#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from trajectory_msgs.msg import JointTrajectory

import common.params as params
from common.utils import unwrap_2D_traj_msg


class OpenLoopCmdSender():
    """Open-loop cmd sender node

    Receives trajectories and pops off commands to send.
    
    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('open_loop_cmd_sender')
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.current_trajectory = None
        self.next_trajectory = None

        # Publishers
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Subscribers
        traj_sub = rospy.Subscriber('planner/traj', JointTrajectory, self.traj_callback)

        # # Logging
        # path = '/home/navlab-nuc/Rover/flightroom_data/4_12_2022/debug/'
        # filename = 'track_'+str(rospy.get_time())+'.csv'
        # self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        # self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta', 
        #                       'x_nom', 'y_nom', 'theta_nom', 'v_nom'])


    def traj_callback(self, data):
        """Trajectory subscriber callback.

        Save received trajectory.

        """
        # Unwrap trajectory message
        time = params.T_VEC  # TODO: add t2start from message
        traj = unwrap_2D_traj_msg(data, time)
        print("Received trajectory: ", traj)

        # Compute twist controls


        # self.new_traj_flag = True
        # # If no current trajectory yet, set it
        # if self.current_trajectory is None:
        #      pass
        # # Otherwise, set next trajectory
        # else:
        #     pass
        # rospy.loginfo("Received trajectory of length %d", len(data.states))


    def track(self):
        """Track next point in the current trajectory.
        TODO
        """
        pass

    
    # def stop_motors(self):
    #     """Send stop command to all motors
    #     """
    #     rospy.loginfo("Stopping motors")
    #     motor_cmd = Twist()
    #     motor_cmd.linear.x = 0.0
    #     motor_cmd.angular.z = 0.0
    #     self.cmd_pub.publish(motor_cmd)


    def run(self):
        """Run node
        """
        rospy.loginfo("Running Open-loop cmd sender")
        while not rospy.is_shutdown():
            
            if self.current_trajectory is not None:
                self.track()

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    olcs = OpenLoopCmdSender()
    try:
        olcs.run()
    except rospy.ROSInterruptException:
        pass