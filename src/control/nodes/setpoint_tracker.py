#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os

from geometry_msgs.msg import Twist


class SetpointTracker():
    """Setpoint tracker

    """
    def __init__(self):
        # Initialize node 
        rospy.init_node('setpoint_tracker', anonymous=True)
        self.rate = rospy.Rate(100)  # NOTE: can adjust this

        # Publishers
        self.cmd_pub = rospy.Publisher('cmd_vel', Twist, queue_size=10)

        # Subscribers
        mocap_sub = rospy.Subscriber('sensing/mocap', State, self.mocap_callback)

        # Logging
        path = '/home/navlab-nuc/flightroom_data/4_12_2022/debug/'
        filename = 'track_'+str(rospy.get_time())+'.csv'
        self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta', 
                              'x_nom', 'y_nom', 'theta_nom', 'v_nom', 
                              'x_hat', 'y_hat', 'theta_hat', 'v_hat', 'u_w', 'u_a'])


    def mocap_callback(self, data):
        """Mocap subscriber callback.

        Save received ground-truth.

        """
        pass


    def run(self):
        """Run node

        """
        rospy.loginfo("Running Setpoint Tracker")
        while not rospy.is_shutdown():

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    st = SetpointTracker()
    try:
        st.run()
    except rospy.ROSInterruptException:
        pass
