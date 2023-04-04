#!/usr/bin/env python
""" Sends zero velocity command failsafe.

Publishes zeros to cmd_vel for every robot which does not have
another node publishing to it's cmd_vel.

Publishes to topics called: "/turtlebot*/cmd_vel"

"""

__authors__ = "D. Knowles"
__date__ = "04 Mar 2023"

import rospy
import numpy as np
from geometry_msgs.msg import Twist

class ZeroCmdFailsafe():
    """Sends goals to turtlebots.

    """
    def __init__(self):

        # Initialize ROS node.
        rospy.init_node('zero_command_failsafe', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(10) # 10Hz sleep rate

        # initialize subscriber
        self.sub = rospy.Subscriber("cmd_vel", Twist,
                               callback = self.cmd_callback)

        # initialize publisher
        self.pub = rospy.Publisher("cmd_vel", Twist, queue_size=10)

        # number of seconds after which failsafe is started
        self.failsafe_timeout = 2
        self.last_cmd = rospy.get_rostime()

        while not rospy.is_shutdown():
            if self.sub.get_num_connections() <= 1 or rospy.get_rostime() - self.last_cmd >= rospy.Duration(self.failsafe_timeout):
                self.publish_zero_command()
            rate.sleep()

    def cmd_callback(self, data_measured):
        """Callback function when new cmd_vel messages are published.

        Parameters
        ----------
        data_measured : geometry_msgs/Twist
            Data from the /cmd_vel topic that was published.

        """

        data = [data_measured.linear.x,
                data_measured.linear.y,
                data_measured.linear.z,
                data_measured.angular.x,
                data_measured.angular.y,
                data_measured.angular.z,
        ]
        if np.any(data != 0.):
            self.last_cmd = rospy.get_rostime()

    def publish_zero_command(self):
        """Publish zero command velocity.

        """
        msg = Twist()
        self.pub.publish(msg)
        rospy.logwarn("zero_command_failsafe publishing zero command")


    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """

        self.publish_zero_command()

        # sleep until only 1 connection
        while self.sub.get_num_connections() > 1:
            pass

        self.publish_zero_command()
        print("closing zero_cmd_failsafe safely")

if __name__ == '__main__':
    try:
        zcf = ZeroCmdFailsafe()
        rospy.on_shutdown(zcf.cleanup)
    except rospy.ROSInterruptException:
        pass
