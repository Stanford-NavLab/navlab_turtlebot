#!/usr/bin/env python

import rospy
import numpy as np
import time
import argparse

from tf.transformations import euler_from_quaternion
#from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64

from navlab_turtlebot_common.msg import State
from navlab_turtlebot_common.utils import wrap_angle
import navlab_turtlebot_common.params as params


class Odom():
    """Process and publish motion capture measurements
    """
    def __init__(self, name=''):
        self.name = name
        
        # Initialize node 
        node_name = name + '_odom' if name else 'odom'
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(10)

        # Class variables
        self.x = 0
        self.y = 0
        self.theta = 0
        self.x_std = params.MOCAP_SIGMA[0]
        self.y_std = params.MOCAP_SIGMA[1]
        self.theta_std = params.MOCAP_SIGMA[2]

        # Publishers and subscribers
        vrpn_sub = rospy.Subscriber('/' + name + '/odom', Odometry, self.odom_callback)
        self.mocap_pub = rospy.Publisher('/' + name + '/sensing/mocap', State, queue_size=1)
        self.mocap_noisy_pub = rospy.Publisher('/' + name + '/sensing/mocap_noisy', State, queue_size=1)


    def odom_callback(self, msg):
        """Odometry callback function

        Parameters
        ----------
        msg : Odometry
            Odometry message

        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        # r = R.from_quat(q)
        # theta = r.as_euler('xyz')[2]
        angles = euler_from_quaternion(q)
        self.theta = wrap_angle(angles[2])

        self.x = x; self.y = y

        rospy.loginfo("Received data: (%f, %f, %f)", self.x, self.y, self.theta)

        self.publish()


    def publish(self):
        """Publish latest ground-truth and noisy measurements
        """
        s = State()
        s.x = self.x
        s.y = self.y
        s.theta = self.theta
        self.mocap_pub.publish(s)

        s = State()
        s.x = self.x + np.random.normal(0, self.x_std)
        s.y = self.y + np.random.normal(0, self.y_std)
        s.theta = self.theta + np.random.normal(0, self.theta_std)
        self.mocap_noisy_pub.publish(s)


    def run(self):
        rospy.loginfo("Running Mocap node")
        while not rospy.is_shutdown():

            self.rate.sleep()
        
        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args = argParser.parse_args()

    odom = Odom(args.name)
    try:
        odom.run()
    except rospy.ROSInterruptException:
        pass