#!/usr/bin/env python

"""Obstacle Publisher

Publishes list of static obstacles for TEB local planner.
TODO: also subscribe to robot poses and publish those as dynamic obstacles.

"""

import rospy
import rospkg
import sys
import os
import numpy as np
import time
import argparse

from tf.transformations import euler_from_quaternion
from geometry_msgs.msg import Point32
from nav_msgs.msg import Odometry
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg

class ObstaclePublisher:
    """Obstacle Publisher node

    """
    def __init__(self):

        # Initialize node 
        rospy.init_node('obstacle_publisher', disable_signals=True)
        self.rate = rospy.Rate(10)

        # Publishers
        self.obs_pub = rospy.Publisher('/turtlebot1/move_base/TebLocalPlannerROS/obstacles', 
                                       ObstacleArrayMsg, queue_size=1)

        # Subscribers
        odom_sub = rospy.Subscriber('/turtlebot1/odom', Odometry, self.odom_callback)
        self.odom = None

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


    def publish_obstacles(self):
        """Publish obstacles to TEB local planner

        """
        obs_msg = ObstacleArrayMsg()
        obs_msg.header.stamp = rospy.Time.now()
        obs_msg.header.frame_id = 'map'

        # Add obstacles
        obs = ObstacleMsg()
        obs.header.stamp = rospy.Time.now()
        obs.header.frame_id = 'map'
        obs.id = 0
        # Circle obstacle
        obs.polygon.points = [Point32(x=3.0, y=0.0, z=0.0)]
        obs.radius = 0.5
        obs_msg.obstacles.append(obs)

        # Publish
        self.obs_pub.publish(obs_msg)

    
    def run(self):
        """Run obstacle publisher

        """
        rospy.loginfo('Running obstacle publisher')
        while not rospy.is_shutdown():
            self.publish_obstacles()
            self.rate.sleep()


if __name__ == '__main__':

    obstacle_publisher = ObstaclePublisher()
    try:
        obstacle_publisher.run()
    except rospy.ROSInterruptException:
        pass