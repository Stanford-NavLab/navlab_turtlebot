#!/usr/bin/env python
"""Extended Kalman Filter using UWB measurements.

"""

__authors__ = "D. Knowles"
__date__ = "28 Feb 2024"

import os
import argparse

import tf
import rospy
import numpy as np
import message_filters
from nav_msgs.msg import Odometry

from navlab_turtlebot_msgs.msg import UWBRange

class UWBEKF():
    """UWB Extended Kalman Filter

    Params
    ------
    name : string
        Turtlebot name used to publish messages.
    odom_source : string
        Odom source for robot.


    """
    def __init__(self, name="turtlebot3", odom_source=None):

        # Initialize node
        node_name = name + '_uwb_ekf'
        rospy.init_node(node_name, anonymous=True)
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(15) # 15Hz

        if name is None:
            self.name = os.environ['USER']
        else:
            self.name = name

        # odometry publisher
        self.odom_pub = rospy.Publisher('/' + name + '/odom_ekf',
                                        Odometry,
                                        queue_size = 10)

        # subscribers
        self.uwb_subscribers = set()

        # keep track of all odom subscribers
        self.odom_subscribers = set()
        self.odom_cache = {}

        # subscriber data
        self.data = {}

        # save last timestamp
        self.last_timestep = rospy.Time.now()

        self.uwb_positions = {}
        self.uwb_positions["turtlebot1"] = [3.048, 5.4864]
        self.uwb_positions["turtlebot2"] = [0.0, 4.2672]
        self.uwb_positions["turtlebot4"] = [0.0, 0.0]

        # measurements initialization
        self.features = np.array([[3.048,  0.0,    0.0],
                                  [5.4864, 4.2672, 0.0]]).T

        # EKF state is [x position, y position, heading angle]
        self.ekf_state = np.ones((3,1))
        self.sigma = 3.*np.eye(3)

        while not rospy.is_shutdown():

            self.publish_message()

            all_topics = rospy.get_published_topics()

            # update measured topics
            measured_topics = {x[0] for x in all_topics if "/" + self.name \
                                + "/uwb/" in x[0]}
            new_measured_topics = measured_topics - self.uwb_subscribers
            if len(new_measured_topics) > 0:
                self.subscribe_to_all(new_measured_topics)

            # odom_topics = {x[0] for x in all_topics if "/uwb/range_truth/" in x[0]}
            # new_truth_topics = truth_topics - self.truth_subscribers
            # if len(new_truth_topics) > 0:
            #     self.cache_all(new_truth_topics)

            rate.sleep()

    def subscribe_to_all(self, current_topics):
        """Subscribes to all new topics and adds to dictionary.

        Parameters
        ----------
        current_topics : list
            List of [topic_name, topic_type]

        """

        for topic_name in current_topics:
            # Subscribe to topic and set callback function

            uwb_source = self.get_uwb_source(topic_name)

            rospy.Subscriber(topic_name, UWBRange,
                             callback = self.uwb_callback,
                             callback_args = uwb_source)
            self.uwb_subscribers.add(topic_name)
            self.data[uwb_source] = np.nan

    # def cache_all(self, current_topics):
    #     """Subscribes to new topics and creates message_filters cache.
    #
    #     Parameters
    #     ----------
    #     current_topics : list
    #         List of [topic_name, topic_type]
    #
    #     """
    #     for topic_name in current_topics:
    #         # Subscribe to topic and set cache
    #         sub = message_filters.Subscriber(topic_name, RangeStamped)
    #
    #         sorted_topic_name = self.get_sorted_name(topic_name)
    #
    #         self.truth_cache[sorted_topic_name] = message_filters.Cache(sub, 100)
    #         self.truth_subscribers.add(topic_name)

    def uwb_callback(self, data_measured, uwb_source):
        """Callback function when new /uwb/ messages are published.

        Parameters
        ----------
        data_measured : ROS /range_measured/ UWBRange message
            Data from the topic that was published.
        uwb_source : string
            Source of UWB ranging meausrement.

        """

        self.data[uwb_source] = data_measured.range_dist_m

        if len(self.data) >= 3 and not np.any(np.isnan(list(self.data.values()))):
            timestamp_measured = data_measured.header.stamp
            time_delta = (timestamp_measured - self.last_timestep).to_sec()
            self.last_timestep = timestamp_measured
            self.ekf_step(time_delta)
            for key, value in self.data.items():
                self.data[key] = np.nan

    def ekf_step(self, time_delta):
        """Run EKF step.

        Params
        ------
        time_delta : float
            Time in seconds since last update.

        """

        # model noise
        Q = 3*time_delta*np.eye(3)

        # measurment noise
        R = 0.1*np.eye(3)

        mx = self.ekf_state[0]
        my = self.ekf_state[1]
        mth = self.ekf_state[2]
        mubar = self.ekf_state # \
               # + np.array([[time_delta*vel*np.cos(mth)],
               #             [time_delta*vel*np.sin(mth)],
               #             [time_delta*phi[ii]]])

        mubar[2,0] = self.wrap(mubar[2,0])
        mbx = mubar[0,0]
        mby = mubar[1,0]
        mbth = mubar[2,0]

        # Jd = np.array([[1.,0.,-time_delta*vel*np.sin(mth)],
        #                [0., 1., time_delta*vel*np.cos(mth)],
        #                [0., 0., 1.]])
        Jd = np.eye(3)
        Jm = np.array([[-(self.features[0,0] - mbx)/np.linalg.norm(mubar[:2,:] - self.features[0:1,:].T),
                        -(self.features[0,1] - mby)/np.linalg.norm(mubar[:2,:] - self.features[0:1,:].T),
                        0.],
                       [-(self.features[1,0] - mbx)/np.linalg.norm(mubar[:2,:] - self.features[1:2,:].T),
                        -(self.features[1,1] - mby)/np.linalg.norm(mubar[:2,:] - self.features[1:2,:].T),
                        0.],
                       [-(self.features[2,0] - mbx)/np.linalg.norm(mubar[:2,:] - self.features[2:3,:].T),
                        -(self.features[2,1] - mby)/np.linalg.norm(mubar[:2,:] - self.features[2:3,:].T),
                        0.],
                       ])
        sigmabar = Jd.dot(self.sigma).dot(Jd.T) + Q
        Kt = sigmabar.dot(Jm.T).dot(np.linalg.inv(Jm.dot(sigmabar).dot(Jm.T)+R))
        ybar = np.array([[np.linalg.norm(mubar[:2,:] - self.features[0:1,:].T)],
                         [np.linalg.norm(mubar[:2,:] - self.features[1:2,:].T)],
                         [np.linalg.norm(mubar[:2,:] - self.features[2:3,:].T)],
                         ])
        yt = np.array([[self.data["turtlebot1"]],
                       [self.data["turtlebot2"]],
                       [self.data["turtlebot4"]],
                       ])

        mu = mubar + Kt.dot(yt - ybar)
        mu[2,0] = self.wrap(mu[2,0])
        self.sigma = (np.eye(3) - Kt.dot(Jm)).dot(sigmabar)

        self.ekf_state = mu


    def publish_message(self):

        msg = Odometry()
        msg.header.stamp = self.last_timestep
        msg.child_frame_id = self.name + "_tf/odom_ekf"
        msg.pose.pose.position.x = self.ekf_state[0,0]
        msg.pose.pose.position.y = self.ekf_state[1,0]
        msg.pose.pose.position.z = 1.

        quaternion = tf.transformations.quaternion_from_euler(0.,0.,0.)
        msg.pose.pose.orientation.x = quaternion[0]
        msg.pose.pose.orientation.y = quaternion[1]
        msg.pose.pose.orientation.z = quaternion[2]
        msg.pose.pose.orientation.w = quaternion[3]

        self.odom_pub.publish(msg)

    def get_uwb_source(self, topic_name):
        """Get source of uwb information.

        Parameters
        ----------
        topic_name : string
            Current full topic name

        Returns
        -------
        uwb_source : string
            Source of UWB ranging meausrement.

        """
        name_list = topic_name.split("/")[-1]
        uwb_identifiers = sorted(name_list.split("_"))
        uwb_source = [x for x in uwb_identifiers if x != self.name][0]

        return uwb_source

    def wrap(self, angle):
        """Wrap angle [-pi, pi).

        """
        while angle >= np.pi:
            angle -= 2.*np.pi
        while angle < -np.pi:
            angle += 2.*np.pi
        return angle

    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """

        print("closing safely")

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    argParser.add_argument("__name", help="ros name")
    argParser.add_argument("__log", help="log name")
    args = argParser.parse_args()

    # Auto-get turtlebot name
    if args.name is None:
        # args.name = os.environ['USER']
        args.name = "turtlebot3"

    try:
        uwb_ekf = UWBEKF(args.name)
        rospy.on_shutdown(uwb_ekf.cleanup)
    except rospy.ROSInterruptException:
        pass
