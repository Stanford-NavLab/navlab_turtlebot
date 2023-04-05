#!/usr/bin/env python
"""Goal planner for Turtlebots.

Publishes new goals after checking all turtlebots have reached their
previous goal.

Looks for odometry on topics called: "/turtlebot*/odom"

Publishes goal on topics called: "/turtlebot*/move_base_simple/goal"

"""

__authors__ = "D. Knowles"
__date__ = "28 Feb 2023"

from threading import Lock

import tf
import rospy
import numpy as np
import message_filters
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped
from diagnostic_msgs.msg import DiagnosticStatus, KeyValue

class GoalPlanner():
    """Sends goals to turtlebots.

    """
    def __init__(self):

        # Initialize ROS node.
        rospy.init_node('goal_planner', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        rate1 = rospy.Rate(1) # 1Hz sleep rate
        rate10 = rospy.Rate(0.1) # 0.1Hz sleep rate

        self.diagnostic_pub = rospy.Publisher("goal_planner/diagnostics",
                                              DiagnosticStatus,
                                              queue_size = 10)

        # Max distance threshold
        # next goal is published after all robots get within this amount
        # of goal location in meters
        self.goal_threshold = 0.25   # [m]

        # keep track of all odom subscribers
        self.odom_subscribers = {}
        self.odom_subscribed_topics = {}
        self.odom_cache = {}
        self.last_odom = {}

        # if no new odom messages received after this timeout, the robot
        # is assumed to have left the area and its goal distance will no
        # longer be checked in seconds
        self.goal_timeout_secs = 2.0

        # keep track of all goal publishers
        self.goal_publishers = {}

        # goal in [x,y,yaw]
        self.goals = []
        goal_params = ["goal_x","goal_y","goal_yaw"]

        while True:
            for tt in range(21): # max expected turtlebot count
                robot_name = "turtlebot" + str(tt)
                if all([rospy.has_param('/' + robot_name + '/' + x) for x in goal_params]):
                    self.goals.append([rospy.get_param('/' + robot_name + '/' + "goal_x"),
                                       rospy.get_param('/' + robot_name + '/' + "goal_y"),
                                       rospy.get_param('/' + robot_name + '/' + "goal_yaw"),
                                      ])
            if len(self.goals) > 0:
                break
            else:
                rate.sleep()

        # lock object for locking synchronizer variables
        # locked to prevent race conditions during parallelized
        # callbacks and main loop updated topic subscribers
        self.lock = Lock()

        # set initial goal positions
        # key is the turtlebot identifier (whatever follows turtlebot)
        # value is the index of their starting goal position
        with self.lock:
            self.current_goal_idx = {str(i+1):i for i in range(len(self.goals))}
            self.goal_distances = {}

        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()

            # update odom topics
            odom_topics = {x[0] for x in all_topics if "/odom" in x[0]}
            new_odom_topics = odom_topics - set(self.odom_subscribed_topics.values())
            if len(new_odom_topics) > 0:
                self.subscribe_to_all(new_odom_topics)

            self.publish_goals()
            rate10.sleep()


    def subscribe_to_all(self, new_topics):
        """Subscribes to all new topics and adds to dictionary.

        Parameters
        ----------
        new_topics : list
            List of topic names for which to subscribe

        """

        for topic_name in new_topics:
            # Subscribe to topic and set callback function
            turtlebot_id = self.get_turtlebot_id(topic_name)

            sub = rospy.Subscriber(topic_name, Odometry,
                             callback = self.odom_callback,
                             callback_args = turtlebot_id)
            self.odom_subscribers[turtlebot_id] = sub
            self.odom_subscribed_topics[turtlebot_id] = topic_name

            publish_name = "/turtlebot"+ turtlebot_id + "/move_base_simple/goal"
            self.goal_publishers[turtlebot_id] = rospy.Publisher(publish_name,
                                                        PoseStamped,
                                                        queue_size = 10)

            self.goal_distances[turtlebot_id] = np.inf
            self.last_odom[turtlebot_id] = rospy.get_rostime()

    def odom_callback(self, data_measured, turtlebot_id):
        """Callback function when new /odom messages are published.

        Parameters
        ----------
        data_measured : nav_msgs/Odometry
            Data from the /odom topic that was published.
        turtlebot_id : string
            Turtlebot number for which this callback function was called.

        """

        robot_x = data_measured.pose.pose.position.x
        robot_y = data_measured.pose.pose.position.y
        with self.lock:
            self.last_odom[turtlebot_id] = data_measured.header.stamp
            robot_goal_idx = self.current_goal_idx[turtlebot_id]
            goal_x, goal_y = self.goals[robot_goal_idx][:2]

            goal_distance = np.linalg.norm([goal_x-robot_x,goal_y-robot_y])

            self.goal_distances[turtlebot_id] = goal_distance
            current_time = rospy.get_rostime() # in nsecs
            robot_names = list(self.goal_distances.keys())
            for key in robot_names:
                if key in self.last_odom and (current_time - self.last_odom[key]) > rospy.Duration(self.goal_timeout_secs):
                    if key in self.goal_distances: del self.goal_distances[key]
                    if key in self.goal_publishers: del self.goal_publishers[key]
                    if key in self.odom_subscribers:
                        self.odom_subscribers[key].unregister()
                        del self.odom_subscribers[key]
                    if key in self.odom_subscribed_topics: del self.odom_subscribed_topics[key]
                    del self.last_odom[key]
                    rospy.logwarn("removing goal tracking for turtlebot" + str(key))

            distances = list(self.goal_distances.values())
            distances = np.array([d for d in distances if not np.isinf(d)])
            if np.all(distances < self.goal_threshold):
                self.current_goal_idx = {k:(g+1)%len(self.goals) for k,g in self.current_goal_idx.items()}
                self.publish_goals()

    def publish_goals(self):
        """Publish new goal locations.

        """

        diagnostic_msg = DiagnosticStatus()
        diagnostic_values = []

        for id, publisher in self.goal_publishers.items():
            msg = PoseStamped()
            msg.header.frame_id = "map"
            msg.header.stamp = rospy.Time.now()
            msg.pose.position.x = self.goals[self.current_goal_idx[id]][0]
            msg.pose.position.y = self.goals[self.current_goal_idx[id]][1]
            quaternion = tf.transformations.quaternion_from_euler(0.,0.,self.goals[self.current_goal_idx[id]][2])
            msg.pose.orientation.x = quaternion[0]
            msg.pose.orientation.y = quaternion[1]
            msg.pose.orientation.z = quaternion[2]
            msg.pose.orientation.w = quaternion[3]
            publisher.publish(msg)

            key_value = KeyValue()
            key_value.key = id
            key_value.value = str(self.goal_distances[id])
            diagnostic_values.append(key_value)

        diagnostic_msg.values = diagnostic_values
        self.diagnostic_pub.publish(diagnostic_msg)



    def get_turtlebot_id(self, topic_name):
        """Get new topic name sorted alphnumerically

        Parameters
        ----------
        topic_name : string
            Current full topic name

        Returns
        -------
        turtlebot_id : string
            Turtlebot identification number
        """

        # remove initial slash if it exists
        if topic_name[0] == "/": topic_name = topic_name[1:]

        turtlebot_name = topic_name.split("/")[0]

        # use whatever follows "turtlebot" as the ID
        turtlebot_id = turtlebot_name[9:]

        return turtlebot_id

    def cleanup(self):
        """Gets called with Exceptions.

        You can handle any necessary file saving or cleanup here to
        prevent data loss.

        """

        print("closing safely")


if __name__ == '__main__':
    try:
        gp = GoalPlanner()
        rospy.on_shutdown(gp.cleanup)
    except rospy.ROSInterruptException:
        pass
