#!/usr/bin/env python3
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

class GoalPlanner():
    """Sends goals to turtlebots.

    """
    def __init__(self):

        # Max distance threshold
        # next goal is published after all robots get within this amount
        # of goal location in meters
        self.goal_threshold = 0.5   # [m]

        # keep track of all odom subscribers
        self.odom_subscribers = set()
        self.odom_cache = {}

        # keep track of all goal publishers
        self.goal_publishers = {}

        # goal in [x,y,yaw]
        self.goals = [[ 2., 2.,0.],
                      [ 2.,-2.,3*np.pi/2.],
                      [-2.,-2.,np.pi],
                      [-2., 2.,np.pi/2.]]

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

        # Initialize ROS node.
        rospy.init_node('goal_planner', anonymous=False)
        rospy.on_shutdown(self.cleanup)
        rate = rospy.Rate(1) # 1Hz

        while not rospy.is_shutdown():

            all_topics = rospy.get_published_topics()

            # update odom topics
            odom_topics = {x[0] for x in all_topics if "/vrpn_client_node" in x[0]}
            new_odom_topics = odom_topics - self.odom_subscribers
            if len(new_odom_topics) > 0:
                self.subscribe_to_all(new_odom_topics)

            self.publish_goals()
            rate.sleep()

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

            rospy.Subscriber(topic_name, PoseStamped,
                             callback = self.odom_callback,
                             callback_args = turtlebot_id)
            self.odom_subscribers.add(topic_name)

            publish_name = "/turtlebot"+ turtlebot_id + "/move_base_simple/goal"
            self.goal_publishers[turtlebot_id] = rospy.Publisher(publish_name,
                                                        PoseStamped,
                                                        queue_size = 10)

            self.goal_distances[turtlebot_id] = np.inf

    def odom_callback(self, data_measured, turtlebot_id):
        """Callback function when new /odom messages are published.

        Parameters
        ----------
        data_measured : nav_msgs/Odometry
            Data from the /odom topic that was published.
        turtlebot_id : string
            Turtlebot number for which this callback function was called.

        """

        robot_x = data_measured.pose.position.x
        robot_y = data_measured.pose.position.y
        with self.lock:
            robot_goal_idx = self.current_goal_idx[turtlebot_id]
            goal_x, goal_y = self.goals[robot_goal_idx][:2]

            goal_distance = np.linalg.norm([goal_x-robot_x,goal_y-robot_y])

            self.goal_distances[turtlebot_id] = goal_distance
            distances = np.array(list(self.goal_distances.values()))
            if np.all(distances < self.goal_threshold):
                self.current_goal_idx = {k:(g+1)%len(self.goals) for k,g in self.current_goal_idx.items()}
                self.publish_goals()

    def publish_goals(self):
        """Publish new goal locations.

        """

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

        turtlebot_name = topic_name.split("/")[1]

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
