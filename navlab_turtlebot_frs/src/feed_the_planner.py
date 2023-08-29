#!/usr/bin/env python

# Import libraries
import rospy
import numpy as np
import argparse
import csv

# Import message files
from navlab_turtlebot_frs.msg import ZonotopeMsg, ZonotopeMsgArray, pZonotopeMsg, pZonotopeMsgArray, FRSArray
from nav_msgs.msg import Path
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PoseStamped, Pose, Point, Polygon, Point32

# Import functions from other python files
from reachability import compute_FRS
from probzonotope import probZonotope
from zonotope import Zonotope

class feed_the_planner:
    def __init__(self):
        # General stuff
        self.n_bots = rospy.get_param("/n_bots")
        self.obs = ObstacleArrayMsg()
        self.generate_obstacles()
        
        # ROS stuff
        rospy.init_node(self.name + 'feed_the_planner', anonymous=True)
        self.pubs = []
        for i in range(self.n_bots):
            self.pubs.append(rospy.Publisher("/turtlebot"+ str(i+1) + "/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=10))
        self.rate = rospy.Rate(10)
     
    def generate_obstacles(self):
        obs1 = ObstacleMsg(polygon=Polygon(points=[Point32(x=1,y=1),Point32(x=1,y=5)]))
        self.obs.obstacles.append(obs1)
        obs2 = ObstacleMsg(polygon=Polygon(points=[Point32(x=1,y=-1),Point32(x=1,y=-5)]))
        self.obs.obstacles.append(obs2)
    
    def publish(self):
        """
        Publish the obstacles to different topics.
        """
        for i in range(self.n_bots):
            self.pubs[i].publish(self.obs)
    
    '''def traj_cb(self, global_plan, args):
        """
        Save received trajectories as 2xN numpy arrays, where N is the length of the trajectory.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        traj = np.zeros((2,len(global_plan.poses)))
        for t in range(len(global_plan.poses)):
            traj[0][t] = global_plan.poses[t].pose.position.x
            traj[1][t] = global_plan.poses[t].pose.position.y
        self.trajs[args] = traj
        self.update()
        self.received[args] = 1'''
    
    def run(self):
        while (not rospy.is_shutdown()):
            'rospy.Subscriber("/turtlebot" + str(i+1) + "/move_base/TebLocalPlannerROS/global_plan", Path, self.traj_cb, (i))'
            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    feeder = feed_the_planner()
    feeder.run()
