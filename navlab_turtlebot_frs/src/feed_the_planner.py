#!/usr/bin/env python

# Import libraries
import rospy
import numpy as np
import argparse
import csv

# Import message files
from navlab_turtlebot_frs.msg import ZonotopeMsg, ZonotopeMsgArray, pZonotopeMsg, pZonotopeMsgArray, FRSArray
from nav_msgs.msg import Path, Odometry
from costmap_converter.msg import ObstacleArrayMsg, ObstacleMsg
from geometry_msgs.msg import PoseStamped, Pose, Point, Polygon, Point32, PoseWithCovariance
from std_msgs.msg import Header

# Import functions from other python files
from reachability import compute_FRS
from probzonotope import probZonotope
from zonotope import Zonotope

class feed_the_planner:
    def __init__(self):
        # General stuff
        self.n_bots = rospy.get_param("/n_bots")
        self.n_obs = 3
        self.obs = ObstacleArrayMsg(header=Header(), obstacles=[])
        self.generate_obstacles()
        self.curr_locs = [None,None,None,None]
        self.vis_trck = np.zeros((self.n_obs,self.n_bots))
        self.vis_obs = [ObstacleArrayMsg(header=Header(), obstacles=[]), ObstacleArrayMsg(header=Header(), obstacles=[]), \
                        ObstacleArrayMsg(header=Header(), obstacles=[]), ObstacleArrayMsg(header=Header(), obstacles=[])]
        self.vis_rad = 2 # m
        
        # ROS stuff
        rospy.init_node('feed_the_planner', anonymous=True)
        self.pubs = []
        for i in range(self.n_bots):
            self.pubs.append(rospy.Publisher("/turtlebot"+ str(i+1) + "/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=10))
        self.rate = rospy.Rate(10)
     
    def generate_obstacles(self):
        for i in range(self.n_obs):
            obstacle = ObstacleMsg(header=Header(),polygon=Polygon(points=[Point32(x=1,y=1)]),radius=1)
            self.obs.obstacles.append(obstacle)
    
    def update(self):
        for i, item in enumerate(self.obs.obstacles):
            for bot in range(self.n_bots):
                if self.vis_trck[i][bot+1] == 0 and self.check_range(item, bot+1):
                    self.vis_obs[bot].obstacles.append(item)
                    
    def check_range(self, obstacle, bot):
        obs_loc = np.array([obstacle.polygon.points[0].x, obstacle.polygon.points[0].y])
        bot_loc = self.curr_locs[bot]
        return np.sum((obs_loc - bot_loc)**2)**.5 <= self.vis_rad
    
    def publish(self):
        """
        Publish the obstacles to different topics.
        """
        for i in range(self.n_bots):
            self.pubs[i].publish(self.vis_obs)
    
    def odom_cb(self, odom, args):
        """
        Save the current positions of the turtlebots.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        self.curr_locs[args] = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        self.update()
    
    def run(self):
        while (not rospy.is_shutdown()):
            for i in range(self.n_bots):
                rospy.Subscriber("/turtlebot" + str(i+1) + "/odom", Odometry, self.odom_cb, (i))
            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    feeder = feed_the_planner()
    feeder.run()
