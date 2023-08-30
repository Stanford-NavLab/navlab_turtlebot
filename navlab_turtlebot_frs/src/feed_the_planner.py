#!/usr/bin/env python

# Import libraries
import rospy
import numpy as np
import pandas as pd
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
        self.stay_away = []
        for i in range(self.n_bots):
            self.stay_away.append(np.array([rospy.get_param("/turtlebot"+str(i+1)+"/start_x"), \
                                            rospy.get_param("/turtlebot"+str(i+1)+"/start_y")]))
            self.stay_away.append(np.array([rospy.get_param("/turtlebot"+str(i+1)+"/goal_x"),\
                                            rospy.get_param("/turtlebot"+str(i+1)+"/goal_y")]))
        self.n_obs = 3
        self.obs = ObstacleArrayMsg(header=Header(), obstacles=[])
        self.generate_obstacles()
        self.curr_locs = [None,None,None,None]
        if rospy.get_param("/sim_or_cal")=='cal':
            self.saved_odom = None
        self.vis_trck = np.zeros((self.n_obs,self.n_bots))
        self.vis_obs = [ObstacleArrayMsg(), ObstacleArrayMsg(header=Header(), obstacles=[]), \
                        ObstacleArrayMsg(header=Header(), obstacles=[]), ObstacleArrayMsg(header=Header(), obstacles=[])]
        self.vis_rad = 2 # m
        
        # ROS stuff
        rospy.init_node('feed_the_planner', anonymous=True)
        self.pubs = []
        for i in range(self.n_bots):
            self.pubs.append(rospy.Publisher("/turtlebot"+ str(i+1) + "/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=10))
        self.rate = rospy.Rate(10)
        self.last = rospy.get_time()
     
    def generate_obstacles(self):
        """
        Make some random obstacles.
        """
        rows = [[]]
        for i in range(self.n_obs):
            viable = False
            while not viable:
                # Random locations are in 5x5 square off-center from origin
                loc = np.random.rand(2)*np.array([-5, 5]) - np.array([0, 2.5])
                obstacle = ObstacleMsg(polygon=Polygon(points=[Point32(x=loc[0],y=loc[1])]),radius=1)
                viable = True
                for pt in self.stay_away:
                    # Generate new obstacles until they aren't intersecting the starting or goal positions
                    viable = viable and not self.check_range(obstacle,pt,.178/2+1)
            self.obs.obstacles.append(obstacle)
            rows[0].append(loc[0])
            rows[0].append(loc[1])
        with open("/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/simdeets.csv","a") as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(rows)
    
    def update(self,bot):
        """
        Update the list of obstacles that each bot can see.
        """
        for i, item in enumerate(self.obs.obstacles):
            if self.vis_trck[i][bot] == 0 and self.check_range(item, self.curr_locs[bot], self.vis_rad):
                self.vis_obs[bot].obstacles.append(item)
                    
    def check_range(self, obstacle, bot_loc, rng):
        """
        Check if the bot is close enough to the obstacle to see it or hit it.
        """
        obs_loc = np.array([obstacle.polygon.points[0].x, obstacle.polygon.points[0].y])
        return np.sum((obs_loc - bot_loc)**2)**.5 <= rng
    
    def publish(self):
        """
        Publish the obstacles to different topics.
        """
        for i in range(self.n_bots):
            self.pubs[i].publish(self.vis_obs[i])
    
    def odom_cb(self, odom, args):
        """
        Save the current positions of the turtlebots.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        self.curr_locs[args] = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        self.update(args)
        # Save calibration data if it has been .1 seconds
        if rospy.get_time()-self.last>=.1:
            print(np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(args)+'.csv', delimiter=','))
            self.last = rospy.get_time()
            #print(odom.pose.pose.position.x, odom.pose.pose.position.y)
            if not self.saved_odom is None:
                self.saved_odom = np.hstack((self.saved_odom.copy(), self.curr_locs[args].reshape((2,1)).copy()))
            else:
                self.saved_odom = self.curr_locs[args].reshape((2,1)).copy()
            if self.saved_odom.shape[1] < 120/.1:
                if len(self.saved_odom[0]) != 1:
                    df = pd.read_csv('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(args)+'.csv', header=None)
                    df = df.drop(df.index[-1])
                    df = df.drop(df.index[-1])
                    new_data = np.hstack((self.saved_odom,np.zeros((2,int(120/.1)-self.saved_odom.shape[1]))))
                    df.loc[2] = new_data[0].tolist()
                    df.loc[3] = new_data[1].tolist()
                    df.to_csv('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(args)+'.csv', index=False, header=False)
                else:
                    rows = np.hstack((self.saved_odom,np.zeros((2,int(120/.1)-self.saved_odom.shape[1]))))
                    with open('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(args)+'.csv',"a") as csvfile:
                        csvwriter = csv.writer(csvfile)
                        csvwriter.writerows(rows)
            print("\n")
            print(np.loadtxt('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(args)+'.csv', delimiter=','))
            print("\n\n\n")
    
    def frs_cb(self, frs, args):
        pass
    
    def run(self):
        while (not rospy.is_shutdown()):
            for i in range(self.n_bots):
                rospy.Subscriber("/turtlebot" + str(i+1) + "/odom", Odometry, self.odom_cb, (i))
                rospy.Subscriber("/turtlebot1/" + str(i+1) + '/frs', FRSArray, self.frs_cb, (i))
            self.publish()
            self.rate.sleep()

if __name__ == '__main__':
    feeder = feed_the_planner()
    feeder.run()
