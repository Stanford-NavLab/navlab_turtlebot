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
        # ROS stuff
        self.n_bots = rospy.get_param("/n_bots")
        rospy.init_node('feed_the_planner', anonymous=True)
        rospy.on_shutdown(self.write_log)
        self.pubs = []
        for i in range(self.n_bots):
            self.pubs.append(rospy.Publisher("/turtlebot"+ str(i+1) + "/move_base/TebLocalPlannerROS/obstacles", ObstacleArrayMsg, queue_size=1))
        self.rate = rospy.Rate(10)
        self.last = [rospy.get_time(),rospy.get_time(),rospy.get_time(),rospy.get_time()]
        self.stay_away = []
        for i in range(self.n_bots):
            self.stay_away.append(np.array([rospy.get_param("/turtlebot"+str(i+1)+"/start_x"), \
                                            rospy.get_param("/turtlebot"+str(i+1)+"/start_y")]))
            self.stay_away.append(np.array([rospy.get_param("/turtlebot"+str(i+1)+"/goal_x"),\
                                            rospy.get_param("/turtlebot"+str(i+1)+"/goal_y")]))

        # Obstacle generation
        self.n_obs = 3
        self.obs = ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map"))
        self.vis_obs_tracker = np.zeros((self.n_obs,self.n_bots))
        self.vis_trck = np.zeros((self.n_obs,self.n_bots))
        self.vis_obs = [ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), \
                        ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map"))]
        self.vis_rad = 2 # m
        self.generate_obstacles()

        self.curr_locs = [None,None,None,None]
        if rospy.get_param("/sim_or_cal")=='cal':
            self.saved_odom = [None, None, None, None]
        self.ag = [ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), \
                        ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map")), ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map"))]
        self.logged = False

        self.start = rospy.get_time()

        self.saved_odom = [None, None, None, None]

    def generate_obstacles(self):
        """
        Make some random obstacles.
        """
        rows = [[]]
        print(self.stay_away)
        for i in range(self.n_obs):
            print("Generating obstacle #"+str(i))
            viable = False
            while not viable:
                # Random locations are in 5x5 square off-center from origin
                loc = np.random.rand(2)*np.array([-5, 5]) - np.array([0, 2.5])
                obstacle = ObstacleMsg(polygon=Polygon(points=[Point32(x=loc[0],y=loc[1])]),id=i,radius=.25)
                #obstacle = ObstacleMsg(polygon=Polygon(points=[Point32(x=loc[0]-.25,y=loc[1]-.25), \
                #                                               Point32(x=loc[0]-.25,y=loc[1]+.25), \
                #                                               Point32(x=loc[0]+.25,y=loc[1]-.25), \
                #                                               Point32(x=loc[0]+.25,y=loc[1]+.25)]),id=i)
                viable = None
                for pt in self.stay_away:
                    # Generate new obstacles until they aren't intersecting the starting or goal positions
                    if viable is None:
                        viable = not self.check_range(obstacle,pt,.105+.25)
                    else:
                        viable = viable and not self.check_range(obstacle,pt,.105+.25)
                    print(obstacle.polygon.points[0].x, obstacle.polygon.points[0].y)
                    print(pt)
                    print(viable)
            self.obs.obstacles.append(obstacle)
            rows[0].append(loc[0])
            rows[0].append(loc[1])
        # Make sure csv lines up correctly
        for i in range(self.n_obs*2,6):
            rows[0].append(1.)
        print("obstacles")
        print(rows)
        print("\n")
        with open("/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/simdeets.csv","a") as csvfile:
            csvwriter = csv.writer(csvfile)
            csvwriter.writerows(rows)

    def update(self,bot):
        """
        Update the list of obstacles that each bot can see.
        """
        for i, item in enumerate(self.obs.obstacles):
            if self.vis_trck[i][bot] == 0 and self.check_range(item, self.curr_locs[bot], self.vis_rad):
                self.vis_obs[bot].obstacles.append(item)
                # Record that you already added this obstacle for this bot
                self.vis_trck[i][bot] = 1

    def check_range(self, obstacle, bot_loc, rng):
        """
        Check if the bot is close enough to the obstacle to see it or hit it.
        True = in range
        """
        obs_loc = np.array([obstacle.polygon.points[0].x, obstacle.polygon.points[0].y])
        if rng == .105+.25:
            print(np.sum((obs_loc - bot_loc)**2)**.5)
        return np.sum((obs_loc - bot_loc)**2)**.5 <= rng

    def publish(self):
        """
        Publish the obstacles to different topics.
        """
        for i in range(self.n_bots):
            if i==0:
                pub_data = ObstacleArrayMsg(header=Header(stamp=rospy.Time.now(),frame_id="map"))
                pub_data.obstacles = list(self.vis_obs[i].obstacles)
                for bot in range(1, self.n_bots):
                    pub_data.obstacles = list(pub_data.obstacles) + list(self.ag[bot].obstacles)
                #pub_data.obstacles.append(ObstacleMsg(polygon=Polygon(points=[Point32(x=rospy.get_param("/turtlebot1/start_x"),y=rospy.get_param("/turtlebot1/start_y"))]),id=i,radius=.25))
                self.pubs[i].publish(pub_data)
            else:
                self.pubs[i].publish(self.vis_obs[i])
                #pub_data.obstacles.append(ObstacleMsg(polygon=Polygon(points=[Point32(x=rospy.get_param("/turtlebot1/start_x"),y=rospy.get_param("/turtlebot1/start_y"))]),id=i,radius=.25))
        """for i in range(self.n_bots):
            if pub_data is None:
                self.pubs[i].publish(self.vis_obs[i])
            else:
                self.pubs[i].publish(combined_data)"""

    def odom_cb(self, odom, args):
        """
        Save the current positions of the turtlebots.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        self.curr_locs[args] = np.array([odom.pose.pose.position.x, odom.pose.pose.position.y])
        self.update(args)
        # Save calibration data if it has been .1 seconds
        if rospy.get_time()-self.last[args]>=.1:
            self.last[args] = rospy.get_time()
            if self.saved_odom[args] is None:
                self.saved_odom[args] = self.curr_locs[args].reshape((2,1)).copy()
            else:
                self.saved_odom[args] = np.hstack((self.saved_odom[args].copy(), self.curr_locs[args].reshape((2,1)).copy()))

    def frs_cb(self, frs, args):
        #print("updating",args)
        # Reset the agent obstacles
        obslist = []

        newstuff = True
        reachability = True
        p = .1

        if newstuff:
            # Get the relevant data out of the frs
            ind = int(self.t_sim/.3)
            if len(frs.trajbased.pzonotopes) <= ind:
                ind = -1
            second = frs.trajbased.pzonotopes[ind]
            third = frs.faultbased

            # Make an obstacle out of the no-fault case
            first_obs = probZonotope(np.array(second.generators[:2]).reshape((2,1)),np.array(second.generators[2:10]).reshape((4,2)).T,np.array(second.generators[10:]).reshape((2,2)).T)
            # If p=.5 and weight=0, then zonotope generators should all be 0
            # If p=.5 and weight=1, then it's just like the other case doesn't exist
            first_obs = first_obs.get_zono(p*(1-frs.weight))
            first_v = first_obs.vertices()
            first_obstacle = ObstacleMsg(polygon=Polygon(),id=3+args*2)
            for vert in first_v.T:
                first_obstacle.polygon.points.append(Point32(x=vert[0],y=vert[1]))
            obslist.append(first_obstacle)
            # Make an obstacle out of the fault case
            second_obs = probZonotope(np.array(third.generators[:2]).reshape((2,1)),np.array(third.generators[2:6]).reshape((2,2)).T,np.array(third.generators[6:]).reshape((2,2)).T)
            second_obs = second_obs.get_zono(p*frs.weight)
            second_v = second_obs.vertices()
            second_obstacle = ObstacleMsg(polygon=Polygon(),id=3+args*2+1)
            for vert in second_v.T:
                second_obstacle.polygon.points.append(Point32(x=vert[0],y=vert[1]))
            obslist.append(second_obstacle)

            fourth = None
            if len(frs.trajbased.pzonotopes) > ind+50:
                fourth = frs.trajbased.pzonotopes[ind+50]
            else:
                fourth = frs.trajbased.pzonotopes[-1]
            if not fourth is None:
                third_obs = probZonotope(np.array(fourth.generators[:2]).reshape((2,1)),np.array(fourth.generators[2:10]).reshape((4,2)).T,np.array(fourth.generators[10:]).reshape((2,2)).T)
                # If p=.5 and weight=0, then zonotope generators should all be 0
                # If p=.5 and weight=1, then it's just like the other case doesn't exist
                third_obs = third_obs.get_zono(p*(1-frs.weight))
                third_v = third_obs.vertices()
                third_obstacle = ObstacleMsg(polygon=Polygon(),id=3+args*2)
                for vert in third_v.T:
                    third_obstacle.polygon.points.append(Point32(x=vert[0],y=vert[1]))
                obslist.append(third_obstacle)
        elif reachability:
            # Get the relevant data out of the frs
            first = frs.total.zonotopes[int(self.t_sim/.3)]
            obs = Zonotope(np.array(first.generators[:2]).reshape((2,1)),np.array(np.array(first.generators[2:])).reshape((4,2)).T)
            v = obs.vertices()
            obstacle = ObstacleMsg(polygon=Polygon(),id=3+args)
            for vert in v.T:
                obstacle.polygon.points.append(Point32(x=vert[0],y=vert[1]))
            obslist.append(obstacle)

        self.ag[args].obstacles = obslist

    def run(self):
        while (not rospy.is_shutdown()):
            if self.start == 0:
                self.start = rospy.get_time()
            self.t_sim = rospy.get_time() - self.start
            for i in range(self.n_bots):
                rospy.Subscriber("/turtlebot" + str(i+1) + "/odom", Odometry, self.odom_cb, (i))
                rospy.Subscriber("/turtlebot1/" + str(i) + '/frs', FRSArray, self.frs_cb, (i))

            self.publish()
            self.rate.sleep()

    def write_log(self):
        print("writing to log now!")
        for i in range(self.n_bots):
            # Make all rows the same length
            if self.saved_odom[i].shape[1] >= 120/.1:
                rows = self.saved_odom[i][:,:int(120/.1)]
            else:
                rows = np.hstack((self.saved_odom[i],np.zeros((2,int(120/.1)-self.saved_odom[i].shape[1]))))
            # Log!
            with open('/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calodom'+str(i)+'.csv',"a") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
        print("finished writing!")

if __name__ == '__main__':
    try:
        feeder = feed_the_planner()
        feeder.run()
        rospy.on_shutdown(feeder.write_log)
    except rospy.ROSInterruptException:
        pass
