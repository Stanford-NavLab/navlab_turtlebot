#!/usr/bin/env python

# Import libraries
import rospy
import numpy as np
import argparse
import csv

# Import message files
from navlab_turtlebot_frs.msg import ZonotopeMsg, ZonotopeMsgArray, pZonotopeMsg, pZonotopeMsgArray, FRSArray
from nav_msgs.msg import Path
from geometry_msgs.msg import PoseStamped, Pose, Point

# Import functions from other python files
from reachability import compute_FRS
from probzonotope import probZonotope
from zonotope import Zonotope

class frs_generator:
    def __init__(self, name=''):
        # General stuff
        self.name = name
        self.n_bots = rospy.get_param("/n_bots")
        self.time = int(120/.3)
        # I make four lists even though there may be fewer bots, other lists might just stay empty
        self.frss = [[],[],[],[]]
        self.trajs = [[],[],[],[]]
        self.received = np.zeros((4,))
        self.goals = []
        
        # ROS stuff
        rospy.init_node(self.name + 'frs_generator', anonymous=True)
        self.frs_pubs = []
        for i in range(self.n_bots):
            self.frs_pubs.append(rospy.Publisher(self.name + "/" + str(i) + '/frs', FRSArray, queue_size=10))
        self.rate = rospy.Rate(10)
        self.start = rospy.get_time()
        self.t_sim = 0
        
    def publish_frs(self):
        """
        Publish all FRSs to different topics.
        """
        for i in range(self.n_bots):
            # Don't publish if we haven't received a trajectory yet
            if self.received[i] == 0:
                break
            
            representation = FRSArray()
            # 50% chance of random communications loss every second (given one of the other two didn't happen)
            # 50% chance that the comms are broken but it's still moving
            # Find the chance that it died at the beginning
            # Starts at ~0% chance
            # Ends at ~50% chance
            representation.weight = 1-.5**(self.t_sim)-.5
            
            # First
            first = ZonotopeMsgArray()
            for zono in self.frss[i][0]:
                zonomsg = ZonotopeMsg()
                zonomsg.dim = 2
                zonomsg.num_gen = zono.n_gen
                zonomsg.generators = np.vstack((zono.c.T, zono.G.T))
                first.zonotopes.append(zonomsg)
            representation.total = first
            
            rows = []
            for t, zono in enumerate(first.zonotopes):
                rows.append([self.t_sim+t*.1] + list(zono.generators.flatten()))
            with open('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrsfirst.csv', 'w') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
            
            # Second
            second = pZonotopeMsgArray()
            for zono in self.frss[i][1]:
                zonomsg = pZonotopeMsg()
                zonomsg.dim = 2
                zonomsg.num_gen = 2
                zonomsg.generators = np.vstack((zono.c.T, zono.G.T, zono.cov.T))
                second.pzonotopes.append(zonomsg)
            representation.trajbased = second
            
            rows = []
            for t, zono in enumerate(second.pzonotopes):
                rows.append([self.t_sim+t*.1] + list(zono.generators.flatten()))
            with open('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrssecond.csv', 'w') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
            
            # Third
            third = pZonotopeMsg()
            third.dim = 2
            third.num_gen = self.frss[i][2].n_gen
            third.generators = np.vstack((self.frss[i][2].c.T, self.frss[i][2].G.T, self.frss[i][2].cov.T))
            representation.faultbased = third
            
            rows = [[self.t_sim] + list(third.generators.flatten())]
            with open('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrsthird.csv', 'w') as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
            
            # Publish
            self.frs_pubs[i].publish(representation)
    
    def traj_cb(self, plan, args):
        """
        Save received trajectories as 2xN numpy arrays, where N is the length of the trajectory.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        # If this hasn't been done for this bot before...
        if self.received[args]==0:
            # Just once get goals. Do this here so it happens after params are set but pretty much immediately
            if args==1:
                for i in range(self.n_bots):
                    self.goals.append(np.array([rospy.get_param("/turtlebot"+str(i+1)+"/goal_x"),\
                                                rospy.get_param("/turtlebot"+str(i+1)+"/goal_y")]))

            self.received[args] = 1
            traj = np.zeros((2,len(plan.poses)))
            for t in range(len(plan.poses)):
                traj[0][t] = plan.poses[t].pose.position.x
                traj[1][t] = plan.poses[t].pose.position.y
            self.trajs[args] = traj
            self.update(args)
        
    def update(self, args):
        """
        Update the agent you just got the plan for.
        Three FRSs are generated for each agent.
        First: a standard FRS made of zonotopes
        Second: an FRS made of p-zonotopes along the trajectory
        Third: an FRS made of p-zonotopes representing the fault case
        """
        # Get the initial position
        init_p = self.trajs[args][:,0]

        # Generate the first FRS (normal)
        self.frss[args].append(compute_FRS(init_p, N=self.time))

        # Generate the second FRS (pzonos)
        self.frss[args].append(compute_FRS(init_p, traj=self.trajs[args], N=self.time, goal=self.goals[args], args=args))
        
        # Save calibration data if this is turtlebot2 and it's time to calibrate
        traj = self.trajs[args]
        if args == 1 and rospy.get_param("/sim_or_cal")=="cal":
            # Make sure rows are all same size
            if len(traj[0]) < 120/.3:
                rows = [np.hstack((traj[0],np.zeros((int(120/.3)-len(traj[0]),)))), \
                        np.hstack((traj[1],np.zeros((int(120/.3)-len(traj[0]),))))]
                with open("/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/caltraj.csv","a") as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerows(rows)
                with open("/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calsplit.txt","a") as f:
                    f.write(str(rospy.get_param("/split"))+"\n")

        # Generate the third FRS (fault)
        self.frss[args].append(probZonotope(np.vstack((init_p.reshape((2,1)),np.zeros((2,1)))), \
                                         np.vstack((np.eye(2)*.178/2, np.zeros((2,2)))), \
                                         np.vstack((np.eye(2), np.zeros((2,2))))))
    
    def run(self):
        while (not rospy.is_shutdown()):
            self.t_sim = rospy.get_time() - self.start
            # Subscribe to the global plans for every other agent
            for i in range(self.n_bots):
                # If this isn't the same as the main bot
                if i!=int(self.name[-1])-1:
                    rospy.Subscriber("/turtlebot" + str(i+1) + "/move_base/TebLocalPlannerROS/local_plan", Path, self.traj_cb, (i))
            self.publish_frs()
            self.rate.sleep()

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args, unknown = argParser.parse_known_args()
    
    frs_gen = frs_generator(args.name)
    frs_gen.run()
