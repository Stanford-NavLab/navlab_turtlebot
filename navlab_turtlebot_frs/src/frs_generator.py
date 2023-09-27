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
        self.written = np.zeros((4,))
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

        This is not generalized for more than two robots yet.
        """
        for i in range(self.n_bots):
            # Don't publish if we haven't received a trajectory yet
            if self.received[1] == 0:
                break

            representation = FRSArray()
            # 50% chance of random communications loss every second (given one of the other two didn't happen)
            # 50% chance that the comms are broken but it's still moving
            # Find the chance that it died at the beginning
            # Starts at ~0% chance
            # Ends at ~50% chance
            #representation.weight = 1-.5**(self.t_sim)-.5

            # Setting probability of EMI given communication loss to .94, duration to 1s
            # Setting probability of occlusion given communication loss to .05, duration to 3s
            representation.weight = 1 - .94**(self.t_sim+1) - .05**(self.t_sim/3+1)

            # First
            first = ZonotopeMsgArray()
            for zono in self.frss[1][0]:
                zonomsg = ZonotopeMsg()
                zonomsg.dim = 2
                zonomsg.num_gen = zono.n_gen
                zonomsg.generators = np.hstack((zono.c[:2], zono.G[:2])).T.flatten().tolist()
                first.zonotopes.append(zonomsg)
            representation.total = first

            if self.written[1] == 0:
                rows = []
                for t, zono in enumerate(first.zonotopes):
                    rows.append([self.t_sim+t*.1] + list(zono.generators))
                rows.append(np.zeros((len(rows[-1]),)).tolist())
                with open('/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrsfirst.csv', 'a') as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerows(rows)

            # Second
            second = pZonotopeMsgArray()
            for zono in self.frss[1][1]:
                zonomsg = pZonotopeMsg()
                zonomsg.dim = 2
                zonomsg.num_gen = 2
                zonomsg.generators = np.hstack((zono.c[:2], zono.G[:2], zono.cov[:2,:2])).T.flatten().tolist()
                second.pzonotopes.append(zonomsg)
            representation.trajbased = second

            if self.written[1] == 0:
                rows = []
                for t, zono in enumerate(second.pzonotopes):
                    rows.append([self.t_sim+t*.1] + list(zono.generators))
                rows.append(np.zeros((len(rows[-1]),)).tolist())
                with open('/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrssecond.csv', 'a') as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerows(rows)

            # Third
            third = pZonotopeMsg()
            third.dim = 2
            third.num_gen = self.frss[1][2].n_gen
            third.generators = np.hstack((self.frss[1][2].c[:2], self.frss[1][2].G[:2], self.frss[1][2].cov[:2,:2])).T.flatten().tolist()
            representation.faultbased = third

            if self.written[1] == 0:
                rows = [[self.t_sim] + list(third.generators)]
                with open('/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrsthird.csv', 'a') as csvfile:
                    csvwriter = csv.writer(csvfile)
                    csvwriter.writerows(rows)
                self.written[1] = 1

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
        first = compute_FRS(init_p, N=self.time)
        self.frss[args].append(first[0])

        # Generate the second FRS (pzonos)
        second = compute_FRS(init_p, traj=self.trajs[args], N=self.time, goal=self.goals[args], args=args)
        self.frss[args].append(second[0])
        # Update trajectory with what was calculated in the FRS function
        self.trajs[args] = second[1]

        # Save calibration data if this is turtlebot2 and it's time to calibrate
        traj = self.trajs[args]
        if args == 1 and rospy.get_param("/sim_or_cal")=="cal":
            # Make sure rows are all same size
            if len(traj[0]) < 120/.3:
                rows = [np.hstack((traj[0],np.zeros((int(120/.3)-len(traj[0]),)))), \
                        np.hstack((traj[1],np.zeros((int(120/.3)-len(traj[0]),))))]
            else:
                rows = [traj[0,:int(120/.3)],traj[1,:int(120/.3)]]
            with open("/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/caltraj.csv","a") as csvfile:
                csvwriter = csv.writer(csvfile)
                csvwriter.writerows(rows)
            with open("/home/derek/jpl_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/calsplit.txt","a") as f:
                f.write(str(rospy.get_param("/split"))+"\n")

        # Generate the third FRS (fault)
        self.frss[args].append(probZonotope(np.vstack((init_p.reshape((2,1)),np.zeros((2,1)))), \
                                         np.vstack((np.eye(2)*.105, np.zeros((2,2)))), \
                                         np.vstack((np.eye(2), np.zeros((2,2))))))

        self.received[args] = 1

    def run(self):
        self.start = rospy.get_time()
        while (not rospy.is_shutdown()):
            if self.start == 0:
                self.start = rospy.get_time()
            self.t_sim = float(rospy.get_time()) - float(self.start)
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
