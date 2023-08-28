#!/usr/bin/env python

# Import libraries
import rospy
import numpy as np
import argparse

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
        # I make four lists even though there may be fewer bots, other lists might just stay empty
        self.frss = [[],[],[],[]]
        self.trajs = [[],[],[],[]]
        self.received = np.zeros((4,))
        
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
                zonomsg.generators = np.hstack((zono.c, zono.G))
                first.zonotopes.append(zonomsg)
            representation.total = first
            
            # Second
            second = pZonotopeMsgArray()
            for zono in self.frss[i][1]:
                zonomsg = pZonotopeMsg()
                zonomsg.dim = 2
                zonomsg.num_gen = 2
                zonomsg.generators = np.hstack((zono.c, zono.cov))
                second.pzonotopes.append(zonomsg)
            representation.trajbased = second
            
            # Third
            third = pZonotope()
            third.dim = 2
            third.num_gen = self.frss[i][2].n_gen
            third.generators = np.hstack((self.frss[i][2].c, self.frss[i][2].cov))
            representation.faultbased = third
            
            # Publish
            self.frs_pubs[i].publish(representation)
            
            # Log what you published
            with open('/home/izzie/catkin_ws/src/navlab_turtlebot/navlab_turtlebot_frs/data/happyfrs.txt', 'w') as f:
                f.write(str(third))
    
    def traj_cb(self, global_plan, args):
        """
        Save received trajectories as 2xN numpy arrays, where N is the length of the trajectory.
        args is a tuple with one item, the integer number of the agent this plan is for
        """
        traj = np.zeros((2,len(global_plan.poses)))
        for t in range(len(global_plan.poses)):
            traj[0][t] = global_plan.poses[t].pose.position.x
            traj[1][t] = global_plan.poses[t].pose.position.y
        print(args)
        self.trajs[args[0]].append(traj)
        self.received[args[0]] = 1
        self.update()
        
    def update(self):
        """
        Three FRSs are generated for each agent.
        First: a standard FRS made of zonotopes
        Second: an FRS made of p-zonotopes along the trajectory
        Third: an FRS made of p-zonotopes representing the fault case
        """
        for i in range(self.n_bots):
            # Generate the first FRS (normal)
            frss[i].append(compute_FRS(init_p, N=1200))

            # Generate the second FRS (pzonos)
            frss[i].append(compute_FRS(init_p, traj=self.trajs[i], N=1200))

            # Generate the third FRS (fault)
            frss[i].append(probZonotope(np.vstack((p_0s[:,i].reshape((2,1)),np.zeros((2,1)))),np.zeros((4,2)), \
                                            np.vstack((np.eye(2), np.zeros((2,2))))))
    
    def run(self):
        while (not rospy.is_shutdown()):
            self.t_sim = rospy.get_time() - self.start
            # If we haven't received the trajectories yet
            if np.sum(self.received)!=self.n_bots:
                # Subscribe to the global plans for every other agent
                for i in range(self.n_bots):
                    if i!=int(self.name[-1]):
                        print("/turtlebot" + str(i+1) + "/move_base/TebLocalPlannerROS/global_plan")
                        rospy.Subscriber("/turtlebot" + str(i+1) + "/move_base/TebLocalPlannerROS/global_plan", Path, self.traj_cb, (i))
            self.publish_frs()
            self.rate.sleep()

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args, unknown = argParser.parse_known_args()
    
    frs_gen = frs_generator(args.name)
    frs_gen.run()
