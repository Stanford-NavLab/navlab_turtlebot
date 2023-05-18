#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray
from navlab_turtlebot_common.msg import NominalTrajectory
from navlab_turtlebot_common.msg import State
from navlab_turtlebot_obstacles.msg import ZonotopeMsg, ZonotopeMsgArray
from zonotope import Zonotope
from gazebo_msgs.msg import ModelStates
import argparse

class comms_node():
    def __init__(self,n):
        # Dubins planner
        self.n = int(n)
        self.trajs = [NominalTrajectory()]*self.n
        self.zonotopes = []
        self.down = False
        self.locs = np.zeros((self.n,2))

    def robot1_cb(self,data):
        self.trajs[0] = data

    def robot2_cb(self,data):
        self.trajs[1] = data

    def robot3_cb(self,data):
        self.trajs[2] = data

    def robot4_cb(self,data):
        self.trajs[3] = data

    def gazebo_cb(self,data):
        robot1 = data.name.index("turtlebot1")
        robot2 = data.name.index("turtlebot2")
        self.locs[0,0] = data.pose[robot1].position.x
        self.locs[0,1] = data.pose[robot1].position.y
        self.locs[1,0] = data.pose[robot2].position.x
        self.locs[1,1] = data.pose[robot2].position.y
        if self.n >= 3:
            robot3 = data.name.index("turtlebot3")
            self.locs[2,0] = data.pose[robot3].position.x
            self.locs[2,1] = data.pose[robot3].position.y
            if self.n >= 4:
                robot4 = data.name.index("turtlebot4")
                self.locs[3,0] = data.pose[robot4].position.x
                self.locs[3,1] = data.pose[robot4].position.y

    def zonotope_cb(self,data):
        self.zonotopes = data.zonotopes
        #print(self.zonotopes[0].generators)

    def check_occlusions(self,zero,one):
        """
        Checks if the path between robot zero and robot one is obstructed by a 2D obstacle.
        """
        r_m = (self.locs[one,1]-self.locs[zero,1])/(self.locs[one,0]-self.locs[zero,0])
        r_b = r_m*self.locs[one,0] + self.locs[one,1]
        for obs in self.zonotopes:
            c = np.array(obs.generators).reshape((obs.dim,obs.num_gen))[:,0].reshape((obs.dim,1))
            g = np.array(obs.generators).reshape((obs.dim,obs.num_gen))[:,1:]
            vertices = Zonotope(c[:2],g[:2,:-1]).vertices()
            for i in range(vertices.shape[1]-1):
                p1 = vertices[:2,i]
                p2 = vertices[:2,i+1]
                m = (p2-p1)[1]/(p2-p1)[0]
                b = m*p2[0]+p2[1]
                if abs(m-r_m)>.001: #Check the two line segments aren't parallel
                    b = m*p2[0]+p2[1]
                    if np.absolute(m) == np.inf:
                        x = p2[0]
                        y = r_m*x+r_b
                    elif np.absolute(r_m) == np.inf:
                        x = self.locs[zero,0]
                        y = m*x+b
                    else:
                        x = (r_b-b)/(m-r_m)
                        y = m*x+b
                    # If the point is on the line segments, the difference between x and bounds should have opposite signs to be in between
                    check1 = x-p1[0]
                    check2 = x-p2[0]
                    check3 = x-self.locs[zero,0]
                    check4 = x-self.locs[one,0]
                    if (abs(check1)/(check1)!=abs(check2)/(check2) and \
                       abs(check3)/(check3)!=abs(check4)/(check4)) or \
                       (np.absolute(m+r_m)==np.inf and \
                       abs(y-p1[1])/(y-p1[1])!=abs(y-p2[1])/(y-p2[1]) and \
                       abs(y-self.locs[zero,1])/(y-self.locs[zero,1])!=abs(y-self.locs[one,1])/(y-self.locs[one,1])):
                        #print(x,y)
                        #print(self.locs)
                        return False
                else: # Assumes the lines are roughly parallel
                    # Is the first point on the line between robots?
                    check1 = p1[0]-self.locs[zero,0]
                    check2 = p1[0]-self.locs[one,0]
                    # Is the second point on the line between robots?
                    check3 = p2[0]-self.locs[zero,0]
                    check4 = p2[0]-self.locs[one,0]
                    if abs(check1)/check1!=abs(check2)/check2 and \
                       abs(check3)/check3!=abs(check4)/check4 and \
                       abs(b-r_b)<.001: # Only return false if collinear
                        return False
        return True

    def comms(self):
        pub = rospy.Publisher('comms_list',Int16MultiArray,queue_size=10)
        rospy.init_node('comms',anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.Subscriber('/gazebo/model_states',ModelStates,self.gazebo_cb)
            rospy.Subscriber('/map/zonotopes',ZonotopeMsgArray,self.zonotope_cb)
            # Probability 1% of comms dropping each .1 sec, comms drop for occlusions
            randloss = np.random.choice([0,1],size=(self.n,self.n),p=[.99,.01])
            comms_group = np.array([np.inf]*self.n)
            group = 0
            comms_group[0] = 0
            old_group = np.copy(comms_group)
            # Until each robot has been assigned a group
            while np.sum(comms_group) == np.inf:
                # for each robot
                for i in range(self.n):
                    # if it is in the current group we are assigning robots to
                    if comms_group[i] == group:
                        # for each robot
                        for j in range(self.n):
                            # if it isn't the same robot and it doesn't already have a group
                            if not j==i and comms_group[j]==np.inf:
                                # If the line of sight is not blocked:
                                if self.check_occlusions(i,j) and randloss[i,j]==0:
                                    # Assign to the same comms group
                                    comms_group[j] = comms_group[i]
                # If the last iteration didn't change anything,
                if np.array_equal(comms_group, old_group):
                    # Add a new comms group
                    group += 1
                    # If we are at one robot left, just stop
                    if np.sum(np.where(comms_group==np.inf,1,0))==1:
                        comms_group = np.where(comms_group==np.inf,group,comms_group)
                old_group = np.copy(comms_group)
            #if self.check_occlusions():
                #no_comms = np.random.choice([0,1],size=(1,2),p=[.99,.01])
            #else:
                #no_comms = np.array([1,1]).reshape((1,2))
            # If comms are working, robot1 gets the info from robot2, and vice versa
            #comms_list = Int16MultiArray(data=list(no_comms[0]))
            comms_list = Int16MultiArray(data=list(comms_group))
            pub.publish(comms_list)
            if np.sum(comms_group)==0:
                #pub1.publish(trajs[1])
                #pub2.publish(trajs[0])
                if self.down:
                    self.down = False
                    print("Comms back!")
            else:
                if not self.down:
                    self.down = True
                    print("Comms down!")
            rate.sleep()

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-num", "--number", help="number of robots")
    args = argParser.parse_args()
    node = comms_node(args.number)
    node.comms()
