#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray
from navlab_turtlebot_common.msg import NominalTrajectory
from navlab_turtlebot_common.msg import State
from navlab_turtlebot_obstacles.msg import ZonotopeMsg, ZonotopeMsgArray
from zonotope import Zonotope

class comms_node():
    def __init__(self):
        # Dubins planner
        self.trajs = [NominalTrajectory()]*2
        self.zonotopes = []

    def robot1_cb(self,data):
        self.trajs[0] = data

    def robot2_cb(self,data):
        self.trajs[1] = data

    def zonotope_cb(self,data):
        self.zonotopes = data.zonotopes

    def check_occlusions(self):
        #if len(trajs[1].states)>0:
            #print(trajs[1].states[0].x)
        if len(self.trajs[0].states)>0 and len(self.trajs[1].states)>0:
            r1_x = self.trajs[0].states[0].x
            r1_y = self.trajs[0].states[0].y
            r2_x = self.trajs[1].states[0].x
            r2_y = self.trajs[1].states[0].y
            r_m = (r2_y-r1_y)/(r2_x-r1_x)
            r_b = r_m*r2_x+r2_y
            for obs in self.zonotopes:
                c = np.array(obs.generators[0:3]).reshape((obs.dim,1))
                g = np.array(obs.generators[3:]).reshape((obs.dim,obs.num_gen-1))
                vertices = Zonotope(c,g).vertices()
                print(vertices[:2])
                for i in range(vertices.shape[1]):
                    p1 = vertices[:2,i]
                    p2 = vertices[:2,(i+1)%vertices.shape[1]]
                    m = (p2-p1)[1]/(p2-p1)[0]
                    if m != r_m: #Check the two line segments aren't parallel
                        b = m*p2[0]+p2[1]
                        x = (r_b-b)/(m-r_m)
                        y = m*x+b
                        # If the point is on the line segments, the difference between x and bounds should have opposite signs to be in between
                        if abs(x-p1[0])/(x-p1[0])!=abs(x-p2[0])/(x-p2[0]) and \
                           abs(x-r1_x)/(x-r1_x) != abs(x-r2_x)/(x-r2_x):
                            return False
        return True

    def comms(self):
        pub = rospy.Publisher('comms_list',Int16MultiArray,queue_size=10)
        #pub1 = rospy.Publisher('/turtlebot1/comms',NominalTrajectory,queue_size=10)
        #pub2 = rospy.Publisher('/turtlebot2/comms',NominalTrajectory,queue_size=10)
        rospy.init_node('comms',anonymous=True)
        rate = rospy.Rate(10)
        while not rospy.is_shutdown():
            rospy.Subscriber('/turtlebot1/planner/traj',NominalTrajectory,self.robot1_cb)
            rospy.Subscriber('/turtlebot2/planner/traj',NominalTrajectory,self.robot2_cb)
            rospy.Subscriber('/map/zonotopes',ZonotopeMsgArray,self.zonotope_cb)
            # Probability 1% of comms dropping each .1 sec, comms drop for occlusions
            if self.check_occlusions():
                no_comms = np.random.choice([0,1],size=(1,2),p=[.99,.01])
            else:
                no_comms = np.array([1,1])
            # If comms are working, robot1 gets the info from robot2, and vice versa
            comms_list = Int16MultiArray(data=list(no_comms[0]))
            pub.publish(comms_list)
            if np.sum(no_comms)==0:
                #pub1.publish(trajs[1])
                #pub2.publish(trajs[0])
                pass
            else:
                print("Comms down!")
            rate.sleep()

if __name__ == '__main__':
    node = comms_node()
    node.comms()
