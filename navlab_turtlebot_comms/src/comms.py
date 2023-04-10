#!/usr/bin/env python

import rospy
import numpy as np

from std_msgs.msg import Int16MultiArray
from navlab_turtlebot_common.msg import NominalTrajectory
from navlab_turtlebot_common.msg import State
from navlab_turtlebot_obstacles.msg import ZonotopeMsg, ZonotopeMsgArray

# Dubins planner
#robot1 = NominalTrajectory()
#robot2 = NominalTrajectory()
trajs = [NominalTrajectory()]*2
zonotopes = []

def robot1_cb(data):
    trajs[0] = data

def robot2_cb(data):
    trajs[1] = data

def zonotope_cb(data):
    print("here's some zonotopes")
    zonotopes = data

def check_occlusions():
    if len(trajs[1].states)>0:
        print(trajs[1].states[0].x)
    if len(trajs[0].states)>0 and len(trajs[1].states)>0:
        robot2_pos = trajs[1].states[0].x
        print(robot2_pos)
        for obs in zonotopes:
            print(obs)
            print(" ")
            #if not check_obs_collisions(positions,obs,2*params.R_BOT):
                #return False
    return True

def comms():
    pub = rospy.Publisher('comms_list',Int16MultiArray,queue_size=10)
    #pub1 = rospy.Publisher('/turtlebot1/comms',NominalTrajectory,queue_size=10)
    #pub2 = rospy.Publisher('/turtlebot2/comms',NominalTrajectory,queue_size=10)
    rospy.init_node('comms',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('/turtlebot1/planner/traj',NominalTrajectory,robot1_cb)
        rospy.Subscriber('/turtlebot2/planner/traj',NominalTrajectory,robot2_cb)
        rospy.Subscriber('/map/zonotopes',ZonotopeMsgArray,zonotope_cb)
        # Probability 1% of comms dropping each .1 sec, comms drop for occlusions
        if check_occlusions():
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
    comms()
