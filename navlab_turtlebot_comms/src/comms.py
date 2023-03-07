#!/usr/bin/env python

import rospy
import numpy as np
from geometry_msgs.msg import Twist
from trajectory_msgs.msg import JointTrajectory
from common.msg import NominalTrajectory
from common.msg import State
import common.params as params
from planning.utils import check_obs_collision

obstacles = params.OBSTACLES
robot1 = NominalTrajectory()
robot2 = NominalTrajectory()

def robot1_cb(data):
    robot1 = data

def robot2_cb(data):
    robot2 = data

def check_occlusions():
    print(robot2)
    if len(robot2.states)>0:
        print(robot2.states[0].x)
    if len(robot1.states)>0 and len(robot2.states)>0:
        robot2_pos = robot2.states[0].x
        print(robot2_pos)
        #for obs in obstacles:
            #if not check_obs_collisions(positions,obs,2*params.R_BOT):
                #return False
    return True

def comms():
    pub1 = rospy.Publisher('/turtlebot1/comms',NominalTrajectory,queue_size=10)
    pub2 = rospy.Publisher('/turtlebot2/comms',NominalTrajectory,queue_size=10)
    rospy.init_node('comms',anonymous=True)
    rate = rospy.Rate(10)
    while not rospy.is_shutdown():
        rospy.Subscriber('/turtlebot1/planner/traj',NominalTrajectory,robot1_cb)
        rospy.Subscriber('/turtlebot2/planner/traj',NominalTrajectory,robot2_cb)
        # Probability 1% of comms dropping each .1 sec, comms drop for occlusions
        if True: #check_occlusions():
            no_comms = np.random.choice([0,1],size=(1,2),p=[.99,.01])
        else:
            no_comms = np.array([1,1])
        # If comms are working, robot1 gets the info from robot2, and vice versa
        if np.sum(no_comms)==0:
            pub1.publish(robot2)
            pub2.publish(robot1)
        else:
            print("Comms down!")
        rate.sleep()

if __name__ == '__main__':
    comms()
