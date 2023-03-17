#!/usr/bin/env python

import rospy
#import rclpy
#from rclpy.node import Node
from navlab_turtlebot_obstacles.msg import ZonotopeMsg, ZonotopeMsgArray

from world_parser import WorldParser
import numpy as np

class ZonotopePublisher():

    def __init__(self):
        #super().__init__('ground_truth_obstacle_publisher')
        rospy.init_node('ground_truth_obstacle_publisher')

        self.name = rospy.get_namespace()
        # Not sure why this line was necessary ^^

        #self.publisher = self.create_publisher(ZonotopeMsgArray,self.name + 'map/zonotopes',10)
        self.publisher = rospy.Publisher(self.name+'map/zonotopes',ZonotopeMsgArray, queue_size=10)

        # Set node parameters
        rospy.set_param('worldfile','/home/izzie/catkin_ws/src/navlab_turtlebot_base/worlds/flightroom_obstacles.world')
        rospy.set_param('rate',1.0)

        # Use worldfile parameter to get zonotopes
        self.zonotopes = self.get_zonotopes()
        self.zonotope_array = self.get_zonotope_array()

        # Setup timer to publish map
        period = 1.0 / rospy.get_param('rate')
        self.timer = rospy.Timer(rospy.Duration(period),self.publish_map)

    def publish_map(self, event=None):
        self.publisher.publish(self.zonotope_array)

    def get_zonotopes(self):
        worldfile = rospy.get_param('worldfile')
        parser = WorldParser(worldfile)
        return parser.allZonotopes()

    def get_zonotope_array(self):
        arr = [ZonotopeMsg() for _ in range(len(self.zonotopes))]
        for i in range(len(self.zonotopes)):
            zonotope = self.zonotopes[i]
            (dim,num_gen) = zonotope.shape
            arr[i].dim = dim
            arr[i].num_gen = num_gen
            arr[i].generators = list(zonotope.astype(np.float).flatten())
        zon_arr = ZonotopeMsgArray()
        zon_arr.zonotopes = arr

        return zon_arr

def main(args=None):
    #rospy.init(args=args) -> changed to just init_node, called in __init__

    map_publisher = ZonotopePublisher()

    rospy.spin()

    #map_publisher.destroy_node()
    #rospy.shutdown()

if __name__ == "__main__":
    main()
