import rclpy
from rclpy.node import Node
from multi_rtd_interfaces.msg import Cylinder, CylinderArray
from px4_msgs.msg import VehicleOdometry

import numpy as np

class ProximityFilterCylinders(Node):

    def __init__(self):
        super().__init__('proximity_filter_cylinders')

        self.ns = self.get_namespace()
        self.ns = '/iris_0/'

        # Set default parameters
        self.declare_parameter('map_topic','/map/cylinders')
        self.declare_parameter('sense_radius',20.0)

        self.map = []
        self.pose = np.zeros((3,1))

        # Subscribers and Publishers
        odom_sub = self.create_subscription(
                VehicleOdometry,
                self.ns + 'fmu/vehicle_odometry/out',
                self.set_pose,
                10)
                
        map_sub = self.create_subscription(
                CylinderArray,
                self.get_parameter('map_topic').value,
                self.filter_map,
                10)

        self.map_pub = self.create_publisher(
                CylinderArray,
                self.ns + 'detected_cylinders',
                10)

    def set_pose(self,msg):
        self.pose[0] = msg.x
        self.pose[1] = msg.y
        self.pose[2] = msg.z

    def in_map(self,cyl):
        for c in self.map:
            if c.radius == cyl.radius and c.length == cyl.length and c.center == cyl.center:
                return True
        return False

    def filter_map(self,msg):
        self.map = []
        for c in msg.cylinders:
            d = np.sqrt(sum([(self.pose[i]-c.center[i])**2 for i in range(2)]))
            if d < c.radius + self.get_parameter('sense_radius').value:
                self.map.append(c)
        cMsg = CylinderArray()
        cMsg.cylinders = self.map
        self.map_pub.publish(cMsg)

def main(args=None):
    rclpy.init(args=args)

    filter = ProximityFilterCylinders()

    rclpy.spin(filter)

    filter.destory_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
