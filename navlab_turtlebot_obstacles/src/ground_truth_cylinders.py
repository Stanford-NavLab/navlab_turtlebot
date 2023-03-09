import rclpy
from rclpy.node import Node
from rcl_interfaces.srv import GetParameters
from multi_rtd_interfaces.msg import Cylinder, CylinderArray

from world_parser import WorldParser
import numpy as np

class CylinderPublisher(Node):

    def __init__(self):
        super().__init__('ground_truth_cylinder_publisher')

        self.name = self.get_namespace()
        self.publisher = self.create_publisher(CylinderArray,self.name + 'map/cylinders',10)

        self.worlds_path = '/home/navlab-exxact-18/PX4-Autopilot/Tools/sitl_gazebo/worlds/'
        self.declare_parameter('rate',10.0)
        self.worldfile = None
        self.get_world_file()

    def setup(self):
        self.cylinders = self.get_cylinders()
        self.cylinder_array = self.get_cylinder_array()

        period = 1.0 / self.get_parameter('rate').value
        self.timer = self.create_timer(period,self.publish_map)

    def get_world_file(self):
        self.global_param_client = self.create_client(GetParameters,
                '/global_parameter_server/get_parameters')
        request = GetParameters.Request()
        request.names=['world_file']
        self.global_param_client.wait_for_service()
        # Don't use call_async here as we want to block until we get this parameter
        future = self.global_param_client.call_async(request)
        future.add_done_callback(self.global_param_callback)

    def global_param_callback(self,future):
        try:
            result = future.result()
        except Exception as e:
            self.get_logger().warn("Service call failed for %s"%e)
        else:
            self.worldfile = self.worlds_path + result.values[0].string_value + '.world'
            print("Got worldfile:",self.worldfile)
            self.setup()

    def publish_map(self):
        self.publisher.publish(self.cylinder_array)

    def get_cylinders(self):
        parser = WorldParser(self.worldfile)
        return parser.getCylinders()

    def get_cylinder_array(self):
        arr = [Cylinder() for _ in range(len(self.cylinders))]
        for i in range(len(self.cylinders)):
            c = self.cylinders[i]
            r = c['radius']; l = c['length']; center = c['pose'][:3]
            arr[i].center = center
            arr[i].length = l
            arr[i].radius = r

        cyl_arr = CylinderArray()
        cyl_arr.cylinders = arr
        return cyl_arr

def main(args=None):
    rclpy.init(args=args)

    map_publisher = CylinderPublisher()

    rclpy.spin(map_publisher)

    map_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()
