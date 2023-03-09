import rclpy
from rclpy.node import Node
from multi_rtd_interfaces.msg import ZonotopeMsg, ZonotopeMsgArray
from zonotope import Zonotope

from world_parser import WorldParser
import numpy as np

import matplotlib.pyplot as plt
from matplotlib.patches import Polygon

class MapVisualizer(Node):

    def __init__(self):
        super().__init__('zonotope_map_visualizer')
        self.subscription = self.create_subscription(
                ZonotopeMsgArray,
                'zonotope_map',
                self.sub_callback,
                10)

        self.subscription

        # Create pyplot figure
        self.fig = plt.figure()
        self.ax = self.fig.add_subplot(1,1,1)

        self.shown = False


    def sub_callback(self,msg):
        if self.shown:
            return
        self.shown = True

        for z in msg.zonotopes:
            G = np.reshape(z.generators,(z.dim,z.num_gen))
            G = G[:2,:]
            center = G[:,0]
            generators = G[:,1:]
            center = np.reshape(center,(2,1))
            generators = np.reshape(generators,(2,z.num_gen-1))
            z = Zonotope(center,generators)
            p = Polygon(z.vertices().T,color='b',fill=False)
            self.ax.add_patch(p)

        self.ax.set_xlim([-5,55]); self.ax.set_ylim([-5,55])
        plt.show(block=True)


def main(args=None):
    rclpy.init(args=args)

    mapVis = MapVisualizer()
    rclpy.spin(mapVis)

    mapVis.destroy_node
    rclpy.shutdown()
