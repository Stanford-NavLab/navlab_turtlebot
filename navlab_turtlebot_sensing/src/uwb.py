#!/usr/bin/env python
"""Publisher for UWB measurements.

Compatible with the Qorvo DWM1001-DEV firmware available at:
https://github.com/Stanford-NavLab/UWB-mesh-ranging

"""

__authors__ = "D. Knowles"
__date__ = "04 Oct 2023"

import os
import serial
import argparse

import rospy

from navlab_turtlebot_msgs.msg import UWBRange

class UWB():
    """Process and publish UWB measurements

    Params
    ------
    name : string
        Turtlebot name used to publish messages.

    """
    def __init__(self, name="turtlebot3"):

        if name is None:
            self.name = os.environ['USER']
        else:
            self.name = name

        # there will be a different publisher for each neighbor
        self.publishers = {}

        # Initialize node
        node_name = name + '_uwb'
        rospy.init_node(node_name, anonymous=True)

        # open serial port and read from it
        ser = serial.Serial('/dev/ttyACM0', 115200, timeout=1)
        while ser.isOpen() and not rospy.is_shutdown():
            line = ser.readline().decode().split(" ")

            # publish distances
            if line[:2] == ["RX","DIST"] or line[:2] == ["TX","DIST"]:
                self.publish(line)

    def publish(self, line):
        """Publish latest UWB measurement

        Parameters
        ----------
        line : list
            Data from UWB to publish

        """
        msg = UWBRange()
        msg.header.stamp = rospy.Time.now()
        sender_id = "turtlebot" + line[2]
        msg.sender_id = sender_id
        msg.receipient_id = self.name
        msg.range_dist_m = float(line[3])

        if sender_id not in self.publishers:
            self.publishers[sender_id] = rospy.Publisher('/' + self.name + '/uwb/' \
                                          + "_".join(sorted([self.name,sender_id])),
                                           UWBRange,
                                           queue_size=1)


        self.publishers[sender_id].publish(msg)


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    argParser.add_argument("__name", help="ros name")
    argParser.add_argument("__log", help="log name")
    args = argParser.parse_args()

    # Auto-get turtlebot name
    if args.name is None:
        # args.name = os.environ['USER']
        args.name = "turtlebot3"

    uwb = UWB(args.name)
