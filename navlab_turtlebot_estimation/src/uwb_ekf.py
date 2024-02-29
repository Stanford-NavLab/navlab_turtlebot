#!/usr/bin/env python
"""Extended Kalman Filter using UWB measurements.

"""

__authors__ = "D. Knowles"
__date__ = "28 Feb 2024"

class UWBEKF():
    """UWB Extended Kalman Filter

    Params
    ------
    name : string
        Turtlebot name used to publish messages.

    """
    def __init__(self, name=None):

        if name is None:
            self.name = os.environ['USER']
        else:
            self.name = name

        # Initialize node
        node_name = name + '_uwb_ekf'
        rospy.init_node(node_name, anonymous=True)

if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args = argParser.parse_args()

    # Auto-get turtlebot name
    if args.name is None:
        args.name = os.environ['USER']

    uwb_ekf = UWBEKF(args.name)
