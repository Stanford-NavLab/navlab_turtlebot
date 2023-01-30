"""System ID

System ID for turtlebot dynamics.

"""

import rospy
import rospkg
import numpy as np
import time 
from scipy.spatial.transform import Rotation as R

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_srvs.srv import Empty

import common.params as params
from planning.linear_planning_model import LinearPlanningModel
import planning.utils as utils


# Parameters
N_DIM = 2  # number of dimensions
T_PLAN = 5  # planned trajectory time duration [s]
DT = 0.1  # trajectory discretization time interval [s]
N_PLAN = int(T_PLAN / DT)  # number of time steps in planned trajectory

V_MAX = 0.25  # maximum velocity [m/s]
W_MAX = 2.0  # maximum angular velocity [rad/s]

DATA_PATH = '/home/navlab-exxact/data/turtlebot_systemid/'

class SystemID:
    """SystemID class
    """
    def __init__(self):
        # Initialize node
        rospy.init_node('system_id', anonymous=True, disable_signals=True)
        self.rate = rospy.Rate(1//DT)
        
        # cmd publisher
        self.cmd_pub = rospy.Publisher('/cmd_vel', Twist, queue_size=10)

        # Odometry subscriber
        odom_sub = rospy.Subscriber('/odom', Odometry, self.odom_callback)

        self.odom = None  # [x, y, theta]


    def odom_callback(self, msg):
        """Odometry callback function

        Parameters
        ----------
        msg : Odometry
            Odometry message

        """
        x, y = msg.pose.pose.position.x, msg.pose.pose.position.y
        q = np.array([msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z, msg.pose.pose.orientation.w])
        r = R.from_quat(q)
        theta = r.as_euler('xyz')[2]
        self.odom = np.array([x, y, theta])
        

    def reset_sim(self):
        """Reset simulation
        """
        print("Resetting simulation")
        #rospy.wait_for_service('/gazebo/reset_simulation')
        rospy.wait_for_service('/gazebo/reset_world')
        try:
            #reset_proxy = rospy.ServiceProxy('/gazebo/reset_simulation', Empty)
            reset_proxy = rospy.ServiceProxy('/gazebo/reset_world', Empty)
            reset_proxy()
            rospy.sleep(2)
            reset_proxy()  # Reset twice to handle leftover inertia
        except rospy.ServiceException as e:
            print("Service call failed: %s"%e)


    def run_once(self, v, w, cmd, log):
        """Run once
        """
        cmd.linear.x = v
        cmd.angular.z = w

        print(f"Publishing cmd v = {v}, w = {w}")
        for i in range(N_PLAN):
            self.cmd_pub.publish(cmd)
            log[i,] = self.odom
            self.rate.sleep()
    
        np.save(DATA_PATH + f'v_{v:3.2f}_w_{w:2.1f}_{T_PLAN}s.npy', log)
        self.reset_sim()


    def run(self):
        """Run
        """
        # v = 0.25
        # w = 0.1
        cmd = Twist()
        # cmd.linear.x = v
        # cmd.angular.z = w
        log = np.zeros((N_PLAN, 3))  # x, y, theta

        # Wait until we have odometry
        while self.odom is None:
            self.rate.sleep()

        dv = 0.05
        dw = 0.1
        for v in np.arange(-V_MAX, V_MAX+dv, dv):
            for w in np.arange(-W_MAX, W_MAX+dw, dw):
                self.run_once(v, w, cmd, log)


if __name__ == '__main__':
    systemID = SystemID()
    try:
        systemID.run()
    except rospy.ROSInterruptException:
        pass
