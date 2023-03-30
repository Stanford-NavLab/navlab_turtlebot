#!/usr/bin/env python

import rospy
import numpy as np
import csv
import os
import argparse

from scipy.spatial.transform import Rotation as R
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import Float64

from navlab_turtlebot_common.msg import State, Control, NominalTrajectory
from navlab_turtlebot_control.utils import compute_control, EKF_prediction_step, EKF_correction_step, generate_robot_matrices
import navlab_turtlebot_common.params as params


class LQGTracker():
    """LQG tracker
    Tracks nominal trajectories by applying linear control feedback using
    state estimate and sending motor commands. If a new trajectory is received
    while tracking the current trajectory, it will switch to tracking the new
    trajectory once it has finished the current trajectory.
    """
    def __init__(self, name=''):
        self.name = name

        # Initialize node
        node_name = self.name + '_LQG_tracker' if self.name else 'LQG_tracker'
        rospy.init_node(node_name, anonymous=True)
        self.rate = rospy.Rate(1/params.DT)

        # Class variables
        self.idx = 0  # current index in the trajectory
        self.X_nom = None
        self.u_nom = None

        self.x_hat = np.zeros((4,1))  # state estimate
        self.P = params.P_0  # covariance

        self.z = np.zeros((3,1))  # measurement
        self.z_gt = np.zeros((3,1))  # ground-truth measurement (no noise)
        self.v_des = 0
        self.t_start = 0

        self.new_traj_flag = False

        # Publishers
        self.cmd_pub = rospy.Publisher(self.name + '/cmd_vel', Twist, queue_size=10)

        # Subscribers
        traj_sub = rospy.Subscriber(self.name + '/planner/traj', NominalTrajectory, self.traj_callback)
        measurement_sub = rospy.Subscriber('sensing/mocap_noisy', State, self.measurement_callback)
        gt_sub = rospy.Subscriber('sensing/mocap', State, self.gt_callback)

        # Logging
        path = '/home/navlab-nuc/Rover/flightroom_data/4_12_2022/debug/'
        filename = 'track_'+str(rospy.get_time())+'.csv'
        self.logger = csv.writer(open(os.path.join(path, filename), 'w'))
        self.logger.writerow(['t', 'x', 'y', 'theta', 'z_x', 'z_y', 'z_theta',
                              'x_nom', 'y_nom', 'theta_nom', 'v_nom',
                              'x_hat', 'y_hat', 'theta_hat', 'v_hat', 'u_w', 'u_a'])


    def traj_callback(self, data):
        """Trajectory subscriber callback.
        Save nominal states and controls from received trajectory.
        """
        self.X_nom = data.states
        self.u_nom = data.control
        rospy.loginfo("Received trajectory of length %d", len(data.states))


    def measurement_callback(self, data):
        """Measurement subscriber callback.
        Save received measurement.
        """
        # For mocap measurement
        self.z = np.array([[data.x],[data.y],[data.theta]])


    def gt_callback(self, data):
        """Ground-truth subscriber callback.
        Save received ground-truth.
        """
        # Mocap data
        self.z_gt = np.array([[data.x],[data.y],[data.theta]])


    def track(self):
        """Track next point in the current trajectory, and run the EKF for estimation.
        TODO
        """
        print("idx ", self.idx, " ----------------------------------------")

        x_nom_msg = self.X_nom[self.idx]
        x_nom = np.array([x_nom_msg.x, x_nom_msg.y, x_nom_msg.theta])
        u_nom_msg = self.u_nom
        u_nom = np.array([u_nom_msg.v, u_nom_msg.omega])

        # Start of first segment
        if self.idx == 0 and self.seg_num == 1:
            self.x_hat = x_nom

        A,B,C,K = generate_robot_matrices(x_nom, u_nom, params.Q_LQR, params.R_LQR, params.DT)
        # K = np.array([[0, 0, 1, 0],
        #               [0, 0, 0, 1]])

        # ======== EKF Update ========
        self.x_hat, self.P = EKF_correction_step(self.x_hat, self.P, self.z, C, params.R_EKF)

        # ======== Apply feedback control law ========
        u = compute_control(x_nom, u_nom, self.x_hat, K)

        # Create motor command msg
        cmd = Twist()

        # Closed-loop
        cmd.linear.x = u[0]
        cmd.angular.z = u[1]

        print("u_nom ", u_nom, " u ", u)

        self.cmd_pub.publish(cmd)
        self.idx += 1

        # Log data
        # self.logger.writerow([rospy.get_time(), self.z_gt[0][0], self.z_gt[1][0], self.z_gt[2][0],
        #                       self.z[0][0], self.z[1][0], self.z[2][0],
        #                       x_nom[0][0], x_nom[1][0], x_nom[2][0], x_nom[3][0], self.x_hat[0][0],
        #                       self.x_hat[1][0], self.x_hat[2][0], self.x_hat[3][0], u[0][0], u[1][0]])

        # ======== Check for end of trajectory ========
        if self.idx >= len(self.X_nom):
            self.X_nom = None
            self.u_nom = None
            self.idx = 0
            self.stop_motors()

        # ======== EKF Predict ========
        self.x_hat, self.P = EKF_prediction_step(self.x_hat, u, self.P, A, params.Q_EKF, params.DT)


    def stop_motors(self):
        """Send stop command to all motors
        """
        rospy.loginfo("Stopping motors")
        motor_cmd = Twist()
        motor_cmd.linear.x = 0.0
        motor_cmd.angular.z = 0.0
        self.cmd_pub.publish(motor_cmd)


    def run(self):
        """Run node
        """
        rospy.loginfo("Running LQG Tracker")
        while not rospy.is_shutdown():

            if self.X_nom_curr is not None:
                self.track()

            self.rate.sleep()

        # spin() simply keeps python from exiting until this node is stopped
        rospy.spin()


if __name__ == '__main__':
    argParser = argparse.ArgumentParser()
    argParser.add_argument("-n", "--name", help="robot name")
    args = argParser.parse_args()

    lqgt = LQGTracker(args.name)
    try:
        lqgt.run()
    except rospy.ROSInterruptException:
        pass
