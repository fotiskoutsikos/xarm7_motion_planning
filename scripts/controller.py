#!/usr/bin/env python3

"""
Start ROS node to publish angles for the position control of the xArm7.
"""

# Ros handlers services and messages
import rospy, roslib
from std_msgs.msg import Float64
from sensor_msgs.msg import JointState
from gazebo_msgs.msg import ModelStates
#Math imports
from math import sin, cos, atan2, pi, sqrt
from numpy.linalg import inv, det, norm, pinv
import numpy as np
import time as t

# Arm parameters
# xArm7 kinematics class
from kinematics import xArm7_kinematics

# from tf.transformations import quaternion_matrix
# matrix = quaternion_matrix([1, 0, 0, 0])

from logger import SimulationLogger
import utils


class xArm7_controller():
    """Class to compute and publish joints positions"""
    def __init__(self,rate):
        rospy.on_shutdown(self.turn_off)

        # Init xArm7 kinematics handler
        self.kinematics = xArm7_kinematics()
        # Init logger
        self.logger = SimulationLogger()

        # joints' angular positions
        self.joint_angpos = [0, 0, 0, 0, 0, 0, 0]
        # joints' angular velocities
        self.joint_angvel = [0, 0, 0, 0, 0, 0, 0]
        # joints' states
        self.joint_states = JointState()
        # joints' transformation matrix wrt the robot's base frame
        self.A01 = self.kinematics.tf_A01(self.joint_angpos)
        self.A02 = self.kinematics.tf_A02(self.joint_angpos)
        self.A03 = self.kinematics.tf_A03(self.joint_angpos)
        self.A04 = self.kinematics.tf_A04(self.joint_angpos)
        self.A05 = self.kinematics.tf_A05(self.joint_angpos)
        self.A06 = self.kinematics.tf_A06(self.joint_angpos)
        self.A07 = self.kinematics.tf_A07(self.joint_angpos)
        # gazebo model's states
        self.model_states = ModelStates()

        # ROS SETUP
        # initialize subscribers for reading encoders and publishers for performing position control in the joint-space
        # Robot
        self.joint_states_sub = rospy.Subscriber('/xarm/joint_states', JointState, self.joint_states_callback, queue_size=1)
        self.joint1_pos_pub = rospy.Publisher('/xarm/joint1_position_controller/command', Float64, queue_size=1)
        self.joint2_pos_pub = rospy.Publisher('/xarm/joint2_position_controller/command', Float64, queue_size=1)
        self.joint3_pos_pub = rospy.Publisher('/xarm/joint3_position_controller/command', Float64, queue_size=1)
        self.joint4_pos_pub = rospy.Publisher('/xarm/joint4_position_controller/command', Float64, queue_size=1)
        self.joint5_pos_pub = rospy.Publisher('/xarm/joint5_position_controller/command', Float64, queue_size=1)
        self.joint6_pos_pub = rospy.Publisher('/xarm/joint6_position_controller/command', Float64, queue_size=1)
        self.joint7_pos_pub = rospy.Publisher('/xarm/joint7_position_controller/command', Float64, queue_size=1)
        # Obstacles
        self.model_states_sub = rospy.Subscriber('/gazebo/model_states', ModelStates, self.model_states_callback, queue_size=1)

        #Publishing rate
        self.period = 1.0/rate
        self.pub_rate = rospy.Rate(rate)

        self.publish()

    #SENSING CALLBACKS
    def joint_states_callback(self, msg):
        # ROS callback to get the joint_states

        self.joint_states = msg
        # (e.g. the angular position of joint 1 is stored in :: self.joint_states.position[0])

    def model_states_callback(self, msg):
        # ROS callback to get the gazebo's model_states

        self.model_states = msg
        # (e.g. #1 the position in y-axis of GREEN obstacle's center is stored in :: self.model_states.pose[1].position.y)
        # (e.g. #2 the position in y-axis of RED obstacle's center is stored in :: self.model_states.pose[2].position.y)

    def publish(self):
        # defining constants
        number_of_joints = 7
        direction = np.array([0, 1, 0])
        point = np.array([0.617, 0, 0.199])
        P_A = -0.2 * direction + point
        P_B = 0.2 * direction + point
        half_period = 10
        k = 5
        K = k * np.eye(3)
        k0 = 5

        self.logger.log_algorithm_parameters(half_period, k, k0)

        # set configuration
        # total pitch: j2-j4+j6+pi (upwards: 0rad)
        j2 = 0.7 ; j4 = np.pi/2
        j6 = - (j2-j4)
        self.joint_angpos = [0, j2, 0, j4, 0, j6, 0]
        tmp_rate = rospy.Rate(1)
        tmp_rate.sleep()
        self.joint4_pos_pub.publish(self.joint_angpos[3])
        tmp_rate.sleep()
        self.joint2_pos_pub.publish(self.joint_angpos[1])
        self.joint6_pos_pub.publish(self.joint_angpos[5])
        tmp_rate.sleep()
        print("The system is ready to execute your algorithm...")

        rostime_now = rospy.get_rostime()
        time_now = rostime_now.to_nsec()
        start_time = rostime_now.to_sec() - (half_period / 2)

        # fix
        time_prev = rospy.get_rostime().to_nsec()

        while not rospy.is_shutdown():
            # fix
            time_now = rospy.get_rostime().to_nsec()
            dt = (time_now - time_prev) / 1e9  # convert nanoseconds to seconds
            # fix
            rostime_now = rospy.get_rostime().to_sec()
            relative_time = rostime_now - start_time
            # angular_position = np.array(list(self.joint_states.position))
            self.A07 = self.kinematics.tf_A07(self.joint_angpos)
            end_effector_position = np.array(self.kinematics.extract_position(self.A07))
            # Extract obstacle positions
            obs1 = self.model_states.pose[1].position  # Green
            obs2 = self.model_states.pose[2].position  # Red
            obstacle1 = np.array([obs1.x, obs1.y, obs1.z])
            obstacle2 = np.array([obs2.x, obs2.y, obs2.z])

            # Compute each transformation matrix wrt the base frame from joints' angular positions
            # self.A01 = self.kinematics.tf_A01(self.joint_angpos)

            # Compute jacobian matrix
            J = self.kinematics.compute_jacobian(self.joint_angpos)  # shape (3, n_joints)
            pinvJ = pinv(J)  # shape (n_joints, 3)

            # path following
            dx_d_dt = utils.derivative_of_x_desired(relative_time, P_A, P_B, half_period)  # shape (3,)
            desired_position = utils.x_desired(relative_time, P_A, P_B, half_period)
            error = desired_position - end_effector_position  # shape (3,)
            first_component = pinvJ @ (dx_d_dt + K @ error)  # shape (n_joints,)

            # obstacle avoidance
            dq0 = k0 * utils.derivative_of_distance_to_obstacle(self.joint_angpos, obstacle1, obstacle2)  # shape (n_joints,)
            null_space_proj = np.eye(number_of_joints) - pinvJ @ J  # shape (n_joints, n_joints)
            second_component = null_space_proj @ dq0  # shape (n_joints,)

            # combine
            self.joint_angvel = first_component + second_component  # shape (n_joints,)
            
            # Convertion to angular position after integrating the angular speed in time
            # Integration
            self.joint_angpos = np.add( self.joint_angpos, [index * dt for index in self.joint_angvel] )
            # fix
            time_prev = time_now
            # fix
            # Publish the new joint's angular positions
            self.joint1_pos_pub.publish(self.joint_angpos[0])
            self.joint2_pos_pub.publish(self.joint_angpos[1])
            self.joint3_pos_pub.publish(self.joint_angpos[2])
            self.joint4_pos_pub.publish(self.joint_angpos[3])
            self.joint5_pos_pub.publish(self.joint_angpos[4])
            self.joint6_pos_pub.publish(self.joint_angpos[5])
            self.joint7_pos_pub.publish(self.joint_angpos[6])

            # log data
            self.logger.log_time(relative_time)
            self.logger.log_joint_angles(self.joint_states.position)
            self.logger.log_desired_joint_angles(self.joint_angpos)
            self.logger.log_desired_trajectory(desired_position)
            # roll, pitch, yaw assumed to be zero
            # height and radius of cylinders assumed to be constants
            self.logger.log_obstacles([
                obs1.x, obs1.y, obs1.z,
                obs2.x, obs2.y, obs2.z
            ])

            self.pub_rate.sleep()

    def turn_off(self):
        print("Saving log...")
        self.logger.save_to_csv()
        print("Log saved.")

def controller_py():
    # Starts a new node
    rospy.init_node('controller_node', anonymous=True)
    # Reading parameters set in launch file
    rate = rospy.get_param("/rate")

    controller = xArm7_controller(rate)
    rospy.spin()

if __name__ == '__main__':
    try:
        controller_py()
    except rospy.ROSInterruptException:
        pass
