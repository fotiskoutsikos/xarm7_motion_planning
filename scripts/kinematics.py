#!/usr/bin/env python3

"""
Compute state space kinematic matrices for xArm7 robot arm (5 links, 7 joints)
"""

import numpy as np

class xArm7_kinematics():

    A_list = []

    def __init__(self):

        self.l1 = 0.267
        self.l2 = 0.293
        self.l3 = 0.0525
        self.l4 = 0.3512
        self.l5 = 0.1232

        self.theta1 = 0.2225 #(rad) (=12.75deg)
        self.theta2 = 0.6646 #(rad) (=38.08deg)

        self.A_list = []

        pass

    def compute_jacobian(self, r_joints_array):
        # J = np.matrix([ [ J_11 , J_12 , J_13 , J_14 , J_15 , J_16 , J_17 ],\
        #                 [ J_21 , J_22 , J_23 , J_24 , J_25 , J_26 , J_27 ],\
        #                 [ J_31 , J_32 , J_33 , J_34 , J_35 , J_36 , J_37 ]])


        A_list = self.compute_A_list(r_joints_array)
        b = self.get_b(r_joints_array)
        p = self.get_p(r_joints_array)
        p_end = self.extract_position(A_list[7])
        J = np.zeros((3, 7))
        for i in range(7):
            J[:, i] = np.cross(b[i], p_end - p[i])
        return J



    def compute_jacobian_numerical(self, joint_angles, dqi=1e-6):
        n_joints = len(joint_angles)
        J = np.zeros((3, n_joints))

        # Get current end-effector position
        p_0 = self.extract_position(self.tf_A07(joint_angles))

        for i in range(n_joints):
            perturbed = joint_angles.copy()
            perturbed[i] += dqi
            p_i = self.extract_position(self.tf_A07(perturbed))
            J[:, i] = (p_i - p_0) / dqi

        return J

    def Rot(self, axis, angle):
        """Return 4x4 homogeneous rotation matrix about the specified axis ('x', 'y', 'z')."""
        c, s = np.cos(angle), np.sin(angle)
        if axis == 'x':
            R = np.array([
                [1,  0,  0, 0],
                [0,  c, -s, 0],
                [0,  s,  c, 0],
                [0,  0,  0, 1]
            ])
        elif axis == 'y':
            R = np.array([
                [ c, 0,  s, 0],
                [ 0, 1,  0, 0],
                [-s, 0,  c, 0],
                [ 0, 0,  0, 1]
            ])
        elif axis == 'z':
            R = np.array([
                [c, -s, 0, 0],
                [s,  c, 0, 0],
                [0,  0, 1, 0],
                [0,  0, 0, 1]
            ])
        else:
            raise ValueError("axis must be 'x', 'y', or 'z'")
        return R

    def Tra(self, axis, length):
        """Return 4x4 homogeneous translation matrix along the specified axis ('x', 'y', 'z')."""
        T = np.eye(4)
        if axis == 'x':
            T[0, 3] = length
        elif axis == 'y':
            T[1, 3] = length
        elif axis == 'z':
            T[2, 3] = length
        else:
            raise ValueError("axis must be 'x', 'y', or 'z'")
        return T

    def tf_A01(self, r_joints_array):
        q1 = r_joints_array[0]
        tf = self.Rot('z', q1) @ self.Tra('z', self.l1)
        return tf

    def tf_A02(self, r_joints_array):
        q2 = r_joints_array[1]
        tf_A12 = self.Rot('x', -np.pi / 2) @ self.Rot('z', q2)
        tf = np.dot( self.tf_A01(r_joints_array), tf_A12 )
        return tf

    def tf_A03(self, r_joints_array):
        q3 = r_joints_array[2]
        tf_A23 = self.Rot('x', np.pi/2) @ self.Rot('z', q3) @ self.Tra('z', self.l2)
        tf = np.dot( self.tf_A02(r_joints_array), tf_A23 )
        return tf

    def tf_A04(self, r_joints_array):
        q4 = r_joints_array[3]
        tf_A34 = self.Rot('x', np.pi/2) @ self.Tra('x', self.l3) @ self.Rot('z', q4)
        tf = np.dot( self.tf_A03(r_joints_array), tf_A34 )
        return tf

    def tf_A05(self, r_joints_array):
        q5 = r_joints_array[4]
        tf_A45 = self.Rot('x', np.pi/2) @ self.Tra('x', self.l4 * np.sin(self.theta1)) @ self.Rot('z', q5) @ self.Tra('z', self.l4 * np.cos(self.theta1))
        tf = np.dot( self.tf_A04(r_joints_array), tf_A45 )
        return tf

    def tf_A06(self, r_joints_array):
        q6 = r_joints_array[5]
        tf_A56 = self.Rot('x', np.pi/2) @ self.Rot('z', q6)
        tf = np.dot( self.tf_A05(r_joints_array), tf_A56 )
        return tf

    def tf_A07(self, r_joints_array):
        q7 = r_joints_array[6]
        tf_A67 = self.Rot('x', -np.pi/2) @ self.Tra('x', self.l5 * np.sin(self.theta2)) @ self.Rot('z', q7) @ self.Tra('z', self.l5 * np.cos(self.theta2))
        tf = np.dot( self.tf_A06(r_joints_array), tf_A67 )
        return tf
    
    def extract_position(self, tf_matrix):
        position = tf_matrix[:3, 3]
        return position
    
    def extract_x_axis(self, tf_matrix):
        x_axis = tf_matrix[:3, 0]
        return x_axis
 

    def extract_y_axis(self, tf_matrix):
        y_axis = tf_matrix[:3, 1]
        return y_axis

    
    def extract_z_axis(self, tf_matrix):
        z_axis = tf_matrix[:3, 2]
        return z_axis

    def get_p(self, r_joints_array=None):
        if r_joints_array is not None:
            self.compute_A_list(r_joints_array)

        return np.array([self.extract_position(A) for A in self.A_list])


    def get_b(self, r_joints_array=None):
        if r_joints_array is not None:
            self.compute_A_list(r_joints_array)

        # Twist angles α₀…α₆
        alphas = [  0,
               -np.pi/2,
                np.pi/2,
                np.pi/2,
                np.pi/2,
                np.pi/2,
               -np.pi/2 ]

        b = []
        for i, a in enumerate(alphas):
            axis_tf = self.A_list[i] @ self.Rot('x', a)
            b.append(self.extract_z_axis(axis_tf))
        return np.array(b)


    def compute_A_list(self, r_joints_array):
        q1, q2, q3, q4, q5, q6, q7 = r_joints_array

        # Compute the transformation matrices for each joint
        A01 = self.Rot('z', q1) @ self.Tra('z', self.l1)
        A12 = self.Rot('x', -np.pi / 2) @ self.Rot('z', q2)
        A23 = self.Rot('x', np.pi/2) @ self.Rot('z', q3) @ self.Tra('z', self.l2)
        A34 = self.Rot('x', np.pi/2) @ self.Tra('x', self.l3) @ self.Rot('z', q4)
        A45 = self.Rot('x', np.pi/2) @ self.Tra('x', self.l4 * np.sin(self.theta1)) @ self.Rot('z', q5) @ self.Tra('z', self.l4 * np.cos(self.theta1))
        A56 = self.Rot('x', np.pi/2) @ self.Rot('z', q6)
        A67 = self.Rot('x', -np.pi/2) @ self.Tra('x', self.l5 * np.sin(self.theta2)) @ self.Rot('z', q7) @ self.Tra('z', self.l5 * np.cos(self.theta2))
        
        A02 = A01 @ A12
        A03 = A02 @ A23
        A04 = A03 @ A34
        A05 = A04 @ A45
        A06 = A05 @ A56
        A07 = A06 @ A67

        self.A_list = [np.eye(4), A01, A02, A03, A04, A05, A06, A07]
        return self.A_list
    
    def get_A01 (self):
        return self.A_list[1]
    
    def get_A02 (self):
        return self.A_list[2]
    
    def get_A03 (self):
        return self.A_list[3]
    
    def get_A04 (self):
        return self.A_list[4]
    
    def get_A05 (self):
        return self.A_list[5]
    
    def get_A06 (self):
        return self.A_list[6]
    
    def get_A07 (self):
        return self.A_list[7]


    def main(self):
        kin = xArm7_kinematics()
        j2 = 0.7 ; j4 = np.pi/2
        j6 = - (j2-j4)
        joint_angpos = [0, j2, 0, j4, 0, j6, 0]
        a07 = kin.tf_A07(joint_angpos)
        end_pos = kin.extract_position(a07)
        J = kin.compute_jacobian(joint_angpos)
        J_num = kin.compute_jacobian_numerical(joint_angpos)
        print("a07 = \n", a07)
        print("end_pos = \n", end_pos)
        print("J = \n", J)
        print("J_num = \n", J_num)
        print("error = \n", J - J_num)

        for i, b in enumerate(self.get_b(joint_angpos)):
            print(f"b{i} = {np.round(b, 3)} | norm = {np.linalg.norm(b)}")
        for i, p in enumerate(self.get_p(joint_angpos)):
            print(f"p{i} = {np.round(p, 3)} | norm = {np.linalg.norm(p)}")

        
if __name__ == "__main__":
    kinematics = xArm7_kinematics()
    kinematics.main()


