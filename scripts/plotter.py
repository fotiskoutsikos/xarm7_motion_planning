# plotting.py
import matplotlib.pyplot as plt
import pandas as pd
import numpy as np
import os
import utils

class simulationPlotter:
    def __init__(self, logs_dir):
        self.number_of_joints = 7
        self.logs_dir = logs_dir
        self.plots_dir = "plots/" + os.path.basename(os.path.normpath(logs_dir)) + "/"
        os.makedirs(self.plots_dir, exist_ok=True)

        # extract values from logger
        time_data = pd.read_csv(self.logs_dir+'time.csv')
        angles_data = pd.read_csv(self.logs_dir+'joint_angles.csv')
        desired_angles_data = pd.read_csv(self.logs_dir+'desired_joint_angles.csv')
        desired_trajectory = pd.read_csv(self.logs_dir+'desired_trajectory.csv')
        obstacle_positions = pd.read_csv(self.logs_dir+'obstacle_positions.csv')
        obstacle_dimensions = pd.read_csv(self.logs_dir+'obstacle_constants.csv')

        self.time = time_data['time']
        self.joint_angles = angles_data[['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']].values
        self.desired_joint_angles = desired_angles_data[['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7']].values
        self.desired_trajectory = desired_trajectory[['x', 'y', 'z']].values
        self.joint_positions = utils.find_joint_positions(self.desired_joint_angles)
        self.obstacle1_pos = obstacle_positions.iloc[:, :3].values  # First obstacle [x, y, z]
        self.obstacle2_pos = obstacle_positions.iloc[:, 3:6].values  # Second obstacle [x, y, z]
        self.radius = obstacle_dimensions.iloc[0, 0]  # Radius of the obstacles
        self.height = obstacle_dimensions.iloc[0, 1]  # Height of the obstacles

    def plot_joint_angles(self, show=False):
        # Plot the joint angles over time
        plt.figure(figsize=(10, 6))
        for i in range(self.number_of_joints):
            plt.plot(self.time, self.desired_joint_angles[:, i], label=f'Joint {i+1}')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Joint Angle (radians)')
        plt.title('Joint Angles Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir+'joint_angles.png')
        if show:
            plt.show()
        
    def plot_desired_angle_error(self, show=False):
        # Plot the joint angles over time
        plt.figure(figsize=(10, 6))
        for i in range(self.number_of_joints):
            plt.plot(self.time, self.desired_joint_angles[:, i]-self.joint_angles[:, i], label=f'Joint {i+1}')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('angle error (radians)')
        plt.title('Joint Angles error Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir+'joint_angles_error.png')
        if show:
            plt.show()

    def plot_joint_angular_velocities(self, show=False):
        # Compute joint velocities (approximation by numerical differentiation)
        epsilon = 1e-9
        dt = np.diff(self.time) + epsilon  # Time differences between consecutive time steps
        joint_velocities = np.diff(self.joint_angles, axis=0) / dt[:, None]  # Velocity = (angle[i+1] - angle[i]) / dt

        # Plot the joint velocities over time
        plt.figure(figsize=(10, 6))
        for i in range(self.number_of_joints):
            plt.plot(self.time[1:], joint_velocities[:, i], label=f'Joint {i+1} Velocity')
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Joint Velocity (radians/second)')
        plt.title('Joint Velocities Over Time')
        plt.legend()
        plt.grid(True)

        plt.savefig(self.plots_dir+'angular_velocities.png')
        if show:
            plt.show()

    def plot_end_effector_position(self, show=False):
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, self.joint_positions[:, -1, 0], label=f'x-axis')
        plt.plot(self.time, self.joint_positions[:, -1, 1], label=f'y-axis')
        plt.plot(self.time, self.joint_positions[:, -1, 2], label=f'z-axis')

        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('position (meters)')
        plt.title('position of end-effector Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir+'end_effector_position.png')
        if show:
            plt.show()
    
    def plot_end_effector_error(self, show=False):
        """Plot the error between actual and desired end-effector position over time."""
        # Compute error as Euclidean distance at each time step
        error = np.linalg.norm(self.joint_positions[:, -1, :] - self.desired_trajectory, axis=1)

        plt.figure(figsize=(10, 6))
        plt.plot(self.time, error, label='Position Error (Euclidean Distance)', color='red')

        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Error (meters)')
        plt.title('End-Effector Position Error Over Time')
        plt.legend()
        plt.grid(True)

        plt.savefig(self.plots_dir + 'end_effector_error.png')
        if show:
            plt.show()

    def plot_end_effector_velocity_and_acceleration(self, show=False):
        # Compute end-effector velocity and acceleration (approximation by numerical differentiation)
        epsilon = 1e-9
        dt = np.diff(self.time) + epsilon

        end_effector_position = self.joint_positions[:, -1, :]  # Extract the end-effector position
        end_effector_velocity = np.diff(end_effector_position, axis=0) / dt[:, None] # Velocity = (pos[i+1] - pos[i]) / dt
        end_effector_acceleration = np.diff(end_effector_velocity, axis=0) / dt[1:, None] # Acceleration = (vel[i+1] - vel[i]) / dt

        t_velocity = self.time[1:]
        t_acceleration = self.time[2:]

        # Plot the end-effector velocity over time
        plt.figure(figsize=(10, 6))
        plt.plot(t_velocity, end_effector_velocity[:, 0], label='Vx')
        plt.plot(t_velocity, end_effector_velocity[:, 1], label='Vy')
        plt.plot(t_velocity, end_effector_velocity[:, 2], label='Vz')

        plt.xlabel('Time (seconds)')
        plt.ylabel('Velocity (m/s)')
        plt.title('End-Effector Velocity Over Time')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir + 'end_effector_velocity.png')
        if show: 
            plt.show()

        # Plot the end-effector acceleration over time
        plt.figure(figsize=(10, 6))
        plt.plot(t_acceleration, end_effector_acceleration[:, 0], label='ax')
        plt.plot(t_acceleration, end_effector_acceleration[:, 1], label='ay')
        plt.plot(t_acceleration, end_effector_acceleration[:, 2], label='az')

        plt.xlabel('Time (seconds)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('End-Effector Acceleration Over Time')
        plt.legend()
        plt.grid(True)

        plt.savefig(self.plots_dir + 'end_effector_acceleration.png')
        if show:
            plt.show()
        
    def plot_obstacle_robot_distance(self, show=False):
        robot_obstacle_distance = []
        # Loop over each time step and compute distances for each joint
        for i in range(len(self.time)):
            # Compute the distance from each joint to each obstacle
            distance = utils.robot_to_obstacle_distance(self.joint_angles[i], self.obstacle1_pos[i], self.obstacle2_pos[i])
            robot_obstacle_distance.append(distance)
            
        robot_obstacle_distance = np.array(robot_obstacle_distance)

        # Plot the distances over time
        plt.figure(figsize=(10, 6))
        plt.plot(self.time, robot_obstacle_distance)
        
        # Add labels and title
        plt.xlabel('Time (seconds)')
        plt.ylabel('Distance (meters)')
        plt.title('Distance of robot to obstacles')
        plt.grid(True)
        
        plt.savefig(self.plots_dir+'robot_obstacle_distance.png')
        if show:
            plt.show()

    def plot_desired_velocity_acceleration(self, show=False):
        # Compute desired velocity and desired acceleration (approximation by numerical differentiation)
        epsilon = 1e-9
        dt = np.diff(self.time) + epsilon  # Time differences between consecutive time steps
        
        velocity = np.diff(self.desired_trajectory, axis=0) / dt[:, None]  # Velocity = (des_pos[i+1] - des_pos[i]) / dt

        acceleration = np.diff(velocity, axis=0) / dt[1:, None]  # Acceleration = (des_vel[i+1] - des_vel[i]) / dt

        time_velocity = self.time[1:]
        time_acceleration = self.time[2:]

        # Plot the desired velocity over time
        plt.figure(figsize=(10, 6))
        plt.plot(time_velocity, velocity[:, 0], label='Vx')
        plt.plot(time_velocity, velocity[:, 1], label='Vy')
        plt.plot(time_velocity, velocity[:, 2], label='Vz')

        # Add labels and title
        plt.xlabel('Time (s)')
        plt.ylabel('Velocity (m/s)')
        plt.title('Desired End-Effector Velocity')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir + 'desired_velocity.png')
        if show:
            plt.show()

        # Plot the desired acceleration over time
        plt.figure(figsize=(10, 6))
        plt.plot(time_acceleration, acceleration[:, 0], label='ax')
        plt.plot(time_acceleration, acceleration[:, 1], label='ay')
        plt.plot(time_acceleration, acceleration[:, 2], label='az')

        # Add labels and title
        plt.xlabel('Time (s)')
        plt.ylabel('Acceleration (m/s²)')
        plt.title('Desired End-Effector Acceleration')
        plt.legend()
        plt.grid(True)
        
        plt.savefig(self.plots_dir + 'desired_acceleration.png')
        if show:
            plt.show()
        

logs_path = 'logs/2025-04-25_11-22-01/'
plotter = simulationPlotter(logs_path)


plotter.plot_joint_angles(show=False)
plotter.plot_joint_angular_velocities(show=False)
plotter.plot_end_effector_position(show=False)
plotter.plot_end_effector_error(show=False)
plotter.plot_obstacle_robot_distance(show=False)
plotter.plot_desired_velocity_acceleration(show=False)
plotter.plot_end_effector_velocity_and_acceleration(show=False)

