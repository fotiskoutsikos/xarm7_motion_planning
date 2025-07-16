# logger.py
import csv
import os
from datetime import datetime

class SimulationLogger:
    def __init__(self, base_dir="logs"):
        # Create a folder with current date and time
        timestamp = datetime.now().strftime("%Y-%m-%d_%H-%M-%S")
        self.log_dir = os.path.join(base_dir, timestamp)
        os.makedirs(self.log_dir, exist_ok=True)

        # Data buffers
        self.time_log = []
        self.joint_angle_log = []
        self.desired_joint_angle_log = []
        self.desired_trajectory_log = []
        self.obstacle_log = []
        self.algorithm_parameters_log = []

    def log_time(self, time):
        self.time_log.append([time])  # Single-column list
    
    def log_joint_angles(self, joint_angles):
        self.joint_angle_log.append(list(joint_angles))
    
    def log_desired_joint_angles(self, desired_joint_angles):
        self.desired_joint_angle_log.append(desired_joint_angles.copy())

    def log_obstacles(self, obstacle_data):
        self.obstacle_log.append(obstacle_data.copy())

    def log_desired_trajectory(self, desired_trajectory):
        self.desired_trajectory_log.append(desired_trajectory.copy())
    
    def log_algorithm_parameters(self, half_period, K, k0):
        self.algorithm_parameters_log.append([half_period, K, k0])

    def save_to_csv(self):
        # Save time log
        with open(os.path.join(self.log_dir, "time.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['time'])
            writer.writerows(self.time_log)

        # Save joint angles
        with open(os.path.join(self.log_dir, "joint_angles.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7'])
            writer.writerows(self.joint_angle_log)
        
        with open(os.path.join(self.log_dir, "desired_joint_angles.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['j1', 'j2', 'j3', 'j4', 'j5', 'j6', 'j7'])
            writer.writerows(self.desired_joint_angle_log)
        
        with open(os.path.join(self.log_dir, "desired_trajectory.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x', 'y', 'z'])
            writer.writerows(self.desired_trajectory_log)

        # Save obstacle positions
        with open(os.path.join(self.log_dir, "obstacle_positions.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['x1', 'y1', 'z1', 'x2', 'y2', 'z2'])
            writer.writerows(self.obstacle_log)
        
        with open(os.path.join(self.log_dir, "obstacle_constants.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['radius', 'height'])
            writer.writerows([[0.05, 0.80]])
        
        with open(os.path.join(self.log_dir, "algorithm_parameters.csv"), 'w', newline='') as f:
            writer = csv.writer(f)
            writer.writerow(['half_period', 'K', 'k0'])
            writer.writerows(self.algorithm_parameters_log)
