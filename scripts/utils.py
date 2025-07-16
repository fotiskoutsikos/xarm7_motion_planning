import numpy as np
from kinematics import xArm7_kinematics

def compute_distance_to_cylinder(joint_position, obstacle_position, radius, height):
    """
    Computes the shortest distance from a joint to the surface of a cylindrical obstacle.
    
    Args:
        joint_position (np.array): The 3D coordinates of the robot joint [x, y, z].
        obstacle_position (np.array): The 3D coordinates of the obstacle's center [x, y, z].
        radius (float): The radius of the cylindrical obstacle.
        height (float): The height of the cylindrical obstacle.
        
    Returns:
        float: The shortest distance from the joint to the surface of the obstacle.
    """
    # Vector from the joint to the center of the obstacle
    joint_to_obstacle = joint_position - obstacle_position
    # Distance in the x-y plane (ignoring height for now)
    distance_xy = np.sqrt(joint_to_obstacle[0]**2 + joint_to_obstacle[1]**2)
    
    # If the joint is within the cylinder's height range
    if obstacle_position[2] - height/2 <= joint_position[2] <= obstacle_position[2] + height/2:
        # The distance from the joint to the cylinder surface is just the difference
        # between the distance in the x-y plane and the radius
        return max(0, distance_xy - radius)
    else:
        # If the joint is outside the cylinder's height range, return the 3D Euclidean distance
        return np.linalg.norm(joint_position - obstacle_position)

def find_joint_positions(joint_angles):
    kinematics = xArm7_kinematics()
    positions = []
    for angles in joint_angles:
        positions.append(kinematics.get_p(angles))
    return np.array(positions)

def line_to_line_distance(point00, point01, point10, point11, segment_size=5):
    x1 = np.linspace(0, 1, segment_size)
    x2 = np.linspace(0, 1, segment_size)
    distance = np.linalg.norm(point00-point10)
    for line1_scalar in x1:
        for line2_scalar in x2:
            point1 = line1_scalar*(point01-point00)+point00
            point2 = line2_scalar*(point11-point10)+point10
            distance = min(distance, np.linalg.norm(point1-point2))
            
    return distance

def cylinder_to_cylinder_distance(point00, point01, radius0, point10, point11, radius1):
    d = line_to_line_distance(point00, point01,point10, point11)
    return max(0, d-radius0-radius1)

def robot_to_obstacle_distance(joint_angles, obstacle1_pos, obstacle2_pos):
    # assuming obstacle height = 0.8, radius = 0.05
    # assuming joint link radius = 0.126 / 2
    # assuming obstacles stand upright
    kinematics = xArm7_kinematics()
    obstacle_height = 0.8
    obstacle_radius = 0.05
    link_radius = 0.126 / 2
    joint_positions = kinematics.get_p(joint_angles)
    obstacle1_pos1 = obstacle1_pos + obstacle_height*np.array([0, 0, -0.5])
    obstacle1_pos2 = obstacle1_pos + obstacle_height*np.array([0, 0, 0.5])
    obstacle2_pos1 = obstacle2_pos + obstacle_height*np.array([0, 0, -0.5])
    obstacle2_pos2 = obstacle2_pos + obstacle_height*np.array([0, 0, 0.5])
    distance = 10**6
    for i in range(len(joint_positions)-1):
        pos1 = joint_positions[i]
        pos2 = joint_positions[i+1]
        d1 = cylinder_to_cylinder_distance(pos1, pos2, link_radius, obstacle1_pos1, obstacle1_pos2, obstacle_radius)
        d2 = cylinder_to_cylinder_distance(pos1, pos2, link_radius, obstacle2_pos1, obstacle2_pos2, obstacle_radius)
        d = min(d1, d2)
        distance = min(d, distance)

    return distance

def derivative_of_distance_to_obstacle(joint_angles, obstacle1_pos, obstacle2_pos, dqi = 0.01):
    current_distance = robot_to_obstacle_distance(joint_angles, obstacle1_pos, obstacle2_pos)
    derivative = np.zeros_like(joint_angles)
    for i in range(len(joint_angles)):
        new_angles = joint_angles.copy()
        new_angles[i] += dqi
        derivative[i] = (robot_to_obstacle_distance(new_angles, obstacle1_pos, obstacle2_pos) - current_distance) / dqi
    return derivative

def real_modulo(x, y):
    p = np.floor(x / y)
    return x - p * y

def f(t, T):
    tau = (t%T) / T  # scaled time in [0, 1]
    val = 10*tau**3 - 15*tau**4 + 6*tau**5
    # Alternate direction every full period (i.e. each 2T)
    if int(t // T) % 2 == 1:
        val = 1 - val
    return val

def df(t, T):
    tau = (t%T) / T  # scaled time in [0, 1]
    val = (30*tau**2 - 60*tau**3 + 30*tau**4) / T
    # Alternate direction every full period (i.e. each 2T)
    if int(t // T) % 2 == 1:
        val = -1 * val
    return val

def x_desired(t, P_A, P_B, T):
    return f(t, T) * (P_B - P_A) + P_A

def derivative_of_x_desired(t, P_A, P_B, T):
    return df(t, T) * (P_B - P_A)
