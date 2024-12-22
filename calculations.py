import numpy as np
import matplotlib.pyplot as plt
from config import robot_params
from mpl_toolkits.mplot3d import Axes3D  # นำเข้าเพื่อให้แน่ใจว่าใช้ 3D plotting ได้
################################## calculation final ############################


# # Function to calculate inverse kinematics angles
# def calculate_thetas(dx, dy, dz, phi):
#     # Retrieve link lengths from robot_params
#     l1 = robot_params.l1
#     l2 = robot_params.l2
#     l3 = robot_params.l3
#     l4 = robot_params.l4

#     # Calculate theta1 using equation (13)
#     theta1 = np.arctan2(dy, dx)

#     # Compute values A, B, C for further calculations
#     A = dx - l4 * np.cos(theta1) * np.cos(phi)
#     B = dy - l4 * np.sin(theta1) * np.cos(phi)
#     C = dz - l1 - l4 * np.sin(phi)

#     # Calculate theta3 using equation (15)
#     theta3 = np.arccos((A**2 + B**2 + C**2 - l2**2 - l3**2) / (2 * l2 * l3))

#     # Calculate values a, b, c for theta2 calculation
#     a = l3 * np.sin(theta3)
#     b = l2 + l3 * np.cos(theta3)
#     c = dz - l1 - l4 * np.sin(phi)
#     r = np.sqrt(a**2 + b**2)

#     # Calculate theta2 using equation (14)
#     theta2 = np.arctan2(c, np.sqrt(r**2 - c**2)) - np.arctan2(a, b)

#     # Calculate theta4 using equation (16)
#     theta4 = phi - theta2 - theta3

#     # # Convert angles from radians to degrees
#     # theta1_deg = np.rad2deg(theta1)
#     # theta2_deg = np.rad2deg(theta2)
#     # theta3_deg = np.rad2deg(theta3)
#     # theta4_deg = np.rad2deg(theta4)

#     # Return all calculated angles in both radians and degrees
#     return theta1, theta2, theta3, theta4   #, theta1_deg, theta2_deg, theta3_deg, theta4_deg
# # Define the transformation matrix using DH parameters

# def calculate_thetas(dx, dy, dz, phi):
#     # Retrieve link lengths from robot_params
#     l1 = robot_params.l1
#     l2 = robot_params.l2
#     l3 = robot_params.l3
#     l4 = robot_params.l4

#     # Step 1: Calculate theta1
#     theta1 = np.arctan2(dy, dx)

#     # Step 2: Calculate intermediate variables A, B, C
#     A = dx - l4 * np.cos(theta1) * np.cos(phi)
#     B = dy - l4 * np.sin(theta1) * np.cos(phi)
#     C = dz - l1 - l4 * np.sin(phi)

#     # Step 3: Calculate theta3 (two solutions)
#     cos_theta3 = (A**2 + B**2 + C**2 - l2**2 - l3**2) / (2 * l2 * l3)
#     if np.abs(cos_theta3) > 1:
#         raise ValueError("Target position is out of reach")

#     theta3_1 = np.arccos(cos_theta3)
#     theta3_2 = -theta3_1  # Second solution

#     # Step 4: Calculate theta2 and theta4 for both theta3 solutions
#     solutions = []
#     for theta3 in [theta3_1, theta3_2]:
#         a = l3 * np.sin(theta3)
#         b = l2 + l3 * np.cos(theta3)
#         r = np.sqrt(a**2 + b**2)

#         # Two possible theta2 solutions
#         theta2_1 = np.arctan2(C, np.sqrt(r**2 - C**2)) - np.arctan2(a, b)
#         theta2_2 = np.arctan2(C, -np.sqrt(r**2 - C**2)) - np.arctan2(a, b)

#         for theta2 in [theta2_1, theta2_2]:
#             # Calculate theta4 for each theta2 and theta3
#             theta4 = phi - theta2 - theta3
#             solutions.append((theta1, theta2, theta3, theta4))

#         print(f'solution {solutions}')    
        
#         return solutions


def calculate_thetas(dx, dy, dz, phi):
    l1 = robot_params.l1
    l2 = robot_params.l2
    l3 = robot_params.l3
    l4 = robot_params.l4

    # Step 1: Calculate theta1
    theta1 = np.arctan2(dy, dx)

    # Step 2: Calculate intermediate variables A, B, C
    A = dx - l4 * np.cos(theta1) * np.cos(phi)
    B = dy - l4 * np.sin(theta1) * np.cos(phi)
    C = dz - l1 - l4 * np.sin(phi)

    # Step 3: Calculate theta3 (two solutions)
    cos_theta3 = (A**2 + B**2 + C**2 - l2**2 - l3**2) / (2 * l2 * l3)
    if np.abs(cos_theta3) > 1:
        raise ValueError("Target position is out of reach")

    theta3_1 = np.arccos(cos_theta3)
    theta3_2 = -theta3_1  # Second solution

    # Step 4: Calculate theta2 and theta4 for both theta3 solutions
    solutions = []
    for theta3 in [theta3_1, theta3_2]:
        a = l3 * np.sin(theta3)
        b = l2 + l3 * np.cos(theta3)
        r = np.sqrt(a**2 + b**2)

        discriminant = r**2 - C**2
        if discriminant < 0:
            continue  # Skip invalid solutions

        # Two possible theta2 solutions
        theta2_1 = np.arctan2(C, np.sqrt(discriminant)) - np.arctan2(a, b)
        theta2_2 = np.arctan2(C, -np.sqrt(discriminant)) - np.arctan2(a, b)

        for theta2 in [theta2_1, theta2_2]:
            # Calculate theta4 for each theta2 and theta3
            theta4 = phi - theta2 - theta3
            solutions.append((theta1, theta2, theta3, theta4))

    # Return all 4 solutions (or fewer if some are invalid)
    return solutions


def dh_transform(a, d, alpha, theta):
    """
    Creates a Denavit-Hartenberg (DH) transformation matrix.

    Parameters:
    - a (float): Link length
    - d (float): Link offset
    - alpha (float): Twist angle in radians
    - theta (float): Joint angle in radians

    Returns:
    - numpy.ndarray: A 4x4 transformation matrix
    """
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

# Updated forward kinematics function
def forward_kinematics(theta1, theta2, theta3, theta4):
    """
    Calculates the forward kinematics to determine the end effector's position.

    Parameters:
    - theta1 (float): Joint angle for joint 1 in rad
    - theta2 (float): Joint angle for joint 2 in rad
    - theta3 (float): Joint angle for joint 3 in rad
    - theta4 (float): Joint angle for joint 4 in rad

    Returns:
    - tuple: (x, y, z) coordinates of the end effector
    """
    # Retrieve link lengths from robot_params
    l1 = robot_params.l1
    l2 = robot_params.l2
    l3 = robot_params.l3
    l4 = robot_params.l4

    # test
    # print(l1,l2,l3,l4)

    
    theta1_rad = theta1
    theta2_rad = theta2
    theta3_rad = theta3
    theta4_rad = theta4

    # Define the DH parameters for each joint
    # alpha is the twist angle, a is the link length, d is the offset, and theta is the joint angle
    T1 = dh_transform(0, l1, np.pi / 2, theta1_rad)  # Base to joint 1
    T2 = dh_transform(l2, 0, 0, theta2_rad)          # Joint 1 to joint 2
    T3 = dh_transform(l3, 0, 0, theta3_rad)          # Joint 2 to joint 3
    T4 = dh_transform(l4, 0, 0, theta4_rad)          # Joint 3 to end effector

    # Combine all transformation matrices
    T = T1 @ T2 @ T3 @ T4

    # Extract the position of the end effector from the final transformation matrix
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    return x, y, z , T1, T2, T3, T4 

def safety_calulate_theta(dx, dy, dz, phi, z_thrshold_check=False, z_threshold=0):

    dx = dx
    dy = dy
    dz = dz

    jL_th1 = robot_params.jointLimitL_th1
    jR_th1 = robot_params.jointLimitR_th1

    jL_th2 = robot_params.jointLimitL_th2
    jR_th2 = robot_params.jointLimitR_th2

    jL_th3 = robot_params.jointLimitL_th3
    jR_th3 = robot_params.jointLimitR_th3

    jL_th4 = robot_params.jointLimitL_th4
    jR_th4 = robot_params.jointLimitR_th4

    pos_check = []
    state_check = False
    thetas_positive_theta2 = []
    thetas_other = []

    # Calculate all possible solutions
    solutions = calculate_thetas(dx, dy, dz, phi)

    # Separate solutions into theta2 positive and others
    for solution in solutions:
        if solution[1] > 0:  # theta2 > 0
            thetas_positive_theta2.append(solution)
        else:
            thetas_other.append(solution)

    # Prioritize theta2 positive
    prioritized_thetas = thetas_positive_theta2 + thetas_other
    print(f'prioritized_thetas {prioritized_thetas}')

    for thetas in prioritized_thetas:
        theta1, theta2, theta3, theta4 = thetas
        print(f"Checking theta set: {np.rad2deg(theta1)}, {np.rad2deg(theta2)}, {np.rad2deg(theta3)}, {np.rad2deg(theta4)}")

        angles = [theta1, theta2, theta3, theta4]
        is_not_null_check = None not in angles and not np.any(np.isnan(angles))

        in_joint_limit = (
            jL_th1 < theta1 < jR_th1 and
            jL_th2 < theta2 < jR_th2 and
            jL_th3 < theta3 < jR_th3 and
            jL_th4 < theta4 < jR_th4
        )

        state_check = is_not_null_check and in_joint_limit
        if not in_joint_limit:
            print("Joint limit problem")
        if not is_not_null_check:
            print("Singularity Problem")

        if z_thrshold_check:
            __, __, __, T1, T2, T3, T4 = forward_kinematics(theta1, theta2, theta3, theta4)
            pos_check = [dz, T1[2, 3], (T1 @ T2)[2, 3], (T1 @ T2 @ T3)[2, 3]]

            if np.all(np.array(pos_check) > z_threshold) and state_check:
                state_check = True
                break
            elif np.all(np.array(pos_check) < z_threshold):
                print("Hit the floor")
                state_check = False
            else:
                state_check = False
        elif state_check and not z_thrshold_check:
            break

    return state_check, theta1, theta2, theta3, theta4

def movej(dx,dy,dz,phi = 0, z_thrshold_check = False , z_threshold = 0):

    state_check, theta1, theta2, theta3, theta4 = safety_calulate_theta(dx, dy, dz, phi, z_thrshold_check, z_threshold)

    if state_check:
        return np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4)
    elif not state_check:
        return (None,None,None,None)

def joints_plot(theta1, theta2, theta3, theta4, show_obj = False):

    dx, dy, dz, T1, T2, T3, T4 = forward_kinematics(theta1, theta2, theta3, theta4)
    T02 = T1@T2  # frame 2 ref 0
    T03 = T02@T3 # frame 3 ref 0

    # get positions from joints
    x = [0, T1[0,3] , T02[0,3], T03[0,3], dx]
    y = [0, T1[1,3] , T02[1,3], T03[1,3], dy]
    z = [0, T1[2,3] , T02[2,3], T03[2,3], dz]

    print(f'x position = {x}')
    print(f'y position = {y}')
    print(f'z position = {z}')

    return x, y, z


def movestep_theta(current_theta,goal_theta,step = 20):

    step_th1 = np.linspace(current_theta[0],goal_theta[0],step)
    step_th2 = np.linspace(current_theta[1],goal_theta[1],step)
    step_th3 = np.linspace(current_theta[2],goal_theta[2],step)
    step_th4 = np.linspace(current_theta[3],goal_theta[3],step)

    return step_th1,step_th2,step_th3,step_th4
