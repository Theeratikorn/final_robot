### TEST VERSION 2 ###
import numpy as np
import matplotlib.pyplot as plt
from config import robot_params
from mpl_toolkits.mplot3d import Axes3D  # นำเข้าเพื่อให้แน่ใจว่าใช้ 3D plotting ได้

def calculate_thetas(dx, dy, dz, phi):
    # Retrieve link lengths from robot_params
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

        # Two possible theta2 solutions
        theta2_1 = np.arctan2(C, np.sqrt(r**2 - C**2)) - np.arctan2(a, b)
        theta2_2 = np.arctan2(C, -np.sqrt(r**2 - C**2)) - np.arctan2(a, b)

        # theta2_1 = np.arctan2(C, np.sqrt(r**2 - dz**2)) - np.arctan2(a, b)  # change C (uppercase) to dz
        # theta2_2 = np.arctan2(C, -np.sqrt(r**2 - dz**2)) - np.arctan2(a, b) # change C (uppercase) to dz

        for theta2 in [theta2_1, theta2_2]:
            # Calculate theta4 for each theta2 and theta3
            theta4 = phi - theta2 - theta3
            solutions.append((theta1, theta2, theta3, theta4))

    return solutions

def dh_transform(a, d, alpha, theta):
    return np.array([
        [np.cos(theta), -np.sin(theta) * np.cos(alpha), np.sin(theta) * np.sin(alpha), a * np.cos(theta)],
        [np.sin(theta), np.cos(theta) * np.cos(alpha), -np.cos(theta) * np.sin(alpha), a * np.sin(theta)],
        [0, np.sin(alpha), np.cos(alpha), d],
        [0, 0, 0, 1]
    ])

def forward_kinematics(theta1, theta2, theta3, theta4):
    l1 = robot_params.l1
    l2 = robot_params.l2
    l3 = robot_params.l3
    l4 = robot_params.l4

    T1 = dh_transform(0, l1, np.pi / 2, theta1)
    T2 = dh_transform(l2, 0, 0, theta2)
    T3 = dh_transform(l3, 0, 0, theta3)
    T4 = dh_transform(l4, 0, 0, theta4)

    T = T1 @ T2 @ T3 @ T4
    x, y, z = T[0, 3], T[1, 3], T[2, 3]

    return x, y, z, T1, T2, T3, T4

def safety_calulate_theta(dx, dy, dz, phi, z_thrshold_check=False, z_threshold=0):
    jL_th1 = robot_params.jointLimitL_th1
    jR_th1 = robot_params.jointLimitR_th1

    jL_th2 = robot_params.jointLimitL_th2
    jR_th2 = robot_params.jointLimitR_th2

    jL_th3 = robot_params.jointLimitL_th3
    jR_th3 = robot_params.jointLimitR_th3

    jL_th4 = robot_params.jointLimitL_th4
    jR_th4 = robot_params.jointLimitR_th4

    state_check = False
    thetas = []
    theta1 = theta2 = theta3 = theta4 = None

    try:
        solutions = calculate_thetas(dx, dy, dz, phi)
        for solution in solutions:
            t1, t2, t3, t4 = solution
            if (jL_th1 < t1 < jR_th1 and jL_th2 < t2 < jR_th2 and
                jL_th3 < t3 < jR_th3 and jL_th4 < t4 < jR_th4):
                state_check = True
                theta1, theta2, theta3, theta4 = t1, t2, t3, t4
                print(f'theta from safety_calculate theta {theta1, theta2, theta3, theta4}')
                
                #_ , _ , z = joints_plot(np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4))
                _ , _ , z = joints_plot(theta1, theta2, theta3, theta4)

                if z_thrshold_check:
                    if np.any(np.array(z[1:]) < 0):
                        print(z[1:])
                        print("Hit the floor")
                        state_check = False
                        print(z)
                print(f'before break {z}')
                break

    except ValueError:
        print("No valid solutions found for the given position.")

    if not state_check:
        print("No solutions within joint limits or safety constraints.")

    return state_check, theta1, theta2, theta3, theta4

def movej(dx, dy, dz, phi=0, z_thrshold_check=False, z_threshold=0):
    state_check, theta1, theta2, theta3, theta4 = safety_calulate_theta(dx, dy, dz, phi, z_thrshold_check, z_threshold)
    if state_check:
        return (np.rad2deg(theta1), np.rad2deg(theta2), np.rad2deg(theta3), np.rad2deg(theta4))
    else:
        return (None, None, None, None)

def joints_plot(theta1, theta2, theta3, theta4, show_obj=False):
    print(f'input th into joint_plot {theta1, theta2, theta3, theta4}')
    dx, dy, dz, T1, T2, T3, T4 = forward_kinematics(theta1, theta2, theta3, theta4)
    T02 = T1 @ T2
    T03 = T02 @ T3

    x = [0, T1[0, 3], T02[0, 3], T03[0, 3], dx]
    y = [0, T1[1, 3], T02[1, 3], T03[1, 3], dy]
    z = [0, T1[2, 3], T02[2, 3], T03[2, 3], dz]

    print(f'x position = {x}')
    print(f'y position = {y}')
    print(f'z position = {z}')

    return x, y, z

def movestep_theta(current_theta, goal_theta, step=20):
    step_th1 = np.linspace(current_theta[0], goal_theta[0], step)
    step_th2 = np.linspace(current_theta[1], goal_theta[1], step)
    step_th3 = np.linspace(current_theta[2], goal_theta[2], step)
    step_th4 = np.linspace(current_theta[3], goal_theta[3], step)

    return step_th1, step_th2, step_th3, step_th4
