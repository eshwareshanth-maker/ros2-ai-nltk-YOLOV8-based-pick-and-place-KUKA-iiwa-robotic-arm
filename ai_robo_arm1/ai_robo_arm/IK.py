#=================================================================
#Author : Eshanth Eshwar M
#email : eshwareshanth@gmail.com
#=================================================================


import numpy as np

import rclpy
from rclpy.node import Node

import time

from trajectory_msgs.msg import JointTrajectory
from trajectory_msgs.msg import JointTrajectoryPoint
from sensor_msgs.msg import JointState

from std_msgs.msg import Float32MultiArray





D1, D3, D5, D7 = 0.340, 0.400, 0.400, 0.126
ALPHA = [0, -np.pi/2, np.pi/2, np.pi/2, -np.pi/2, -np.pi/2, np.pi/2]
A     = [0, 0, 0, 0, 0, 0, 0]
D     = [D1, 0, D3, 0, D5, 0, D7]

JOINT_LIMITS_DEG = [170, 120, 170, 120, 170, 120, 175]  # 
CAM_ROLL_DEG = 0.0

def _cam_roll_matrix(deg):
    r = np.radians(deg)
    c, s = np.cos(r), np.sin(r)
   
    return np.array([
        [c, -s, 0],
        [s,  c, 0],
        [0,  0, 1]
    ])

R_cam_ee = _cam_roll_matrix(CAM_ROLL_DEG)


CAMERA_OFFSET = np.array([0.0, 0.0, 0.0])


def dh_transform(alpha, a, d, theta):
    ca, sa = np.cos(alpha), np.sin(alpha)
    ct, st = np.cos(theta), np.sin(theta)
    return np.array([
        [ct,      -st,       0,      a],
        [st*ca,   ct*ca,    -sa,   -sa*d],
        [st*sa,   ct*sa,     ca,    ca*d],
        [0,        0,        0,      1]
    ])


def forward_kinematics(theta):
    
    T = np.eye(4)
    origins = [T[:3, 3].copy()]
    z_axes  = [T[:3, 2].copy()]
    for i in range(7):
        T = T @ dh_transform(ALPHA[i], A[i], D[i], theta[i])
        origins.append(T[:3, 3].copy())
        z_axes.append(T[:3, 2].copy())
    end_effector_pos = T[:3, 3]
    end_effector_rot = T[:3, :3]
    return end_effector_pos, origins, z_axes, end_effector_rot


def position_jacobian(theta):
    p_end, origins, z_axes, _ = forward_kinematics(theta)
    J = np.zeros((3, 7))
    for i in range(7):
        J[:, i] = np.cross(z_axes[i], p_end - origins[i])
    return J, p_end


def inverse_kinematics_position(target_xyz, theta_current, max_iters=500,
                                 tol=1e-4, damping=0.05, step_scale=1.0):
    
    theta = np.array(theta_current, dtype=float).copy()
    target = np.array(target_xyz, dtype=float)
    limits = np.radians(JOINT_LIMITS_DEG)

    for _ in range(max_iters):
        J, p_end = position_jacobian(theta)
        error = target - p_end
        if np.linalg.norm(error) < tol:
            break
        JJt = J @ J.T
        dls = J.T @ np.linalg.inv(JJt + (damping**2) * np.eye(3))
        d_theta = step_scale * (dls @ error)
        theta = theta + d_theta
        theta = np.clip(theta, -limits, limits)

    final_pos, _, _, _ = forward_kinematics(theta)
    return theta, final_pos, np.linalg.norm(target - final_pos)

    

rclpy.init()
node = rclpy.create_node('list_subscriber')
    
current_positions = [0.0] * 7

received = False

def listener_callback(msg):
    global current_positions, received
    current_positions = list(msg.position)
    received = True

received2 = False
target = [0.0, 0.0, 0.0]
def listener_callback2(msg1):
    print(msg1.data)
    global target, received2
    target = list(msg1.data)
    received2 = True    
    



if __name__ == "__main__":

    node.create_subscription(JointState,
            '/joint_states',
            listener_callback,
            10)

    node.create_subscription(Float32MultiArray,
                '/mesure',
                listener_callback2,
                10)
    
    pub = node.create_publisher(Float32MultiArray, '/gesture', 10)

    gesture_msg = Float32MultiArray()
    base_msg = Float32MultiArray()


    node.get_logger().info('Waiting for /joint_states...')
    while not received:
        rclpy.spin_once(node, timeout_sec=1.0)

    while not received2:
        rclpy.spin_once(node, timeout_sec=1.0)

    theta_current = current_positions
    print(target)
    
    X = target[0]
    Y = target[1]
    Z = target[2]

    print("Enter target end-effector position:")
    x = X
    y = Y
    z = Z

    base = [0.0, -0.7854, 0.0, 1.3962, 0.0, 0.6109, 0.0]

   
    p_ee, _, _, R_ee = forward_kinematics(theta_current)


    target_camera = np.array([x, y, z])


    camera_origin = p_ee + R_ee @ CAMERA_OFFSET


    target_base = camera_origin + R_ee @ (R_cam_ee @ target_camera)

    print("\nEnter current joint radians (θ1 to θ7):")
    print(theta_current)
    print(target)
    print("\nTarget in base frame:", target_base)

    solved_theta, final_pos, err = inverse_kinematics_position(
        target_base, theta_current
    )

    print("\nSolved joint angles (radians):")
    for i, t in enumerate(solved_theta, start=1):
        print(f"  θ{i}: {t: .6f} rad")

    print(f"\nResulting end-effector position: {final_pos}")
    print(f"Position error: {err:.6f} m")

   
    delta = solved_theta - np.array(theta_current)
    print("\nJoint displacement (radians) from current pose:")
    target_rad = []
    for i, d in enumerate(delta, start=1):
        print(f"  Δθ{i}: {d: .6f} rad  ({np.degrees(d): .2f}°)")
        target_rad.append(d)
    print(f"\nMax single-joint displacement: {np.degrees(np.max(np.abs(delta))):.2f}°")
    print(target_rad)

    gesture_msg.data = [float(t) for t in target_rad]
    base_msg.data = [float(b) for b in base]
    
    pub.publish(gesture_msg)
    
    target_tolerance = 0.02  # radians (~1.1 degrees)

    

    current = np.array(current_positions)
    target = np.array(solved_theta)

    joint_error = np.max(np.abs(current - target))

    print(f"Joint Error = {joint_error:.5f} rad")

    if joint_error < target_tolerance:
        print("Robot reached the target.")
        
    for i in range(20):
        print("reached target", i)
        

    
    base_msg.data = [float(b) for b in base]
    time.sleep(10)
    pub.publish(base_msg)
    node.get_logger().info("Returning to base pose.")

    
for _ in range(20):
    rclpy.spin_once(node, timeout_sec=0.1)

node.destroy_node()
rclpy.shutdown()
