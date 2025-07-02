import numpy as np
from simulator import Simulator
from pathlib import Path
from typing import Dict
import pinocchio as pin
import os
import scipy
import matplotlib.pyplot as plt

####################
trajectory = True # if True manipulator will move along a circular trajectory
cube = False # if True manipulator wiil move move after the cube
plots = True # if True plots will be drawn
####################

# Load the robot model from scene XML
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "universal_robots_ur5e/ur5e.xml")
model = pin.buildModelFromMJCF(xml_path)
data = model.createData()

def so3_error(Rd, R):
    """Compute orientation error using matrix logarithm"""
    error_matrix = Rd @ R.T
    error_log = scipy.linalg.logm(error_matrix)
    error_vector = skew_to_vector(error_log)
    return error_vector

def skew_to_vector(skew_matrix):
    """Extract the vector from a skew-symmetric matrix"""
    return np.array([skew_matrix[2, 1], skew_matrix[0, 2], skew_matrix[1, 0]])

def circular_trajectory(t, center, radius, omega):
    """
    Set circular trajectory.
    t: time
    center: centre of circle (x_c, y_c, z_c)
    radius: radius
    omega: angular speed
    """
    x_c, y_c, z_c = center

    x = x_c + radius * np.cos(omega * t)
    y = y_c
    z = z_c + radius * np.sin(omega * t)

    vx = -radius * omega * np.sin(omega * t)
    vy = 0
    vz = radius * omega * np.cos(omega * t)

    ax = -radius * omega**2 * np.cos(omega * t)
    ay = 0
    az = -radius * omega**2 * np.sin(omega * t)
    
    position = np.array([x, y, z]) # conversion to a column vector
    velocity = np.array([vx, vy, vz, 0, 0, 0]) # conversion to a column vector
    acceleration = np.array([ax, ay, az, 0, 0, 0]) # conversion to a column vector
    
    return position, velocity, acceleration


def task_space_controller(q: np.ndarray, dq: np.ndarray, t: float, desired: Dict) -> np.ndarray:
    """Example task space controller."""

    if trajectory:
        # Parameters of circle
        center = [0.3, 0.3, 0.4]  # Centre of circle
        radius = 0.1              # Radius
        omega = 2 * np.pi / 1     # Angular speed (1 round per 1 seconds)

        p_des, dp_des, ddp_des = circular_trajectory(t, center, radius, omega)

        # Euler angles (roll, pitch, yaw) in grad
        roll = np.deg2rad(0)
        pitch = np.deg2rad(0)
        yaw = np.deg2rad(0)

        # Creating a rotation matrix
        R_des = pin.utils.rpyToMatrix(roll, pitch, yaw)

    else:
        if cube:
            p_des = desired['pos'] # desired coordinates x, y, z

            desired_quaternion = desired['quat'] # [w, x, y, z] in MuJoCo format

            # Convert desired pose to SE3
            desired_quaternion_pin = np.array([*desired_quaternion[1:], desired_quaternion[0]]) # Convert to [x,y,z,w] for Pinocchio
            desired_pose = np.concatenate([p_des, desired_quaternion_pin])

            # Creating a rotation matrix
            R_des = pin.XYZQUATToSE3(desired_pose).rotation

        else:
            # Desired values
            p_des = np.array([0.2, 0.35, 0.5])

            # Euler angles (roll, pitch, yaw) in grad
            roll = np.deg2rad(0)
            pitch = np.deg2rad(0)
            yaw = np.deg2rad(0)

            # Creating a rotation matrix
            R_des = pin.utils.rpyToMatrix(roll, pitch, yaw)


        dp_des = np.zeros(6)
        ddp_des = np.zeros(6)


    # PD Gains
    Kp = np.diag([100, 100, 100, 100, 100, 100]) # Proportional coefficients

    Kd = np.diag([20, 20, 20, 20, 20, 20]) # Derriative coefficients

    ###################################################################################################
    
    # Compute all dynamics quantities at once
    pin.computeAllTerms(model, data, q, dq)

    # Compute forward kinematics
    #pin.forwardKinematics(model, data, q, dq)

    # Get end-effector frame ID
    ee_frame_id = model.getFrameId("end_effector")
    frame = pin.LOCAL

    # Calculate kinematics of frames
    pin.updateFramePlacement(model, data, ee_frame_id)

    # Get current position and orientation of end-effector
    ee_pose = data.oMf[ee_frame_id]
    p = ee_pose.translation # conversion to a column vector
    #print('pos\n', p)
    R = ee_pose.rotation
    #print('orient\n', R)

    # Get current velocities of end-effector
    twist = pin.getFrameVelocity(model, data, ee_frame_id, frame)
    v = twist.linear # conversion to a column vector
    w = twist.angular # conversion to a column vector
    dp = np.hstack((v, w)) # column vector of current speed in local coordinate frame

    desired_twist = ee_pose.actInv(pin.Motion(dp_des)) # conversion desired speed vector to local coordinate frame
    dp_des_local = np.hstack([desired_twist.linear, desired_twist.angular]) # column vector of desired speed in local coordinate frame


    # Get Mass Matrix and Nonlinear effects (Coriolis + gravity) Matrix
    M = data.M # Mass Matrix

    h = data.nle # Nonlinear effects Matrix

    J = pin.getFrameJacobian(model, data, ee_frame_id, frame) # Jacobian

    pin.computeJointJacobiansTimeVariation(model, data, q, dq)
    dJ = pin.getFrameJacobianTimeVariation(model, data, ee_frame_id, frame) # Derriative of Jacobian

    # Conversion desired accelerations of end-effector in local coordinate frame
    desired_acc = ee_pose.actInv(pin.Motion(ddp_des[:3], ddp_des[3:]))
    ddp_des_local = np.hstack([desired_acc.linear, desired_acc.angular]) # column vector of desired accelerations in local coordinate frame

    # Evaluating errors
    p_err = p_des - p
    print('err pos\n', p_err)

    rot_err = so3_error(R_des, R)
    print('err orient\n', rot_err)

    pose_err = np.hstack((p_err, rot_err)) # total column-vector of errors by position and orientation

    dp_err = dp_des_local - dp # column-vector of errors by speed
    print('speed err\n', dp_err, '\n')

    # Control
    ddp = Kp @ pose_err + Kd @ dp_err + ddp_des_local

    ddq_des = np.linalg.pinv(J) @ (ddp - dJ @ dq)

    tau = M @ ddq_des + h # column-vector of torques on joints
    return tau

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    Path("logs/plots").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning task space controller...")
    sim = Simulator(
        xml_path="universal_robots_ur5e/scene.xml",
        enable_task_space=True,
        show_viewer=True,
        record_video=True,
        video_path="logs/videos/task_space.mp4",
        fps=30,
        width=1920,
        height=1080
    )
    sim.set_controller(task_space_controller)

    sim.run(time_limit=5.0)

    sim.plot_results()

if __name__ == "__main__":
    main() 