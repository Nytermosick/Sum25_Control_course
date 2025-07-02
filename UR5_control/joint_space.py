"""Joint space control example for robot manipulation.

This example demonstrates basic joint space control of a UR5e robot arm using
PD (Proportional-Derivative) control. The robot moves from its initial configuration
to a target joint configuration while minimizing position and velocity errors.

Key Concepts Demonstrated:
    - Joint space PD control implementation
    - Real-time simulation visualization
    - Video recording of simulation
    - Basic robot state access and control

Example:
    To run this example:
    
    $ python 01_joint_space.py

Notes:
    - The controller gains (kp, kd) are tuned for the UR5e robot
    - The target configuration (q0) is set to a predefined pose
    - The simulation runs for 10 seconds with real-time visualization
"""

import numpy as np
from simulator import Simulator
from pathlib import Path
import os
import pinocchio as pin
import matplotlib.pyplot as plt

# Load the robot model from scene XML
current_dir = os.path.dirname(os.path.abspath(__file__))
xml_path = os.path.join(current_dir, "universal_robots_ur5e/ur5e.xml")
model = pin.buildModelFromMJCF(xml_path)
data = model.createData()

# Target joint configuration
q0 = ...
dq0 = ...

def joint_controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    """Joint space PD controller.
    
    Args:
        q: Current joint positions [rad]
        dq: Current joint velocities [rad/s]
        t: Current simulation time [s]
        
    Returns:
        tau: Joint torques command [Nm]
    """

    
    tau = ...

    return tau

def main():
    # Create logging directories
    Path("logs/videos").mkdir(parents=True, exist_ok=True)
    Path("logs/plots").mkdir(parents=True, exist_ok=True)
    
    print("\nRunning real-time joint space control...")
    sim = Simulator(
        xml_path="universal_robots_ur5e/scene.xml",
        record_video=True,
        video_path="logs/videos/joint_space.mp4",
        width=1920,
        height=1080
    )
    sim.set_controller(joint_controller)
    
    sim.run(time_limit=5)
    sim.plot_results()

if __name__ == "__main__":
    main() 