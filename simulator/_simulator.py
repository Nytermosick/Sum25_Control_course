import numpy as np
from pathlib import Path
import os
import pinocchio as pin
import matplotlib.pyplot as plt
import mujoco
import mediapy as media
from typing import Callable, Optional, Dict, Union, List, Any
import time
from datetime import datetime
import mujoco.viewer

class ActuatorMotor:
    def __init__(self, torque_range = [-100,100]) -> None:
        self.range = torque_range
        self.dyn = np.array([1, 0, 0])
        self.gain = np.array([1, 0, 0])
        self.bias = np.array([0, 0, 0])

    def __repr__(self) -> str:
        return f"ActuatorMotor(dyn={self.dyn}, gain={self.gain}, bias={self.bias})"
    
class Simulator:
    def __init__(self,
                 xml_path: str,
                 actuator: ActuatorMotor = ActuatorMotor(),
                 dt: float = 0.002,
                 fps: int = 30,
                 width: int = 1920,
                 height: int = 1080,
                 record_video: bool = True,
                 video_path: str = None) -> None:
        
        # Load the model
        self.xml_path = xml_path

        self.model = mujoco.MjModel.from_xml_path(self.xml_path)
        self.data = mujoco.MjData(self.model)

        self.actuator = actuator
        self._set_actuator(actuator)

        self.controller = None

        self.model.opt.timestep = dt
        self.dt = dt

        #Video parameters
        self.log_path = None

        self.width = width
        self.height = height
        self.fps = fps


        # Setup renderer
        self.renderer = mujoco.Renderer(self.model, width=self.width, height = self.height)

        self.record_video = record_video

        # Path for logs
        if self.record_video:
            self.make_log_path()

        self.frames: List[np.ndarray] = []

        if video_path is None:
            video_path = f"logs/{self.log_path}/simulation.mp4"
        self.video_path = Path(video_path)

        self._setup_video_recording()

    def make_log_path(self) -> None:
        self.log_path = f'{self.xml_path.split("/")[-1]}/{datetime.now().strftime("%d_%m_%H_%M_%S")}'
        os.makedirs(f'logs/{self.log_path}', exist_ok=True)

    def _setup_video_recording(self) -> None:
        """Setup video recording directory if enabled."""
        if self.record_video:
            self.video_path.parent.mkdir(parents=True, exist_ok=True)

    
    def _save_video(self) -> None:
        """Save recorded video frames to file if any were captured."""
        if self.record_video and self.frames:
            print(f"Saving video to {self.video_path}...")
            media.write_video(str(self.video_path), self.frames, fps=self.fps)
            self.frames = []


    def _capture_frame(self) -> np.ndarray:
        """Capture a frame using the renderer.
        
        Returns:
            RGB image array of current scene
        """
        self.renderer.update_scene(self.data)
        pixels = self.renderer.render()
        return pixels.copy()


    def _set_actuator(self, actuator: str):
        for actuator_id in range(self.model.nu):
            self._update_actuator(actuator_id, self.actuator)

        
    def _update_actuator(self, actuator_id: Union[str, int], actuator: ActuatorMotor) -> None:
        """Update specific actuator in the model.
        
        Args:
            actuator_id: Actuator name or ID
            actuator: Actuator configuration object
        """
        model_actuator = self.model.actuator(actuator_id)
        model_actuator.dynprm = np.zeros(len(model_actuator.dynprm))
        model_actuator.gainprm = np.zeros(len(model_actuator.gainprm))
        model_actuator.biasprm = np.zeros(len(model_actuator.biasprm))
        model_actuator.ctrlrange = actuator.range 
        model_actuator.dynprm[:3] = actuator.dyn
        model_actuator.gainprm[:3] = actuator.gain
        model_actuator.biasprm[:3] = actuator.bias

    def reset(self) -> None:
        """Reset the simulation to the initial state."""
        mujoco.mj_resetData(self.model, self.data)


    def set_controller(self, controller: Callable) -> None:
        """Set the controller function.
        
        Args:
            controller: Controller function that takes the model data and returns the control input.
        """
        self.controller = controller


    def get_state(self) -> Dict[str, np.ndarray]:
        """Get the current state of the model.
        
        Returns:
            State vector
        """
        state = {
            'q': self.data.qpos.copy(),
            'dq': self.data.qvel.copy(),
        }

        return state
    
    def _def_init_state(self, q0: np.ndarray = None, dq0: np.ndarray = None):
        if q0:
            self.data.qpos = q0
        if dq0:
            self.data.qvel = dq0
    
    
    def step(self, tau: np.ndarray) -> None:
        """Step the simulation forward.
        
        Args:
            tau: Control input
        """
        self.data.ctrl = tau
        mujoco.mj_step(self.model, self.data)


    def run(self, time_steps: int, q0 = None, dq0 = None) -> None:
        """Run simulation with visualization and recording.
        
        Args:
            time_limit: Maximum simulation time in seconds
            
        Raises:
            AssertionError: If controller is not set
        """
        assert self.controller is not None, "Controller not set!"

        viewer = mujoco.viewer.launch_passive(
            model = self.model,
            data = self.data,
            show_left_ui = False,
            show_right_ui = False
        )

        self.reset()

        self.pos = []
        self.controls = []
        self.times = []

        if q0 or dq0:
            self._def_init_state(q0, dq0)

        mujoco.mjv_defaultFreeCamera(self.model, viewer.cam)

        try:
            t = 0
            start_time = time.perf_counter()

            while viewer.is_running():
                step = time.perf_counter()

                # Get state and compute control input
                state = self.get_state()

                tau = self.controller(
                    q = state['q'],
                    dq = state['dq'],
                    t = t
                )

                # Data for plotting
                self.pos.append(state['q'])
                self.controls.append(tau)
                self.times.append(t)

                # Step the simulation
                self.step(tau)


                # Update the viewer
                if viewer:
                    viewer.sync()

                # Record video if enabled
                if self.record_video:
                    if len(self.frames) < self.fps * t:
                        self.frames.append(self._capture_frame())

                # time keeping
                t += self.dt
                if time_steps and t >= time_steps:
                    break

                # Real-time synchronization
                real_time = time.perf_counter() - start_time
                if t > real_time:
                    time.sleep(t - real_time)
                elif real_time - t > self.dt:
                    print(f"Warning: Simulation running slower than real-time by {real_time - t:.3f}s")

        except KeyboardInterrupt:
            print("\nSimulation interrupted by user")
        finally:
            if viewer:
                viewer.close()
            self._save_video()
                

    def plot_results(self):
        """Plot and save simulation results."""

        if self.log_path is None:
            self.make_log_path()

        self.pos = np.array(self.pos)
        self.controls = np.array(self.controls)
        self.times = np.array(self.times)
        
        # Joint positions plot
        plt.figure(figsize=(10, 6))
        for i in range(self.pos.shape[1]):
            plt.plot(self.times, self.pos[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Joint Position [rad]')
        plt.title('Joint Position over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'logs/{self.log_path}/position.png')
        plt.close()

        # Joint position errors plot
        plt.figure(figsize=(10, 6))
        for i in range(self.pos.shape[1]):
            plt.plot(self.times, 0 - self.pos[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Joint Position error [rad]')
        plt.title('Joint Position error over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'logs/{self.log_path}/position_error.png')
        plt.close()


        # Joint controls plot
        if self.controls.ndim == 1:
            self.controls = self.controls.reshape(-1, 1)

        for i in range(self.controls.shape[1]):
            plt.plot(self.times, self.controls[:, i], label=f'Joint {i+1}')
        plt.xlabel('Time [s]')
        plt.ylabel('Joint control signals')
        plt.title('Joint control signals over Time')
        plt.legend()
        plt.grid(True)
        plt.savefig(f'logs/{self.log_path}/control_signals.png')
        plt.close()