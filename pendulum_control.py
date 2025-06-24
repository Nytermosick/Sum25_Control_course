import numpy as np
from simulator import Simulator, ActuatorMotor



def wrap_to_pi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    q = wrap_to_pi(q)

    u = 0

    return u

def main():
    torque_control = ActuatorMotor()
    
    sim = Simulator(
        xml_path="simple_pendulum.xml",
        actuator= torque_control,
        fps=30,
        record_video=False
    )

    sim.set_controller(controller)
    sim.run(time_steps=40.0, q0=[0], dq0=[0.01])

    # sim.plot_results()

if __name__ == "__main__":
    main()