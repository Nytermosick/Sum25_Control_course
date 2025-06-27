import numpy as np
from simulator import Simulator, ActuatorMotor

# TODO: заполнить параметры маятника из xml файла модели
g = ...
m = ...
l = ...

# TODO: найти требуемую энергии для достижения верхней точки маятником (для energy-shaping контроллера)
Edes = ...

# Вспомогательная функция для обрезки угла
def wrap_to_pi(theta):
    return (theta + np.pi) % (2 * np.pi) - np.pi


def controller(q: np.ndarray, dq: np.ndarray, t: float) -> np.ndarray:
    theta, dtheta = q, dq
    
    #####################################################################
    # TODO: №1 Реализовать контроллер со сменой гравитации
    tau = ...

    #####################################################################
    # TODO: №2 Реализовать ПД-регулятор
    # Заполните желаемый угол и скорость
    theta_des = ...
    dtheta_des = ...

    # Выберите коэффициенты ПД-регулятора:
    Kp = ...
    Kd = ...

    tau = ...

    #####################################################################
    # TODO: №3 Реализовать energy-shaping контроллер
    theta = wrap_to_pi(theta)
    # Рассчитывайте энергию маятника на каждом шаге
    
    E = ...

    # Выберите коэффициент усиления
    k = ...

    tau = ...
    tau = np.clip(tau, -5, 5)

    #####################################################################
    # TODO: Реализуйте energy-shaping контроллер с ПД-регулятором (структура есть в HW2.md)
    ...

    tau = np.clip(tau, -5, 5)


    return tau # Выберите необходимый контроллер, а остальные закомментируйте


def main():
    torque_control = ActuatorMotor()
    
    sim = Simulator(
        xml_path="simple_pendulum.xml",
        actuator= torque_control,
        fps=30,
        record_video=True # Запись видео в папке logs/simple_pendulum/...
    )

    sim.set_controller(controller)
    sim.run(time_steps=20.0, q0=[0], dq0=[0.0]) # Задание времени моделирования и начальных условий (q0 - начальный угол маятника, dq0 - начальная скорость маятника)

    sim.plot_results() # Функция для построения графиков в папке logs/simple_pendulum/...

if __name__ == "__main__":
    main()