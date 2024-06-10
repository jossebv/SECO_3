from dataclasses import dataclass, field
from typing import Literal, Optional
from tqdm import tqdm
import os
import math
import matplotlib.pyplot as plt
import matplotlib
import scipy.signal as signal
import numpy as np
from PIDControllers import *

matplotlib.use("Agg")

K = 2652.28
p = 64.986
G = signal.lti([K], [1, p, 0])


@dataclass
class PIDConstants:
    Kps: list[float] | np.ndarray = field(default_factory=list)
    Kis: list[float] | np.ndarray = field(default_factory=list)
    Kds: list[float] | np.ndarray = field(default_factory=list)


def get_reference_signal(type: Literal["unit_step", "ramp", "parabolic"], start, end):
    if type == "unit_step":
        return np.arange(start, end, 1e-3), np.ones(int((end - start) / 1e-3))
    elif type == "ramp":
        return np.arange(start, end, 1e-3), np.arange(start, end, 1e-3)
    elif type == "parabolic":
        return np.arange(start, end, 1e-3), np.arange(start, end, 1e-3) ** 2
    else:
        raise ValueError("Invalid reference signal type")


def obtain_transfer_functions(
    controller_type: Literal["P", "P-D", "PD", "PI", "PID", "PI-D", "PID-D", "D|PID"],
    pid_constants: PIDConstants,
) -> list[Controller]:
    """
    Obtain a list of transfer functions for the given controller type and PID constants
    """

    Kps = pid_constants.Kps
    Kis = pid_constants.Kis
    Kds = pid_constants.Kds
    if controller_type == "P":
        return [PController(p=p, K=K, Kp=kp) for kp in Kps]
    elif controller_type == "P-D":
        return [P_DController(p=p, K=K, Kp=kp, tauD=kd) for kp, kd in zip(Kps, Kds)]
    elif controller_type == "PD":
        return [PDController(p=p, K=K, Kp=kp, tauD=kd) for kp, kd in zip(Kps, Kds)]
    elif controller_type == "PI":
        return [PIController(p=p, K=K, Kp=kp, tauI=ki) for kp, ki in zip(Kps, Kis)]
    elif controller_type == "PID":
        return [
            PIDController(p=p, K=K, Kp=kp, tauI=ki, tauD=kd)
            for kp, ki, kd in zip(Kps, Kis, Kds)
        ]
    elif controller_type == "PI-D":
        return [
            PI_DController(p=p, K=K, Kp=kp, tauI=ki, tauD=kd)
            for kp, ki, kd in zip(Kps, Kis, Kds)
        ]
    elif controller_type == "PID-D":
        return [
            PID_DController(p=p, K=K, Kp=kp, tauI=ki, tauD=kd)
            for kp, ki, kd in zip(Kps, Kis, Kds)
        ]
    elif controller_type == "D|PID":
        return [
            D__PIDController(p=p, K=K, Kp=kp, tauI=ki, tauD=kd)
            for kp, ki, kd in zip(Kps, Kis, Kds)
        ]
    else:
        raise ValueError("Invalid controller type")


def simulate_transfer_function(
    H: signal.lti, u: np.ndarray, t: np.ndarray
) -> tuple[np.ndarray, float, float]:
    [t, yout, xout] = signal.lsim(H, u, t)
    rising_time = t[np.argmax(yout > (0.9 * u))] - t[0]
    max_overshoot = max(yout - u) * 100
    return yout, max_overshoot, rising_time


def plot_system_response(
    controllers: list[Controller],
    reference_name: Literal["unit_step", "ramp", "parabolic"],
    start,
    end,
    plot_error=False,
):
    """
    Plot the system response for the given controllers and reference signal type. Save the plot in a file
    """
    # Plot the step response
    t, u = get_reference_signal(reference_name, start, end)
    controller_type = controllers[0].get_controller_type()
    os.makedirs(f"Figuras/{controller_type}", exist_ok=True)

    youts = []
    for cntr in tqdm(controllers, desc="Simulating controllers"):
        yout, max_over, rising_time = simulate_transfer_function(
            cntr.get_transfer_function(), u, t
        )
        plt.plot(t, yout, label=cntr.get_label())
        youts.append(yout)

    t_ref = np.insert(t, 0, [-0.2, -0.0000001])
    u_ref = np.insert(u, 0, [0, 0])
    plt.plot(t_ref, u_ref, label="Entrada")
    plt.legend()
    plt.title("Respuesta en el tiempo a la entrada de referencia")
    plt.xlabel("pt [s]")
    plt.ylabel("p * y(t)")
    plt.grid(alpha=0.2)
    plt.xticks(np.arange(t[0], t[-1], 1))
    plt.savefig(
        f"Figuras/{controller_type}/time_response_{reference_name}.png",
        bbox_inches="tight",
    )
    print(
        f"Time response graph saved in Figuras/{controller_type}/time_response_{reference_name}.png"
    )
    plt.close()

    if plot_error:
        for i, cntr in enumerate(controllers):
            plt.plot(t, u - youts[i], label=cntr.get_label())

        plt.legend()
        plt.title("Error en régimen permanente")
        plt.xlabel("pt [s]")
        plt.ylabel("p * e(t)")
        plt.xticks(np.arange(t[0], t[-1], 1), rotation=45)
        plt.grid(alpha=0.2)
        plt.savefig(
            f"Figuras/{controller_type}/steady_state_error_{reference_name}.png",
            bbox_inches="tight",
        )
        print(
            f"Steady state graph saved in Figures/{controller_type}/steady_state_error_{reference_name}.png"
        )
        # plt.show()
        plt.close()


def plot_overshoot_and_rising_time(
    controllers: list[Controller],
    reference_name: Literal["unit_step", "ramp", "parabolic"],
    start: float,
    end: float,
    x_ticks,
    x_label,
):
    """
    Plot the maximum overshoot and rising time for the given controllers and reference signal type. Save the plot in a file
    """
    t, u = get_reference_signal(reference_name, start, end)
    controller_type = controllers[0].get_controller_type()
    os.makedirs(f"Figuras/{controller_type}", exist_ok=True)

    overshoots = []
    rising_times = []
    for tf in tqdm(controllers, desc="Simulating controllers"):
        yout, max_over, rising_time = simulate_transfer_function(
            tf.get_transfer_function(), u, t
        )
        overshoots.append(max_over)
        rising_times.append(rising_time)

    plt.plot(x_ticks, overshoots, color="coral")
    plt.title(f"Sobreelongación máxima en función de {x_label}")
    plt.xlabel(x_label)
    plt.ylabel("Sobreelongación máxima [% del valor final]")
    plt.xticks(np.linspace(x_ticks[0], x_ticks[-1], 15), rotation=45)
    plt.grid(alpha=0.2)
    plt.gca().yaxis.set_major_formatter(
        plt.matplotlib.ticker.StrMethodFormatter("{x:.0f}%")
    )
    plt.savefig(
        f"Figuras/{controller_type}/overshoot_{reference_name}.png", bbox_inches="tight"
    )
    print(
        f"Overshoots graph saved in Figuras/{controller_type}/overshoot_{reference_name}.png"
    )
    # plt.show()
    plt.close()

    plt.plot(x_ticks, rising_times, color="coral")
    plt.title(f"Tiempo de subida en función de {x_label}")
    plt.xlabel(x_label)
    plt.ylabel("Tiempo de subida - pt [s]")
    plt.xticks(np.linspace(x_ticks[0], x_ticks[-1], 15), rotation=45)
    plt.grid(alpha=0.2)
    plt.savefig(
        f"Figuras/{controller_type}/rising_time_{reference_name}.png",
        bbox_inches="tight",
    )
    print(
        f"Rising times graph saved in Figuras/{controller_type}/rising_time_{reference_name}.png"
    )
    # plt.show()
    plt.close()


if __name__ == "__main__":
    CONTROLLER_TYPE = "D|PID"  # "P", "P-D", "PD", "PI", "PID", "PI-D", "PID-D", "D|PID"
    INPUT_FUNCTION = "parabolic"  # "unit_step", "ramp", "parabolic"
    pid_constants = PIDConstants()  # Data structure to store PID constants

    pid_constants.Kds = [0.01, 0.05, 0.1, 0.5, 1]
    pid_constants.Kps = np.ones_like(pid_constants.Kds)
    pid_constants.Kis = np.ones_like(pid_constants.Kds)
    transfer_functions = obtain_transfer_functions(CONTROLLER_TYPE, pid_constants)
    plot_system_response(
        transfer_functions,
        reference_name=INPUT_FUNCTION,
        start=0,
        end=20,
        plot_error=True,
    )

    pid_constants.Kds = np.linspace(pid_constants.Kds[0], pid_constants.Kds[-1], 15)
    pid_constants.Kps = np.ones_like(pid_constants.Kps)
    pid_constants.Kis = np.ones_like(pid_constants.Kps)
    transfer_functions = obtain_transfer_functions(CONTROLLER_TYPE, pid_constants)
    plot_overshoot_and_rising_time(
        transfer_functions,
        reference_name=INPUT_FUNCTION,
        start=0,
        end=100,
        x_ticks=pid_constants.Kds,
        x_label="$\\tau_D$",
    )
