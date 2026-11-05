"""Simulation of a PID controller regulating the temperature of a room.

The room is modelled as a first-order system that loses heat to the
surroundings, discretised with the explicit Euler method:

    T[k+1] = T[k] + dt * (u[k] - (T[k] - T_ambient) / tau)

The heater power u[k] is a base power plus proportional, integral and
derivative terms of the error between target and measured temperature.
"""

import matplotlib.pyplot as plt
import numpy as np


class ControlSystem:
    """A room heated by a PID-controlled heater.

    Attributes:
        current_temperature: Current room temperature (°C).
        target_temperature: Default target temperature (°C).
        ambient_temperature: Outdoor temperature (°C).
        time_constant: Thermal time constant of the room (hours).
        heater_power: Base heater power added to the PID output (W).
        Kc: Proportional gain.
        Ti: Integral time constant (hours).
        Td: Derivative time constant (hours).
        integral: Accumulated error used by the integral term.
        time_step: Simulation time step (hours).
        target_temp_vec: Target temperature at each time step (°C).
    """

    def __init__(
        self,
        initial_temperature: float = 15,
        target_temperature: float = 25,
        ambient_temperature: float = -5,
        Kc: float = 0.5,
        Ti: float = 1,
        Td: float = 0.01,
        time_constant: float = 2,
        time_step: float = 2,
    ) -> None:
        """Initialise the room and controller parameters.

        Args:
            initial_temperature: Room temperature at the start (°C).
            target_temperature: Default target temperature (°C).
            ambient_temperature: Outdoor temperature (°C).
            Kc: Proportional gain.
            Ti: Integral time constant (hours).
            Td: Derivative time constant (hours).
            time_constant: Thermal time constant of the room (hours).
            time_step: Simulation time step (hours).
        """
        # Room
        self.current_temperature = initial_temperature
        self.target_temperature = target_temperature
        self.ambient_temperature = ambient_temperature
        self.time_constant = time_constant

        # Controller
        self.heater_power = 5.0
        self.Kc = Kc
        self.Ti = Ti
        self.Td = Td
        self.integral = 0.0

        # Simulation
        self.time_step = time_step
        self.target_temp_vec = np.array([])

    def set_variating_target_temperature(
        self,
        target_temp: float,
        duration: float,
        perturbed_target_temp: float = 5,
    ) -> np.ndarray:
        """Create a target temperature that changes in three steps.

        The duration is split into three equal parts with targets
        `target_temp`, `target_temp - perturbed_target_temp` and
        `target_temp + 1.5 * perturbed_target_temp`.

        Args:
            target_temp: Target temperature in the first part (°C).
            duration: Total simulation time (hours).
            perturbed_target_temp: Size of the step changes (°C).

        Returns:
            Target temperature at each time step.
        """
        no_of_time_steps = int(duration / self.time_step)
        values_to_fill = [
            target_temp,
            target_temp - perturbed_target_temp,
            target_temp + 1.5 * perturbed_target_temp,
        ]
        return np.array(fill_vector_with_values(no_of_time_steps, values_to_fill))

    def set_target_temperature(
        self,
        target_temp: float,
        duration: float,
        variating: bool = True,
    ) -> None:
        """Set the target temperature for every time step of a simulation.

        Args:
            target_temp: Target temperature (°C).
            duration: Total simulation time (hours).
            variating: If True, the target changes in three steps.
                If False, it is constant.
        """
        self.target_temperature = target_temp
        if variating:
            self.target_temp_vec = self.set_variating_target_temperature(
                target_temp, duration
            )
        else:
            # One extra value so the vector matches the simulated time points
            no_of_time_steps = int(duration / self.time_step) + 1
            self.target_temp_vec = target_temp * np.ones(no_of_time_steps)

    def set_heater_power(self, power: float) -> None:
        """Set the base heater power added to the PID output.

        Args:
            power: Base heater power (W).
        """
        self.heater_power = power

    def simulate(self, duration: float) -> tuple[list[float], list[float]]:
        """Run the simulation with the PID controller.

        `set_target_temperature` must be called first.

        Args:
            duration: Total simulation time (hours).

        Returns:
            Room temperature (°C) and heater power (W) at each time step,
            including the starting values.
        """
        time = 0.0
        time_index = 0
        prev_error = 0.0
        temperature_history = [self.current_temperature]
        heater_power_history = [self.heater_power]

        while time < duration:
            error = self.target_temp_vec[time_index] - self.current_temperature

            # PID terms
            self.integral += error * self.time_step
            P = self.Kc * error
            I = self.Kc / self.Ti * self.integral
            D = self.Kc * self.Td * (error - prev_error) / self.time_step
            heater_power = self.heater_power + P + I + D

            # Room dynamics: heat added minus heat lost to the surroundings
            heat_loss = (
                self.current_temperature - self.ambient_temperature
            ) / self.time_constant
            self.current_temperature += (heater_power - heat_loss) * self.time_step

            temperature_history.append(self.current_temperature)
            heater_power_history.append(heater_power)
            prev_error = error
            time += self.time_step
            time_index += 1

        return temperature_history, heater_power_history


def fill_vector_with_values(vector_length: int, values: list[float]) -> list[float]:
    """Fill a list with equally long segments of the given values.

    The last value is repeated to cover any remainder from the division.
    One extra element is added so the list matches the number of simulated
    time points (time steps plus the starting point).

    Args:
        vector_length: Number of time steps.
        values: Values to fill the segments with, in order.

    Returns:
        List of length `vector_length + 1`.
    """
    segment_length = vector_length // len(values)

    filled_vector: list[float] = []
    for value in values:
        filled_vector.extend([value] * segment_length)

    # Cover the remainder from rounding, plus one extra point
    remaining_length = vector_length - len(filled_vector)
    filled_vector.extend([values[-1]] * (remaining_length + 1))

    return filled_vector


def plot_results(
    time_points: list[float],
    target: np.ndarray,
    temperature: list[float],
    heater_power: list[float],
) -> None:
    """Plot target and measured temperature, and heater power, over time.

    Args:
        time_points: Time at each step (hours).
        target: Target temperature at each step (°C).
        temperature: Measured temperature at each step (°C).
        heater_power: Heater power at each step (W).
    """
    plt.figure(figsize=(10, 6))

    plt.subplot(2, 1, 1)
    plt.plot(time_points, target, "k", label="Target temperature (°C)", lw=3)
    plt.plot(time_points, temperature, "g", label="Measured temperature (°C)")
    plt.xlabel("Time (hours)")
    plt.ylabel("Temperature (°C)")
    plt.title("Temperature / Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time_points, heater_power, label="Heater Power (Watts)")
    plt.xlabel("Time (hours)")
    plt.ylabel("Heater Power (Watts)")
    plt.title("Heater Power / Time")
    plt.grid(True)
    plt.legend(loc="upper left")

    plt.tight_layout()
    plt.show()


def main() -> None:
    """Run an interactive menu for setting parameters, simulating and plotting."""
    time_step = 0.1  # hours
    simulation_duration = 60  # hours

    control_system = ControlSystem(time_step=time_step)

    # Filled in when a simulation has been run
    temperature_data: list[float] = []
    heater_power_data: list[float] = []
    temperature_setpoint = np.array([])
    time_points: list[float] = []

    while True:
        print("\nCurrent Temperature:", control_system.current_temperature)
        print("1. Set Target Temperature")
        print("2. Set Initial heater Power")
        print("3. Simulate")
        print("4. Plot and Exit")
        choice = input("Enter your choice: ")

        if choice == "1":
            target_temp = float(input("Enter target temperature: "))
            control_system.set_target_temperature(target_temp, simulation_duration)

        elif choice == "2":
            heater_power = float(input("Enter initial heater power (Watts): "))
            control_system.set_heater_power(heater_power)

        elif choice == "3":
            control_system.set_target_temperature(
                control_system.target_temperature, simulation_duration
            )
            temperature_data, heater_power_data = control_system.simulate(
                simulation_duration
            )
            temperature_setpoint = control_system.target_temp_vec
            time_points = [i * time_step for i in range(len(temperature_data))]

            print("Simulation completed. Temperature history:")
            for t, temp in zip(time_points, temperature_data):
                print(f"Time: {t:.2f} h, Temperature: {temp:.2f}°C")

        elif choice == "4":
            if not temperature_data:
                print("No simulation data available. Please run a simulation first.")
                continue
            plot_results(
                time_points, temperature_setpoint, temperature_data, heater_power_data
            )
            break

        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
