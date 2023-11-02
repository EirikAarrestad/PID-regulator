import numpy as np
import matplotlib.pyplot as plt


def generate_vector(start, stop, step):
    return [x for x in range(start, stop + 1, step)]


def fill_vector_with_values(vector_length, values):
    num_segments = len(values)
    segment_length = vector_length // num_segments

    filled_vector = []

    for value in values:
        filled_vector.extend([value] * segment_length)

    # Fill any remaining positions in case of rounding errors
    remaining_length = vector_length - len(filled_vector)
    filled_vector.extend([values[-1]] * (remaining_length + 1))

    return filled_vector


# Defining a ControlSystem class
class ControlSystem:
    def __init__(
        self,
        initial_temperature=15,
        target_temperature=25,
        ambient_temperature=-5,
        Kc=0.5,
        Ti=1,
        Td=0.01,
        time_constant=2,
        time_step=2,
    ):
        self.current_temperature = initial_temperature  # Initial temperature
        self.target_temperature = target_temperature  # Target temperature
        self.heater_power = 5  # Initial value of heater power
        self.Kc = Kc  # Proportional gain
        self.Ti = Ti  # Integral time constant
        self.Td = Td  # Derivative time constant
        self.time_constant = time_constant  # Time constant
        self.integral = 0  # Initial value of integral
        self.time_step = time_step  # Time step
        self.ambient_temperature = ambient_temperature  # Ambient temperature

    def set_variating_target_temperature(
        self, target_temp, duration, perturbed_target_temp=5, n_steps=3
    ):
        no_of_time_steps = int(duration / self.time_step)
        values_to_fill = [
            target_temp,
            target_temp - perturbed_target_temp,
            target_temp + 1.5 * perturbed_target_temp,
        ]
        filled_vector = fill_vector_with_values(no_of_time_steps, values_to_fill)
        return np.array(filled_vector)

    def set_target_temperature(self, target_temp, duration=None):
        """
        Sets target temperature
        :param target_temp: Target temperature
        :return: target vector
        """
        if duration:
            self.target_temp_vec = self.set_variating_target_temperature(
                target_temp, duration
            )
        else:
            self.target_temp_vec = target_temp * np.ones(int(duration / self.time_step))

    def set_heater_power(self, power):
        """
        Sets manual heater power
        :param power: Power (W)
        :return:
        """
        self.heater_power = power

    def simulate(self, duration):
        time = 0
        temperature_history = [self.current_temperature]
        heater_power_history = [self.heater_power]
        time_index = 0
        prev_error = 0
        while time < duration:
            error = self.target_temp_vec[time_index] - self.current_temperature

            # PI controller calculations with derivative term (D)
            self.integral += error * self.time_step
            P = self.Kc * error
            I = self.Kc / self.Ti * self.integral
            D = self.Kc * self.Td * (error - prev_error) / self.time_step

            # Heater power input
            heater_power = self.heater_power + P + I + D
            # heater_power = self.heater_power + P + I

            # Room temperature dynamics
            delta_temperature = (
                heater_power
                - (self.current_temperature - self.ambient_temperature)
                / self.time_constant
            ) * self.time_step
            self.current_temperature += delta_temperature

            temperature_history.append(self.current_temperature)
            heater_power_history.append(heater_power)
            prev_error = error
            time += self.time_step
            time_index += 1
        return temperature_history, heater_power_history


# Defining the main function
def main():
    time_step = 0.1  # Time step for simulation
    simulation_duration = 60  # Duration of simulation, number of timesteps
    heater_power_data = []
    control_system = ControlSystem(time_step=time_step)
    time_points = [
        i * time_step for i in range(int(simulation_duration / time_step) + 1)
    ]
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
            temperature_history, heater_power_history = control_system.simulate(
                simulation_duration
            )
            temperature_data = temperature_history  # Store the temperature data
            # temperature_setpoint = control_system.target_temperature * np.ones(len(temperature_data))
            temperature_setpoint = control_system.target_temp_vec
            heater_power_data = heater_power_history
            time_points = [i * time_step for i in range(len(temperature_history))]
            print("Simulation completed. Temperature history:")
            for i, temp in enumerate(temperature_history):
                print(f"Time: {i * time_step:.2f}s, Temperature: {temp:.2f}°C")
        elif choice == "4":
            if not temperature_data:
                print("No simulation data available. Please run a simulation first.")
            else:
                # Create a subplot to show temperature and heater power
                plt.figure(figsize=(10, 6))
                # Plot temperature
                plt.subplot(2, 1, 1)
                plt.plot(
                    time_points,
                    temperature_setpoint,
                    "k",
                    label="Target temperature (°C)",
                    lw=3,
                )
                plt.plot(
                    time_points,
                    temperature_data,
                    "g",
                    label="Measured temperature (°C)",
                )
                plt.xlabel("Time (hours)")
                plt.ylabel("Temperature (°C)")
                plt.title("Temperature vs. Time")
                plt.grid(True)
                plt.legend()
                # Plot heater power
                plt.subplot(2, 1, 2)
                plt.plot(time_points, heater_power_data, label="Heater Power (Watts)")
                plt.xlabel("Time (hours)")
                plt.ylabel("Heater Power (Watts)")
                plt.title("Heater Power vs. Time")
                plt.grid(True)
                plt.legend()
                plt.tight_layout()  # To ensure subplots don't overlap
                plt.show()
            break
        else:
            print("Invalid choice. Please try again.")


if __name__ == "__main__":
    main()
