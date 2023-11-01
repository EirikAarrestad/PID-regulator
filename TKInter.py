# Import necessary libraries
import numpy as np
import matplotlib.pyplot as plt
import tkinter as tk
from tkinter import Label, Entry, Button, Frame


# Define a ControlSystem class to model temperature control
class ControlSystem:
    def __init__(
        self,
        initial_temperature=15,
        target_temperature=25,
        ambient_temperature=-5,
        Kc=0.5,
        Ti=1,
        time_constant=2,
        time_step=2,
    ):
        # Initialize the control system with default or provided parameters
        self.current_temperature = initial_temperature  # Initial temperature
        self.target_temperature = target_temperature  # Target temperature
        self.heater_power = 5  # Initial value of heater power
        self.Kc = Kc  # Proportional gain
        self.Ti = Ti  # Integral time constant
        self.time_constant = time_constant  # Time constant
        self.integral = 0  # Initial value of integral
        self.time_step = time_step  # Time step
        self.ambient_temperature = ambient_temperature  # Ambient temperature

    # Methods to set target temperature and manual heater power
    def set_target_temperature(self, target_temp):
        """
        Sets target temperature
        :param target_temp: Target temperature
        :return:
        """
        self.target_temperature = target_temp

    def set_heater_power(self, power):
        """
        Sets manual heater power
        :param power: Power (W)
        :return:
        """
        self.heater_power = power

    # Simulate the temperature control system with a PI controller
    def simulate(self, duration):
        """
        Simulate dynamic system with PI controller
        :param duration: Duration on simulation (number of timesteps)
        :return: simulated temperature vector and heater vector
        """
        time = 0
        temperature_history = [self.current_temperature]
        heater_power_history = [self.heater_power]
        while time < duration:
            error = self.target_temperature - self.current_temperature
            # PI controller calculations
            self.integral += error * self.time_step
            P = self.Kc * error
            I = self.Kc / self.Ti * self.integral
            # Heater power input
            heater_power = self.heater_power + P + I
            # Room temperature dynamics
            delta_temperature = (
                heater_power
                - (self.current_temperature - self.ambient_temperature)
                / self.time_constant
            ) * self.time_step
            self.current_temperature += delta_temperature
            temperature_history.append(self.current_temperature)
            heater_power_history.append(heater_power)
            time += self.time_step
        return temperature_history, heater_power_history


# Functions for GUI control
def set_target_temperature():
    target_temp = float(target_temp_entry.get())
    control_system.set_target_temperature(target_temp)
    current_temp_label.config(
        text=f"Current Temperature: {control_system.current_temperature:.2f} °C"
    )


def set_initial_heater_power():
    heater_power = float(heater_power_entry.get())
    control_system.set_heater_power(heater_power)


def simulate():
    # Simulate the control system and plot the results
    temperature_history, heater_power_history = control_system.simulate(
        simulation_duration
    )
    temperature_data = temperature_history
    temperature_setpoint = control_system.target_temperature * np.ones(
        len(temperature_data)
    )
    heater_power_data = heater_power_history
    time_points = [i * time_step for i in range(len(temperature_history))]

    # Create and display two subplots for temperature and heater power
    plt.figure(figsize=(10, 6))
    plt.subplot(2, 1, 1)
    plt.plot(
        time_points, temperature_setpoint, "k", label="Target temperature (°C)", lw=3
    )
    plt.plot(time_points, temperature_data, "g", label="Measured temperature (°C)")
    plt.xlabel("Time (s)")
    plt.ylabel("Temperature (°C)")
    plt.title("Temperature vs. Time")
    plt.grid(True)
    plt.legend()

    plt.subplot(2, 1, 2)
    plt.plot(time_points, heater_power_data, label="Heater Power (Watts)")
    plt.xlabel("Time (s)")
    plt.ylabel("Heater Power (Watts)")
    plt.title("Heater Power vs. Time")
    plt.grid(True)
    plt.legend()
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    time_step = 0.1
    simulation_duration = 30
    control_system = ControlSystem(time_step=time_step)

    # Create a GUI window using Tkinter
    root = tk.Tk()
    root.title("Temperature Control System")

    frame = Frame(root)
    frame.pack(padx=10, pady=10)

    # Create labels, entry fields, and buttons in the GUI
    Label(frame, text="Current Temperature:").grid(row=0, column=0)
    Label(frame, text="Target Temperature:").grid(row=1, column=0)
    Label(frame, text="Initial Heater Power (Watts):").grid(row=2, column=0)

    current_temp_label = Label(
        frame, text=f"Current Temperature: {control_system.current_temperature:.2f} °C"
    )
    current_temp_label.grid(row=0, column=1)

    target_temp_entry = Entry(frame)
    target_temp_entry.grid(row=1, column=1)
    target_temp_entry.insert(0, str(control_system.target_temperature))

    heater_power_entry = Entry(frame)
    heater_power_entry.grid(row=2, column=1)
    heater_power_entry.insert(0, str(control_system.heater_power))

    Button(frame, text="Set Target Temperature", command=set_target_temperature).grid(
        row=1, column=2
    )
    Button(
        frame, text="Set Initial Heater Power", command=set_initial_heater_power
    ).grid(row=2, column=2)
    Button(frame, text="Simulate", command=simulate).grid(row=3, column=0, columnspan=3)

    root.mainloop()
