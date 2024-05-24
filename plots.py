import numpy as np
import matplotlib.pyplot as plt

# System parameters
params_exact = {'a': 10, 'b': 100, 'c': 0.3}
params_approx = {'a': 9, 'b': 98, 'c': 0.4}

# Time settings
dt = 0.001
T = 20.0
t = np.arange(0, T, dt)

# Nominal trajectory functions
def nominal_trajectory(A, omega, t):
    return A * np.sin(omega * t)

# PID Control Parameters
Kp = 0.1
Ki = 0.01
Kd = 0.05

# Function to simulate the control system with PID
def simulate_control_system(params, t):
    n = len(t)
    x, y, z = np.zeros(n), np.zeros(n), np.zeros(n)
    ux, uy, uz = np.zeros(n), np.zeros(n), np.zeros(n)
    integral_x, integral_y, integral_z = 0, 0, 0

    for i in range(1, n):
        # Calculate the error
        error_x = nominal_trajectory(2, 0.5, t[i]) - x[i-1]
        error_y = nominal_trajectory(3.8, 0.7, t[i]) - y[i-1]
        error_z = nominal_trajectory(1.5, 0.9, t[i]) - z[i-1]

        # Integrate the error
        integral_x += error_x * dt
        integral_y += error_y * dt
        integral_z += error_z * dt

        # PID control signals
        derivative_x = (error_x - (nominal_trajectory(2, 0.5, t[i-1]) - x[i-2])) / dt if i > 1 else 0
        derivative_y = (error_y - (nominal_trajectory(3.8, 0.7, t[i-1]) - y[i-2])) / dt if i > 1 else 0
        derivative_z = (error_z - (nominal_trajectory(1.5, 0.9, t[i-1]) - z[i-2])) / dt if i > 1 else 0

        ux[i] = Kp * error_x + Ki * integral_x + Kd * derivative_x
        uy[i] = Kp * error_y + Ki * integral_y + Kd * derivative_y
        uz[i] = Kp * error_z + Ki * integral_z + Kd * derivative_z

        # System dynamics
        x[i] = x[i-1] + dt * (params['a'] * y[i-1] + ux[i])
        y[i] = y[i-1] + dt * (-params['c'] * x[i-1] + y[i-1] * z[i-1] + uy[i])
        z[i] = z[i-1] + dt * (params['b'] - y[i-1]**2 + uz[i])

    return t, x, y, z, ux, uy, uz

# Simulate the system with control using exact parameters
t, x_control, y_control, z_control, ux, uy, uz = simulate_control_system(params_exact, t)

# Plotting

# Nominal and Realized Trajectories
plt.figure(figsize=(12, 8))
plt.subplot(311)
plt.plot(t, x_control, label='Realized x')
plt.plot(t, nominal_trajectory(2, 0.5, t), 'r--', label='Nominal x')
plt.title('Nominal and Realized Trajectories')
plt.legend()

plt.subplot(312)
plt.plot(t, y_control, label='Realized y')
plt.plot(t, nominal_trajectory(3.8, 0.7, t), 'g--', label='Nominal y')
plt.legend()

plt.subplot(313)
plt.plot(t, z_control, label='Realized z')
plt.plot(t, nominal_trajectory(1.5, 0.9, t), 'b--', label='Nominal z')
plt.legend()

plt.tight_layout()
plt.show()

# Tracking Errors
plt.figure(figsize=(12, 4))
plt.plot(t, nominal_trajectory(2, 0.5, t) - x_control, 'r', label='Error x')
plt.plot(t, nominal_trajectory(3.8, 0.7, t) - y_control, 'g', label='Error y')
plt.plot(t, nominal_trajectory(1.5, 0.9, t) - z_control, 'b', label='Error z')
plt.title('Tracking Errors')
plt.legend()
plt.tight_layout()
plt.show()

# Phase Spaces
plt.figure(figsize=(12, 8))
plt.subplot(131)
plt.plot(x_control, y_control, label='Phase Space xy')
plt.title('Phase Space xy')
plt.legend()

plt.subplot(132)
plt.plot(y_control, z_control, label='Phase Space yz')
plt.title('Phase Space yz')
plt.legend()

plt.subplot(133)
plt.plot(x_control, z_control, label='Phase Space xz')
plt.title('Phase Space xz')
plt.legend()

plt.tight_layout()
plt.show()

# Control Signals
plt.figure(figsize=(12, 4))
plt.plot(t, ux, 'r', label='Control Signal ux')
plt.plot(t, uy, 'g', label='Control Signal uy')
plt.plot(t, uz, 'b', label='Control Signal uz')
plt.title('Control Signals')
plt.legend()
plt.tight_layout()
plt.show()

plt.savefig('the last task the  four images', format='svg')