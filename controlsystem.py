import numpy as np
import matplotlib.pyplot as plt

# Define the system parameters
params_exact = {'a': 10, 'b': 100, 'c': 0.3}
params_approx = {'a': 9, 'b': 98, 'c': 0.4}

# Nominal trajectory functions
def nominal_x(t): return 2 * np.sin(0.5 * t)
def nominal_y(t): return 3.8 * np.sin(0.7 * t)
def nominal_z(t): return 1.5 * np.sin(0.9 * t)

# Time settings
dt = 0.001
t = np.arange(0, 20, dt)  # Simulation for 20 seconds

# PID Control Parameters
Kp, Ki, Kd = 0.1, 0.01, 0.05

# Control System Simulation Function with PID
def simulate_control_system(params, t):
    n = len(t)
    x, y, z = np.zeros(n), np.zeros(n), np.zeros(n)
    ux, uy, uz = np.zeros(n), np.zeros(n), np.zeros(n)
    integral_x, integral_y, integral_z = 0, 0, 0
    error_x, error_y, error_z = 0, 0, 0

    for i in range(1, n):
        # Calculate the error
        error_x = nominal_x(t[i]) - x[i-1]
        error_y = nominal_y(t[i]) - y[i-1]
        error_z = nominal_z(t[i]) - z[i-1]

        # Integrate the error
        integral_x += error_x * dt
        integral_y += error_y * dt
        integral_z += error_z * dt

        # Derivative of the error
        derivative_x = (error_x - (nominal_x(t[i-1]) - x[i-2])) / dt if i > 1 else 0
        derivative_y = (error_y - (nominal_y(t[i-1]) - y[i-2])) / dt if i > 1 else 0
        derivative_z = (error_z - (nominal_z(t[i-1]) - z[i-2])) / dt if i > 1 else 0

        # PID control laws
        ux[i] = Kp * error_x + Ki * integral_x + Kd * derivative_x
        uy[i] = Kp * error_y + Ki * integral_y + Kd * derivative_y
        uz[i] = Kp * error_z + Ki * integral_z + Kd * derivative_z

        # Update state equations
        x[i] = x[i-1] + dt * (params['a'] * y[i-1] + ux[i])
        y[i] = y[i-1] + dt * (-params['c'] * x[i-1] + y[i-1] * z[i-1] + uy[i])
        z[i] = z[i-1] + dt * (params['b'] - y[i-1]**2 + uz[i])

    return t, x, y, z

# Run the simulation for exact and approximate parameters
t, x_exact, y_exact, z_exact = simulate_control_system(params_exact, t)
_, x_approx, y_approx, z_approx = simulate_control_system(params_approx, t)

# Plotting the results
plt.figure(figsize=(12, 8))
plt.subplot(3, 1, 1)
plt.plot(t, x_exact, 'r-', label='Exact x')
plt.plot(t, x_approx, 'r--', label='Approximate x')
plt.plot(t, nominal_x(t), 'k:', label='Nominal x')
plt.title('Controlled Trajectories x')
plt.legend()

plt.subplot(3, 1, 2)
plt.plot(t, y_exact, 'g-', label='Exact y')
plt.plot(t, y_approx, 'g--', label='Approximate y')
plt.plot(t, nominal_y(t), 'k:', label='Nominal y')
plt.title('Controlled Trajectories y')
plt.legend()

plt.subplot(3, 1, 3)
plt.plot(t, z_exact, 'b-', label='Exact z')
plt.plot(t, z_approx, 'b--', label='Approximate z')
plt.plot(t, nominal_z(t), 'k:', label='Nominal z')
plt.title('Controlled Trajectories z')
plt.legend()

plt.tight_layout()
plt.show()

plt.savefig('images with the control signals', format='svg')