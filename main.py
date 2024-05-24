import numpy as np
import matplotlib.pyplot as plt

# Constants and Parameters
params_exact = {'a': 10, 'b': 100, 'c': 0.3}
params_approx = {'a': 9, 'b': 98, 'c': 0.4}
dt = 0.001
total_time = 20
t = np.arange(0, total_time, dt)

# Initial conditions
initial_conditions = [0.1, 0.1, 0.1]
u_x , u_y , u_z = 0 , 0 , 0

# System Dynamics Function
def simulate_system(params):
    n = len(t)
    x = np.zeros(n)
    y = np.zeros(n)
    z = np.zeros(n)
    x[0], y[0], z[0] = initial_conditions

    for i in range(1, n):
        x[i] = x[i-1] + dt * (params['a'] * y[i-1]) + u_x
        y[i] = y[i-1] + dt * (-params['c'] * x[i-1] + y[i-1] * z[i-1]) + u_y 
        z[i] = z[i-1] + dt * (params['b'] - y[i-1]**2) + u_z

    return x, y, z

# Simulate with exact parameters
x_exact, y_exact, z_exact = simulate_system(params_exact)
# Simulate with approximate parameters
x_approx, y_approx, z_approx = simulate_system(params_approx)

# Plotting
plt.figure(figsize=(18, 10))  # Adjusted figure size

# 3D Phase Space Plot
ax1 = plt.subplot(2, 3, 1, projection='3d')
ax1.plot(x_exact, y_exact, z_exact, label='Exact Parameters')
ax1.plot(x_approx, y_approx, z_approx, label='Approximate Parameters', linestyle='--')
ax1.set_title('3D Phase Space Comparison')
ax1.set_xlabel('x')
ax1.set_ylabel('y')
ax1.set_zlabel('z')
ax1.legend()

# Time Plots for x, y, z
ax2 = plt.subplot(2, 3, 4)
ax2.plot(t, x_exact, label='Exact x')
ax2.plot(t, x_approx, label='Approx x', linestyle='--')
ax2.set_title('Time Plot for x')
ax2.set_xlabel('Time')
ax2.set_ylabel('x')
ax2.legend()

ax3 = plt.subplot(2, 3, 5)
ax3.plot(t, y_exact, label='Exact y')
ax3.plot(t, y_approx, label='Approx y', linestyle='--')
ax3.set_title('Time Plot for y')
ax3.set_xlabel('Time')
ax3.set_ylabel('y')
ax3.legend()

ax4 = plt.subplot(2, 3, 6)
ax4.plot(t, z_exact, label='Exact z')
ax4.plot(t, z_approx, label='Approx z', linestyle='--')
ax4.set_title('Time Plot for z')
ax4.set_xlabel('Time')
ax4.set_ylabel('z')
ax4.legend()

plt.tight_layout(pad=3.0)  # Adjust layout padding to prevent overlap
plt.show()

plt.figure(1)
plt.savefig('images without the control signals', format='svg')
