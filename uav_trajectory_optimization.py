import numpy as np
import matplotlib.pyplot as plt
from scipy.integrate import solve_ivp
from scipy.optimize import minimize

# Define UAV dynamics
def uav_dynamics(t, state, wind_speed, terrain_factor):
    x, y, vx, vy, E = state
    dxdt = vx
    dydt = vy
    dvxdt = -0.01 * vx + wind_speed
    dvydt = -0.01 * vy - terrain_factor * np.sin(0.1 * x)
    dEdt = -0.05 * (vx**2 + vy**2)  # Energy depletion
    return [dxdt, dydt, dvxdt, dvydt, dEdt]

# Initial conditions
initial_state = [0, 0, 2, 2, 100]  # (x, y, vx, vy, energy)
time_span = (0, 50)
time_eval = np.linspace(time_span[0], time_span[1], 500)
wind_speed = 0.5
terrain_factor = 0.2

# Solve original trajectory
solution_original = solve_ivp(uav_dynamics, time_span, initial_state, t_eval=time_eval, args=(wind_speed, terrain_factor))

# Optimization: Minimize energy consumption while reaching target (x=50, y=20)
def cost_function(control_params):
    vx_opt, vy_opt = control_params
    optimized_state = [0, 0, vx_opt, vy_opt, 100]
    sol = solve_ivp(uav_dynamics, time_span, optimized_state, t_eval=time_eval, args=(wind_speed, terrain_factor))
    x_final, y_final = sol.y[0, -1], sol.y[1, -1]
    energy_final = sol.y[4, -1]
    target_x, target_y = 50, 20
    return abs(x_final - target_x) + abs(y_final - target_y) - energy_final * 0.1

# Run optimization
result = minimize(cost_function, [2, 2], bounds=[(1, 5), (1, 5)])
optimal_vx, optimal_vy = result.x
optimized_state = [0, 0, optimal_vx, optimal_vy, 100]
solution_optimized = solve_ivp(uav_dynamics, time_span, optimized_state, t_eval=time_eval, args=(wind_speed, terrain_factor))

# Plot results
plt.figure(figsize=(10, 6))
plt.plot(solution_original.y[0], solution_original.y[1], label='Original Trajectory', linestyle='dashed')
plt.plot(solution_optimized.y[0], solution_optimized.y[1], label='Optimized Trajectory', linewidth=2)
plt.scatter([50], [20], color='red', marker='x', label='Target Point')
plt.xlabel('X Position')
plt.ylabel('Y Position')
plt.title('UAV Trajectory Optimization')
plt.legend()
plt.grid()

# Save the plot as an image
plt.savefig("uav_trajectory_optimization.png", dpi=300, bbox_inches='tight')

plt.show()
