clc; clear; close all;

% Parameters
m = 5;                % UAV mass (kg)
g = 9.81;             % Gravity (m/s²)
Cd = 0.05;            % Drag coefficient
rho = 1.225;          % Air density (kg/m³)
A = 0.3;              % UAV cross-sectional area (m²)
thrust_max = 20;      % Maximum thrust (N)
dt = 0.1;             % Time step (s)
t_end = 10;           % Simulation duration (s)
n_steps = t_end / dt; % Number of time steps

% Initial conditions
x = zeros(1, n_steps);
y = zeros(1, n_steps);
vx = 5;  % Initial velocity in x-direction (m/s)
vy = 2;  % Initial velocity in y-direction (m/s)

% Simulate original trajectory (without optimization)
for i = 2:n_steps
    v = sqrt(vx^2 + vy^2);
    F_drag = 0.5 * Cd * rho * A * v^2;
    
    ax = (thrust_max - F_drag) / m; % Acceleration in x
    ay = (-m * g + 0.5 * Cd * rho * A * v^2) / m; % Acceleration in y
    
    vx = vx + ax * dt;
    vy = vy + ay * dt;
    
    x(i) = x(i-1) + vx * dt;
    y(i) = y(i-1) + vy * dt;
end

% Optimization: Adjust trajectory to minimize energy consumption
thrust_opt = linspace(10, 20, n_steps); % Adjust thrust dynamically
x_opt = zeros(1, n_steps);
y_opt = zeros(1, n_steps);
vx_opt = 5;
vy_opt = 2;

for i = 2:n_steps
    v_opt = sqrt(vx_opt^2 + vy_opt^2);
    F_drag_opt = 0.5 * Cd * rho * A * v_opt^2;
    
    ax_opt = (thrust_opt(i) - F_drag_opt) / m;
    ay_opt = (-m * g + 0.5 * Cd * rho * A * v_opt^2) / m;
    
    vx_opt = vx_opt + ax_opt * dt;
    vy_opt = vy_opt + ay_opt * dt;
    
    x_opt(i) = x_opt(i-1) + vx_opt * dt;
    y_opt(i) = y_opt(i-1) + vy_opt * dt;
end

% Plot results
figure;
plot(x, y, 'r-', 'LineWidth', 2); hold on;
plot(x_opt, y_opt, 'b--', 'LineWidth', 2);
legend('Original Trajectory', 'Optimized Trajectory');
xlabel('X Position (m)');
ylabel('Y Position (m)');
title('UAV Trajectory Optimization');
grid on;

% Save plot
saveas(gcf, 'uav_trajectory_optimization.png');
