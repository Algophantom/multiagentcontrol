clear; clc; close all;

T = 100;%total simulation time
dt = 0.0001;%sampling period
steps = T / dt;%no of steps

mu = [1,3, 2, 5];%mu is the factor in xdot
alpha = [5,8, 7, 3];%alpha and beta are scalars in controller states
%alpha = [15,8, 17, 3];
beta = [3,2, 5, 2];


%initial states
x_p = [30;15; -50; -10];
%x_p = [20; -15; -20];
x_c = [1; 2; -1;1];

Q = [ 1 -1  0  0;
      1  0 -1  0;
      0  1  0 -1;
      0  0  1 -1];%incidence matrix based on the weakly connected graph

x_p_traj = zeros(4, steps);%intialise the state matrices
x_c_traj = zeros(4, steps);
u_p_traj = zeros(4, steps);
sat_limit = 10;%set saturation limits for the input(to simulate a realistic scenario)

for t = 1:steps
    y_p = x_p;%double intergrator plant
    u_c = Q * y_p;%transformed controller input
    dx_c = -alpha' .* x_c - beta' .* (x_c.^3) + u_c;%state updation in OSNI controller
    x_c = x_c + dt * dx_c;
    
    y_c = x_c - u_c;%output of controller
    u_p = Q' * y_c;%transformed control input to the system
    u_p = max(min(u_p, sat_limit), -sat_limit);%saturated input correction
    
    
    dx_p = mu' .* u_p.^3;%plant dynamics
    x_p = x_p + dt * dx_p;%plant state updation
    
    x_p_traj(:, t) = x_p;%stacking of states
    x_c_traj(:, t) = x_c;
    u_p_traj(:, t) = u_p;
end

time = linspace(dt, T, steps);

figure;
set(gcf, 'Color', 'w');
semilogx(time, x_p_traj(2,:), 'r-', 'LineWidth', 1.5); hold on;
semilogx(time, x_p_traj(3,:), 'g-', 'LineWidth', 1.5);
semilogx(time, x_p_traj(4,:), 'b-', 'LineWidth', 1.5);
semilogx(time, x_p_traj(1,:), 'm-', 'LineWidth', 1.5);
xlabel('Time (s) [log scale]');
ylabel('Plant Outputs');
legend('x_{p1}','x_{p2}', 'x_{p3}', 'x_{p4}', 'Location', 'best');
title('Plant Outputs Over Time (Consensus Behavior)');
grid on;

