clear; clc; close all;

T = 500;
dt = 0.001;
steps = T / dt;

mu = [1, 3, 2, 5];
alpha = [5, 8, 7, 3];
beta = [3, 2, 5, 2];

x_p = [15; 13; 17; -10];
x_c = [1; 2; -1; 1];

Q = [ 1 -1  0  0;
      1  0 -1  0;
      0  1  0 -1;
      0  0  1 -1];
Q3 = [ 1 -1  0 0;
       1  0 -1 0;
       0  1 -1 0;
       0 0 0 0];
x_p_traj = zeros(4, steps);
x_c_traj = zeros(4, steps);
u_p_traj = zeros(4, steps);
sat_limit = 10;

failure_time = 100;
failure_step = failure_time / dt;

disturb_start = 20;
disturb_end = 30;
disturb_magnitude = 5;

for t = 1:steps
    current_time = t * dt;

    d_p = zeros(4, 1);
    %if current_time > disturb_start && current_time < disturb_end
     %   d_p(2) = disturb_magnitude;
    %end

    if t == failure_step
        Q=Q3;
        
    end

    y_p = x_p;
    u_c = Q * y_p;
    dx_c = -alpha' .* x_c - beta' .* (x_c.^3) + u_c;

    if t > failure_step
        dx_c(4) = 0;
    end

    x_c = x_c + dt * dx_c;

    y_c = x_c - u_c;
    u_p = Q' * y_c;

    if t > failure_step
        u_p(4) = 0;
    end

    u_p = max(min(u_p, sat_limit), -sat_limit);
    dx_p = mu' .* u_p.^3;

    if t > failure_step
        x_p(4) = -10;
        dx_p(4) = 0;
    end

    x_p = x_p + dt * dx_p;

    x_p_traj(:, t) = x_p;
    x_c_traj(:, t) = x_c;
    u_p_traj(:, t) = u_p;
end

time = linspace(dt, T, steps);

figure;
set(gcf, 'Color', 'w');
plot(time, x_p_traj(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(time, x_p_traj(2,:), 'g-', 'LineWidth', 1.5);
plot(time, x_p_traj(3,:), 'b-', 'LineWidth', 1.5);
plot(time, x_p_traj(4,:), 'k-', 'LineWidth', 1.5);
%semilogx(time, x_p_traj(1,:), 'r-', 'LineWidth', 1.5); hold on;
%semilogx(time, x_p_traj(2,:), 'g-', 'LineWidth', 1.5);
%semilogx(time, x_p_traj(3,:), 'b-', 'LineWidth', 1.5);
%semilogx(time, x_p_traj(4,:), 'k-', 'LineWidth', 1.5);
xline(failure_time, 'm--', 'Node 4 Failed', ...
    'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', ...
    'LabelHorizontalAlignment', 'left');
xlabel('Time (s)');
ylabel('Plant Outputs');
legend('x_{p1}','x_{p2}', 'x_{p3}', 'x_{p4}', 'Location', 'best');
title('Plant Outputs Over Time (Linear Time Axis)');
grid on;

figure;
set(gcf, 'Color', 'w');
plot(time, u_p_traj(1,:), 'r-', 'LineWidth', 1.5); hold on;
plot(time, u_p_traj(2,:), 'g-', 'LineWidth', 1.5);
plot(time, u_p_traj(3,:), 'b-', 'LineWidth', 1.5);
plot(time, u_p_traj(4,:), 'k-', 'LineWidth', 1.5);
xline(failure_time, 'm--', 'Node 4 Failed', ...
    'LineWidth', 1.5, 'LabelVerticalAlignment', 'bottom', ...
    'LabelHorizontalAlignment', 'left');
yline(10, 'k--', 'u_{sat} = 10', 'LineWidth', 1.2);
yline(-10, 'k--', '-u_{sat} = -10', 'LineWidth', 1.2);
xlabel('Time (s)');
ylabel('Control Inputs u_p');
legend('u_{p1}', 'u_{p2}', 'u_{p3}', 'u_{p4}', 'Location', 'best');
title('Control Inputs Over Time (Linear Time Axis)');
grid on;
