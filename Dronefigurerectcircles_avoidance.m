%% NI Consensus Controller + Obstacle Avoidance 
clear; clc;

T = 1000; dt = 0.01;
t = 0:dt:T;
N = length(t);

% State vector: [x1; y1; x2; y2; x3; y3]
X = zeros(6, N);
X(:,1) = [102; 102; 102; 102; 103; 99];

% Formation velocity states
u = zeros(6, N);
u_form = zeros(6, N);
udot = zeros(6, 1);

% Avoidance velocity states per agent
avoid_state1 = [0; 0];
avoid_state2 = [0; 0];
avoid_state3 = [0; 0];

% Parameters
r = [100; 100];     % reference position for leader
r_obs = 0.1;        % obstacle radius
k =1;            % virtual spring constant
m = 3.5;           % "mass" for avoidance dynamics

beta_self   = 0.9;  
beta_leader = 0.1; 

obstacle_path = repmat([102; 100], 1, N);

% Leader circular path parameters
R_circ = 2;      % radius of leader's circular motion
omega  = 0.1;    % angular speed


    offset1 = [0; 2];               
    offset2 = [-sqrt(3); -1];       
    offset3 = [sqrt(3); -1];                 

for i = 1:N-1
    % Current positions
    x1 = X(1,i); y1 = X(2,i);
    x2 = X(3,i); y2 = X(4,i);
    x3 = X(5,i); y3 = X(6,i);
    theta = omega * t(i); 
    R = [cos(theta), -sin(theta); sin(theta), cos(theta)];
    %pf2 = [cos(0.25 * t(i)); sin(0.25 * t(i))];
    p1d = r + R * offset1;   % Desired position of Drone 1
    p2d = r + R * offset2;   % Desired position of Drone 2
    p3d = r + R * offset3;
    % --- Formation control (NI consensus output)
    u_dash = [ ...
        p1d(1) - x1;
        p1d(2) - y1;
        p2d(1) - x2;
        p2d(2) - y2;
        p3d(1) - x3;
        p3d(2) - y3];


    udot =  u_dash - u(:,i);
    u_form(:,i+1) = u_form(:,i) + dt * udot; 

    % --- Obstacle avoidance for each agent
    obstacle_pos = obstacle_path(:,i);

    % Agent 1
    pos1 = [x1; y1];
    ref_vel1 = u_form(1:2, i);
    [avoid_vel1, avoid_state1] = obsavoidance(pos1, ref_vel1, obstacle_pos, r_obs, avoid_state1, k, m, dt);

    % Agent 2
    pos2 = [x2; y2];
    ref_vel2 = u_form(3:4, i);
    [avoid_self2, avoid_state2] = obsavoidance(pos2, ref_vel2, obstacle_pos, r_obs, avoid_state2, k, m, dt);
    avoid_vel2 = beta_self * avoid_self2 + beta_leader * avoid_vel1;


    % Agent 3
    pos3 = [x3; y3];
    ref_vel3 = u_form(5:6, i);
    [avoid_self3, avoid_state3] = obsavoidance(pos3, ref_vel3, obstacle_pos, r_obs, avoid_state3, k, m, dt);
    avoid_vel3 = beta_self * avoid_self3 + beta_leader * avoid_vel1;


    % Stack avoidance velocities
    avoid_vel = [avoid_vel1; avoid_vel2; avoid_vel3];

    
    u(:,i+1) = u_form(:,i+1) + 10*avoid_vel;
    
    % --- Integrate positions
    X(:,i+1) = X(:,i) + dt *  u(:,i+1);
end



%% Obstacle avoidance function
function [avoid_vel, avoid_state] = obsavoidance(pos, ref_vel, obstacle_pos, r_obs, avoid_state, k, m, dt)
  
    heading = ref_vel / norm(ref_vel);

    % Vector from UAV to obstacle
    ac = obstacle_pos - pos;

    % Scalar projection of obstacle vector onto heading
    proj_len = dot(ac, heading);

    % Projection vector
    proj_v = proj_len * heading;

    % Closest point on heading to obstacle center
    closest = pos + proj_v;

    % Vector from obstacle center to closest point
    av = closest - obstacle_pos;
    d = norm(av);

    % If inside obstacle radius, compute avoidance
    if d < r_obs
        overlap = (r_obs - d) * (av / d);
        %avoid_state = avoid_state + dt * (k/m)*overlap;
        avoid_state = avoid_state + dt * ( -0.01*avoid_state + (k/m) * overlap );
        avoid_vel = avoid_state;
    else
        avoid_vel = [0; 0];
    end
end

%% Animation with traces
figure;
hold on; grid on; axis equal;
axis([95 105 95 105]); 
xlabel('X'); ylabel('Y');
title('3-Agent Formation with Obstacle Avoidance');

% Plot obstacle
theta = linspace(0, 2*pi, 50);
obs_x = obstacle_path(1,1) + r_obs*cos(theta);
obs_y = obstacle_path(2,1) + r_obs*sin(theta);
obs_plot = fill(obs_x, obs_y, [1 0.6 0.6], 'EdgeColor', 'r');

% Moving agent markers
agent1_plot = plot(X(1,1), X(2,1), 'bo', 'MarkerFaceColor', 'b');
agent2_plot = plot(X(3,1), X(4,1), 'go', 'MarkerFaceColor', 'g');
agent3_plot = plot(X(5,1), X(6,1), 'mo', 'MarkerFaceColor', 'm');

% Animated traces for each agent
trace1 = animatedline('Color','b','LineWidth',1);
trace2 = animatedline('Color','g','LineWidth',1);
trace3 = animatedline('Color','m','LineWidth',1);

legend('Obstacle','Agent 1','Agent 2','Agent 3');

% Animation loop
for i = 1:20:N
    
    obs_x = obstacle_path(1,i) + r_obs*cos(theta);
    obs_y = obstacle_path(2,i) + r_obs*sin(theta);
    set(obs_plot, 'XData', obs_x, 'YData', obs_y);

    
    set(agent1_plot, 'XData', X(1,i), 'YData', X(2,i));
    set(agent2_plot, 'XData', X(3,i), 'YData', X(4,i));
    set(agent3_plot, 'XData', X(5,i), 'YData', X(6,i));

   
    addpoints(trace1, X(1,i), X(2,i));
    addpoints(trace2, X(3,i), X(4,i));
    addpoints(trace3, X(5,i), X(6,i));

    drawnow;
end
