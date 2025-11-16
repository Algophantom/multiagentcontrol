%% Two-lane platoon with improved frame-based lane switching
clear; clc; close all;

T  = 80;                
dt = 0.01;
t  = 0:dt:T;
N  = numel(t);

%% Road / lane definition
lane1_y = 0;             % lower lane
lane2_y = 50;            % upper lane
lane_y  = [lane1_y lane2_y];

%% Obstacles: [x; y]
obstacle_pos = [120   320;
                lane1_y lane2_y];
r_obs        = [15  2];
n_obs        = size(obstacle_pos,2);

%% Leader nominal motion in x
v_x   = 16;                          % m/s
x_nom = v_x * t;                     % straight in x

%% Platoon / vehicle params
l        = 1;        % unicycle "wheelbase" for tip point
D12      = 15;       % spacing: leader -> follower 1
D23      = 25;       % spacing: follower1 -> follower2
c_avoid  = 2e4;      % NI avoidance scaling

%% State: [x1;y1;phi1; x2;y2;phi2; x3;y3;phi3]
X      = zeros(9,N);
X(:,1) = [-20; lane1_y;  0;   -40; lane1_y; 0;   -60; lane1_y; 0];

v  = zeros(3,N);
w  = zeros(3,N);
u  = zeros(6,N);       % NI consensus states

%% Obstacle-avoid variables with individual frame cooldowns
avoid_state1 = [0;0]; frame1_cooldown = 0;
avoid_state2 = [0;0]; frame2_cooldown = 0; 
avoid_state3 = [0;0]; frame3_cooldown = 0;

k_spring     = 1;
m_mass       = 50;

%% Lane switching - individual lane states
current_lane = [1; 1; 1];  % Each vehicle has its own lane
r_used_log   = zeros(2,N); 
cooldown_frames = 100;     % ~1 second cooldown (100 * 0.01s)

for k = 1:N-1
    % Update frame cooldowns
    frame1_cooldown = max(0, frame1_cooldown - 1);
    frame2_cooldown = max(0, frame2_cooldown - 1);
    frame3_cooldown = max(0, frame3_cooldown - 1);

    p1   = X(1:2,k);  phi1 = X(3,k);
    p2   = X(4:5,k);  phi2 = X(6,k);
    p3   = X(7:8,k);  phi3 = X(9,k);
    
    % Individual obstacle avoidance with cooldown tracking
    [avoid1, avoid_state1, new_lane1, new_frame1] = improved_obsavoidance(...
        p1, obstacle_pos, r_obs, avoid_state1, current_lane(1), frame1_cooldown, k_spring, m_mass, dt);
    
    [avoid2, avoid_state2, new_lane2, new_frame2] = improved_obsavoidance(...
        p2, obstacle_pos, r_obs, avoid_state2, current_lane(2), frame2_cooldown, k_spring, m_mass, dt);
    
    [avoid3, avoid_state3, new_lane3, new_frame3] = improved_obsavoidance(...
        p3, obstacle_pos, r_obs, avoid_state3, current_lane(3), frame3_cooldown, k_spring, m_mass, dt);
    
    % Update lanes and cooldowns
    if new_frame1 > 0 && frame1_cooldown == 0
        current_lane(1) = new_lane1;
        frame1_cooldown = cooldown_frames;
    end
    if new_frame2 > 0 && frame2_cooldown == 0
        current_lane(2) = new_lane2;
        frame2_cooldown = cooldown_frames;
    end
    if new_frame3 > 0 && frame3_cooldown == 0
        current_lane(3) = new_lane3;
        frame3_cooldown = cooldown_frames;
    end

    % Individual references for each vehicle
    r1 = [x_nom(k); lane_y(current_lane(1))];
    r2 = [x_nom(k); lane_y(current_lane(2))];
    r3 = [x_nom(k); lane_y(current_lane(3))];
    r_used_log(:,k) = r1;  % Log leader's reference
    
    % Direction vectors
    d1 = [cos(phi1); sin(phi1)];
    
    % Tip points
    pt1 = p1 + l*[cos(phi1); sin(phi1)];
    pt2 = p2 + l*[cos(phi2); sin(phi2)];
    pt3 = p3 + l*[cos(phi3); sin(phi3)];    

    % NI consensus control
    u_dash = [ (r1 - pt1);   
               (r2 - pt2 - D12 * d1);   
               (r3 - pt3 - D23 * d1) ];  

    % NI filter
    udot      = 100*u_dash - 100*u(:,k);
    u(:,k+1)  = u(:,k) + dt*udot;

    % Add avoidance forces
    u(1:2,k+1) = u(1:2,k+1) + c_avoid*avoid1;
    u(3:4,k+1) = u(3:4,k+1) + c_avoid*avoid2;
    u(5:6,k+1) = u(5:6,k+1) + c_avoid*avoid3;

    % Convert to unicycle controls
    v(1,k) =  cos(phi1)*u(1,k) + sin(phi1)*u(2,k);
    w(1,k) = (-sin(phi1)*u(1,k) + cos(phi1)*u(2,k)) / l;

    v(2,k) =  cos(phi2)*u(3,k) + sin(phi2)*u(4,k);
    w(2,k) = (-sin(phi2)*u(3,k) + cos(phi2)*u(4,k)) / l;

    v(3,k) =  cos(phi3)*u(5,k) + sin(phi3)*u(6,k);
    w(3,k) = (-sin(phi3)*u(5,k) + cos(phi3)*u(6,k)) / l;

    % Integrate dynamics
    X(1,k+1) = X(1,k) + dt*v(1,k)*cos(phi1);
    X(2,k+1) = X(2,k) + dt*v(1,k)*sin(phi1);
    X(3,k+1) = wrapToPi(X(3,k) + dt*w(1,k));

    X(4,k+1) = X(4,k) + dt*v(2,k)*cos(phi2);
    X(5,k+1) = X(5,k) + dt*v(2,k)*sin(phi2);
    X(6,k+1) = wrapToPi(X(6,k) + dt*w(2,k));

    X(7,k+1) = X(7,k) + dt*v(3,k)*cos(phi3);
    X(8,k+1) = X(8,k) + dt*v(3,k)*sin(phi3);
    X(9,k+1) = wrapToPi(X(9,k) + dt*w(3,k));
end

% log last reference point
r_used_log(:,N) = [x_nom(N); lane_y(current_lane(1))];

%% -------- Animation --------
figure; hold on; grid on; axis equal;
axis([-100 500 -100 100]);
xlabel('x'); ylabel('y');
title('Two-lane platoon: NI consensus with cooldown-based lane switching');

theta = linspace(0,2*pi,120);
for j = 1:n_obs
    fill(obstacle_pos(1,j)+r_obs(j)*cos(theta), ...
         obstacle_pos(2,j)+r_obs(j)*sin(theta), ...
         [1 0.6 0.6], 'EdgeColor','r','FaceAlpha',0.6);
end

% lanes
plot([-50 350],[lane1_y lane1_y],'k--');
plot([-50 350],[lane2_y lane2_y],'k--');

d1_plot  = plot(X(1,1), X(2,1), 'ro','MarkerFaceColor','r','DisplayName','Leader');
d2_plot  = plot(X(4,1), X(5,1), 'go','MarkerFaceColor','g','DisplayName','Follower 1');
d3_plot  = plot(X(7,1), X(8,1), 'bo','MarkerFaceColor','b','DisplayName','Follower 2');

arrow_len = 5;
h1 = quiver(X(1,1), X(2,1), arrow_len*cos(X(3,1)), arrow_len*sin(X(3,1)), 0, ...
            'Color','r','LineWidth',1.5,'MaxHeadSize',2);
h2 = quiver(X(4,1), X(5,1), arrow_len*cos(X(6,1)), arrow_len*sin(X(6,1)), 0, ...
            'Color','g','LineWidth',1.5,'MaxHeadSize',2);
h3 = quiver(X(7,1), X(8,1), arrow_len*cos(X(9,1)), arrow_len*sin(X(9,1)), 0, ...
            'Color','b','LineWidth',1.5,'MaxHeadSize',2);

tr1 = animatedline('Color','r','HandleVisibility','off');
tr2 = animatedline('Color','g','HandleVisibility','off');
tr3 = animatedline('Color','b','HandleVisibility','off');

ref_plot = plot(r_used_log(1,1), r_used_log(2,1), 'ks','MarkerFaceColor','k','HandleVisibility','off');

legend('Location','northwest');

for k = 1:50:N  % Show every 50th frame for smoother animation
    set(d1_plot,'XData',X(1,k),'YData',X(2,k));
    set(d2_plot,'XData',X(4,k),'YData',X(5,k));
    set(d3_plot,'XData',X(7,k),'YData',X(8,k));
    
    set(h1,'XData',X(1,k),'YData',X(2,k), ...
           'UData',arrow_len*cos(X(3,k)),'VData',arrow_len*sin(X(3,k)));
    set(h2,'XData',X(4,k),'YData',X(5,k), ...
           'UData',arrow_len*cos(X(6,k)),'VData',arrow_len*sin(X(6,k)));
    set(h3,'XData',X(7,k),'YData',X(8,k), ...
           'UData',arrow_len*cos(X(9,k)),'VData',arrow_len*sin(X(9,k)));
    set(ref_plot,'XData',r_used_log(1,k),'YData',r_used_log(2,k));
    
    addpoints(tr1, X(1,k), X(2,k));
    addpoints(tr2, X(4,k), X(5,k));
    addpoints(tr3, X(7,k), X(8,k));
    drawnow;
end

%% -------- Improved Obstacle Avoidance Function --------
function [avoid_vel, avoid_state, new_lane, trigger_frames] = improved_obsavoidance(pos, obs_pos, r_obs, avoid_state, current_lane, cooldown_remaining, k, m, dt)
    n_obs = size(obs_pos,2);
    total_avoid = [0;0];
    new_lane = current_lane;
    trigger_frames = 0;
    safety = 30;
    
    for j = 1:n_obs
        av = pos - obs_pos(:,j);
        d = norm(av);
        
        % Continuous avoidance force (always active when close)
        if d < (r_obs(j) + safety)
            dir = av / d;
            overlap = (r_obs(j) + safety - d) * dir;
            avoid_state = 0.1 * avoid_state + dt * (k/m) * overlap;
            total_avoid = total_avoid + avoid_state;
            
            % Event-triggered lane switching (only when close and cooldown expired)
            if d < (r_obs(j) + safety * 0.7) && cooldown_remaining == 0
                new_lane = 3 - current_lane; % Toggle between 1 and 2
                trigger_frames = 1; % Signal that lane change should occur
            end
        end
    end
    avoid_vel = total_avoid;
end