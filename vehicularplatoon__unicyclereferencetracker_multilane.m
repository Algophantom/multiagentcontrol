%% Two-lane platoon with NI consensus, lane switching and obstacle avoidance
clear; clc; close all;

%% Simulation
T  = 80;                
dt = 0.01;
t  = 0:dt:T;
N  = numel(t);

%% Road / lane definition
lane1_y = 0;             
lane2_y = 50;    
lane3_y = 100;
lane_y  = [lane1_y lane2_y lane3_y];

%% Obstacles: [x; y]
obstacle_pos = [120   320 330;
                lane1_y lane2_y lane3_y];
r_obs        = [15  5 10];

%% Leader nominal motion in x
v_x   = 40;
x_nom = v_x * t;
r_nom = [x_nom; lane1_y*ones(1,N)];

%% Platoon params
l        = 1;        
D12      = 15;      
D23      = 25;      
c_avoid  = 2e3;      

%% States: [leader; follower1; follower2]
X      = zeros(9,N);
X(:,1) = [-20; lane1_y;  0;   
          -40; lane2_y;  0;   
          -60; lane3_y;  0];

v  = zeros(3,N);
w  = zeros(3,N);
u  = zeros(6,N);       

%% Obstacle avoid states
avoid_state1 = [0;0];
avoid_state2 = [0;0];
avoid_state3 = [0;0];
k_spring     = 1;
m_mass       = 15;

%% Lane switching FSM
current_lane     = 1;
lane_cooldown    = 0;

cooldown_frames  = 40;     

r_used_log = zeros(2,N);

%% Main loop
for k = 1:N-1

    p1   = X(1:2,k);  phi1 = X(3,k);
    p2   = X(4:5,k);  phi2 = X(6,k);
    p3   = X(7:8,k);  phi3 = X(9,k);

    %% NI avoidance forces
    [avoid1, avoid_state1, danger1] = multi_obsavoidance(p1, obstacle_pos, r_obs, avoid_state1, k_spring, m_mass, dt);
    [avoid2, avoid_state2, ~]       = multi_obsavoidance(p2, obstacle_pos, r_obs, avoid_state2, k_spring, m_mass, dt);
    [avoid3, avoid_state3, ~]       = multi_obsavoidance(p3, obstacle_pos, r_obs, avoid_state3, k_spring, m_mass, dt);
    
   
    if lane_cooldown > 0
        lane_cooldown = lane_cooldown - 1;
    else
        if danger1 > 2
            if current_lane == 1
                current_lane = 2;
            elseif current_lane == 2
                current_lane = 3;
            else
                current_lane = 2;
            end
            lane_cooldown = cooldown_frames;
        end
    end

    %% Set reference lane
    y_ref = lane_y(current_lane);
    r_used = [x_nom(k); y_ref];
    r_used_log(:,k) = r_used;

    %% Spacing definition
    d1 = [cos(phi1); sin(phi1)];

    pt1 = p1 + l*d1;
    pt2 = p2 + l*[cos(phi2); sin(phi2)];
    pt3 = p3 + l*[cos(phi3); sin(phi3)];

    %% Consensus error (NI)
    u_dash = [ (r_used - pt1);   
               (r_used - pt2 - D12*d1);   
               (r_used - pt3 - D23*d1) ];

    %% NI controller dynamics
    udot     = 100*u_dash - 100*u(:,k);
    u(:,k+1) = u(:,k) + dt*udot;

    %% Add NI avoidance
    u(1:2,k+1) = u(1:2,k+1) + c_avoid*avoid1;
    u(3:4,k+1) = u(3:4,k+1) + c_avoid*avoid2;
    u(5:6,k+1) = u(5:6,k+1) + c_avoid*avoid3;

    %% Unicycle mapping
    v(1,k) =  cos(phi1)*u(1,k) + sin(phi1)*u(2,k);
    w(1,k) = (-sin(phi1)*u(1,k) + cos(phi1)*u(2,k)) / l;

    v(2,k) =  cos(phi2)*u(3,k) + sin(phi2)*u(4,k);
    w(2,k) = (-sin(phi2)*u(3,k) + cos(phi2)*u(4,k)) / l;

    v(3,k) =  cos(phi3)*u(5,k) + sin(phi3)*u(6,k);
    w(3,k) = (-sin(phi3)*u(5,k) + cos(phi3)*u(6,k)) / l;

    %% Integrate
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

r_used_log(:,N) = [x_nom(N); lane_y(current_lane)];

%% -------- Animation --------
figure; hold on; grid on; axis equal;
axis([-200 1000 -200 200]);
xlabel('x'); ylabel('y');
title('Two-lane platoon: NI consensus + lane switching');

theta = linspace(0,2*pi,120);
for j = 1:3
    fill(obstacle_pos(1,j)+r_obs(j)*cos(theta), ...
         obstacle_pos(2,j)+r_obs(j)*sin(theta), ...
         [1 0.6 0.6],'EdgeColor','r','FaceAlpha',0.6);
end

plot([-50 350],[lane1_y lane1_y],'k--');
plot([-50 350],[lane2_y lane2_y],'k--');
plot([-50 350],[lane3_y lane3_y],'k--');

d1_plot  = plot(X(1,1), X(2,1),'ro','MarkerFaceColor','r');
d2_plot  = plot(X(4,1), X(5,1),'go','MarkerFaceColor','g');
d3_plot  = plot(X(7,1), X(8,1),'bo','MarkerFaceColor','b');

arrow_len = 5;
h1 = quiver(X(1,1), X(2,1), arrow_len*cos(X(3,1)), arrow_len*sin(X(3,1)),0,'r','LineWidth',1.3);
h2 = quiver(X(4,1), X(5,1), arrow_len*cos(X(6,1)), arrow_len*sin(X(6,1)),0,'g','LineWidth',1.3);
h3 = quiver(X(7,1), X(8,1), arrow_len*cos(X(9,1)), arrow_len*sin(X(9,1)),0,'b','LineWidth',1.3);

tr1 = animatedline('Color','r');
tr2 = animatedline('Color','g');
tr3 = animatedline('Color','b');

ref_plot = plot(r_used_log(1,1), r_used_log(2,1),'ks','MarkerFaceColor','k');

for k = 1:N
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

    addpoints(tr1,X(1,k),X(2,k));
    addpoints(tr2,X(4,k),X(5,k));
    addpoints(tr3,X(7,k),X(8,k));

    drawnow limitrate;
end

%% -------- NI Obstacle avoidance --------
function [avoid_vel, avoid_state, danger] = multi_obsavoidance(pos, obs_pos, r_obs, avoid_state, k, m, dt)

    n_obs = size(obs_pos,2);
    total_avoid  = [0;0];
    safety       = 10;
    danger       = 0;     

    for j = 1:n_obs
        av = pos - obs_pos(:,j);
        d  = norm(av);
        

        if d < (r_obs(j) + safety)
            dir     = av / d;
            pen     = (r_obs(j) + safety - d);   
            overlap = pen * dir;
            danger = max(danger,pen);
            

            avoid_state = 0.1*avoid_state + dt*(k/m)*overlap;
            total_avoid = total_avoid + avoid_state;
        end
    end

    avoid_vel = total_avoid;
end
