clear; clc; close all;

%% Simulation
T  = 80;
dt = 0.001;
t  = 0:dt:T;
N  = numel(t);

%% Road / lane definition
lane1_y = 0;
lane2_y = 50;
lane3_y = 100;
lane_y  = [lane1_y lane2_y lane3_y];

%% Obstacles
obstacle_pos = [120   420 630;
                lane1_y lane2_y lane3_y];
r_obs        = [15  5 20];
obs_vel      = [-8, -12, -9];
obs_vel_y    = [0, 0, 0];

%% Leader motion
v_x   = 16;
x_nom = v_x * t;

%% TurtleBot3 geometry
rw = 0.033;
b  = 0.160;
body_L = 0.138;
body_W = 0.178;

vis_scale = 80;
body_Lv = body_L * vis_scale;
body_Wv = body_W * vis_scale;
bv      = b * vis_scale;
wheel_len = 0.05 * vis_scale;

%% Platoon params
l = 1;
D12 = 20;
D23 = 40;
c_avoid = 1e4;

%% States
X = zeros(9,N);
X(:,1) = [-20; lane1_y; 0; -40; lane1_y; 0; -60; lane1_y; 0];

v = zeros(3,N);
w = zeros(3,N);
u = zeros(6,N);

omegaL = zeros(3,N);
omegaR = zeros(3,N);

%% Avoidance states
avoid_state1 = [0;0];
avoid_state2 = [0;0];
avoid_state3 = [0;0];
k_spring = 1;
m_mass = 15;

%% Lane switching
current_lane = 1;
lane_cooldown = 0;
cooldown_frames = 40;

obs_log = zeros(2,3,N);

%% main loop 
for k = 1:N-1
    for j = 1:3
        obstacle_pos(1,j) = obstacle_pos(1,j) + dt*obs_vel(j);
        obstacle_pos(2,j) = obstacle_pos(2,j) + dt*obs_vel_y(j);
    end
    obs_log(:,:,k) = obstacle_pos;

    p1 = X(1:2,k); phi1 = X(3,k);
    p2 = X(4:5,k); phi2 = X(6,k);
    p3 = X(7:8,k); phi3 = X(9,k);

    [a1,avoid_state1,danger1] = multi_obsavoidance(p1,obstacle_pos,r_obs,avoid_state1,k_spring,m_mass,dt);
    [a2,avoid_state2,~ ] = multi_obsavoidance(p2,obstacle_pos,r_obs,avoid_state2,k_spring,m_mass,dt);
    [a3,avoid_state3,~ ] = multi_obsavoidance(p3,obstacle_pos,r_obs,avoid_state3,k_spring,m_mass,dt);

    %% Lane switching trigger
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

    y_ref = lane_y(current_lane);
    r_used = [x_nom(k); y_ref];

    d1v=[cos(phi1);sin(phi1)];

    pt1=p1+l*d1v; 
    pt2=p2+l*[cos(phi2);sin(phi2)]; 
    pt3=p3+l*[cos(phi3);sin(phi3)];

    u_dash=[(r_used-pt1); 
        (r_used-pt2-D12*d1v); 
        (r_used-pt3-D23*d1v)];

    udot=100*u_dash-50*u(:,k);
    u(:,k+1)=u(:,k)+dt*udot;

    u(1:2,k+1)=u(1:2,k+1)+c_avoid*a1;
    u(3:4,k+1)=u(3:4,k+1)+c_avoid*a2;
    u(5:6,k+1)=u(5:6,k+1)+c_avoid*a3;

    v(1,k)=cos(phi1)*u(1,k)+sin(phi1)*u(2,k);
    w(1,k)=(-sin(phi1)*u(1,k)+cos(phi1)*u(2,k))/l;
    v(2,k)=cos(phi2)*u(3,k)+sin(phi2)*u(4,k);
    w(2,k)=(-sin(phi2)*u(3,k)+cos(phi2)*u(4,k))/l;
    v(3,k)=cos(phi3)*u(5,k)+sin(phi3)*u(6,k);
    w(3,k)=(-sin(phi3)*u(5,k)+cos(phi3)*u(6,k))/l;

    omegaR(:,k)=(v(:,k)+(b/2)*w(:,k))/rw;
    omegaL(:,k)=(v(:,k)-(b/2)*w(:,k))/rw;

    X(1,k+1)=X(1,k)+dt*v(1,k)*cos(phi1);
    X(2,k+1)=X(2,k)+dt*v(1,k)*sin(phi1);
    X(3,k+1)=wrapToPi(X(3,k)+dt*w(1,k));

    X(4,k+1)=X(4,k)+dt*v(2,k)*cos(phi2);
    X(5,k+1)=X(5,k)+dt*v(2,k)*sin(phi2);
    X(6,k+1)=wrapToPi(X(6,k)+dt*w(2,k));

    X(7,k+1)=X(7,k)+dt*v(3,k)*cos(phi3);
    X(8,k+1)=X(8,k)+dt*v(3,k)*sin(phi3);
    X(9,k+1)=wrapToPi(X(9,k)+dt*w(3,k));
end
obs_log(:,:,N)=obstacle_pos;

%% ----------------------- ANIMATION -------------------------
figure; hold on; grid on; axis equal;
axis([-200 1000 -200 200]);
xlabel('x'); ylabel('y');
title('Three-lane platoon: NI consensus + moving obstacles');

theta = linspace(0,2*pi,80);

obs = gobjects(1,3);
for j = 1:3
    obs(j) = fill(obs_log(1,j,1)+r_obs(j)*cos(theta), ...
                  obs_log(2,j,1)+r_obs(j)*sin(theta), ...
                  [1 0.6 0.6], 'EdgeColor','r','FaceAlpha',0.6);
end

tr1 = animatedline('Color','r','LineWidth',1);
tr2 = animatedline('Color','g','LineWidth',1);
tr3 = animatedline('Color','b','LineWidth',1);

rb1 = init_bot('r');
rb2 = init_bot('g');
rb3 = init_bot('b');

for k = 1:N
    for j = 1:3
        set(obs(j), ...
            'XData', obs_log(1,j,k) + r_obs(j)*cos(theta), ...
            'YData', obs_log(2,j,k) + r_obs(j)*sin(theta));
    end

    draw_bot(rb1, X(1:3,k), omegaL(1,k), omegaR(1,k), body_Lv, body_Wv, bv, wheel_len);
    draw_bot(rb2, X(4:6,k), omegaL(2,k), omegaR(2,k), body_Lv, body_Wv, bv, wheel_len);
    draw_bot(rb3, X(7:9,k), omegaL(3,k), omegaR(3,k), body_Lv, body_Wv, bv, wheel_len);

    addpoints(tr1, X(1,k), X(2,k));
    addpoints(tr2, X(4,k), X(5,k));
    addpoints(tr3, X(7,k), X(8,k));

    drawnow limitrate;
end

%% ---------------- FUNCTIONS ----------------
function h = init_bot(c)
    h.body = patch('XData',nan,'YData',nan,'FaceColor',c,'FaceAlpha',0.25,'EdgeColor',c,'LineWidth',1.5);
    h.ax   = plot(nan,nan,'k','LineWidth',2);
    h.wl   = plot(nan,nan,'k','LineWidth',4);
    h.wr   = plot(nan,nan,'k','LineWidth',4);
end

function draw_bot(h,X,omegaL,omegaR,L,W,b,wl)
    x = X(1); y = X(2); phi = X(3);

    ef = [cos(phi); sin(phi)];
    el = [-sin(phi); cos(phi)];
    p  = [x; y];

    c1 = p + ef*L/2 + el*W/2;
    c2 = p + ef*L/2 - el*W/2;
    c3 = p - ef*L/2 - el*W/2;
    c4 = p - ef*L/2 + el*W/2;

    corners = [c1 c2 c3 c4 c1];
    set(h.body,'XData',corners(1,:),'YData',corners(2,:));

    pL = p + el*b/2;
    pR = p - el*b/2;

    set(h.ax,'XData',[pL(1) pR(1)],'YData',[pL(2) pR(2)]);

    set(h.wl,'XData',[pL(1)-el(1)*wl/2 pL(1)+el(1)*wl/2], ...
             'YData',[pL(2)-el(2)*wl/2 pL(2)+el(2)*wl/2]);

    set(h.wr,'XData',[pR(1)-el(1)*wl/2 pR(1)+el(1)*wl/2], ...
             'YData',[pR(2)-el(2)*wl/2 pR(2)+el(2)*wl/2]);
end

function [avoid_vel, avoid_state, danger] = multi_obsavoidance(pos, obs_pos, r_obs, avoid_state, k, m, dt)
    n_obs = size(obs_pos,2);
    total_avoid  = [0;0];
    safety       = 20;
    danger       = 0;

    for j = 1:n_obs
        av = pos - obs_pos(:,j);
        d  = norm(av);

        if d < (r_obs(j) + safety)
            dir     = av / d;
            pen     = (r_obs(j) + safety - d);
            overlap = pen * dir;
            danger  = max(danger, pen);
            avoid_state = 0.1*avoid_state + dt*(k/m)*overlap;
            total_avoid = total_avoid + avoid_state;
        end
    end

    avoid_vel = total_avoid;
end
