%% leader-follower NI consensus controller for unicycle plant
clear; clc; close all;

%% Simulation
T  = 1000;                   % total time [s]
dt = 0.001;
t  = 0:dt:T;
N  = numel(t);

%% Params
l        = 1;            % look-ahead distance (m)
c_avoid    = 10000;            % scale for obstacle-avoidance virtual velocity (on \tilde p)

obstacle_pos = [  53  120  240 ;
                   50   80  242 ];
r_obs        = [  20   10   50 ];
n_obs        = size(obstacle_pos,2);


v_lead  = [40; 40];          
r       = v_lead .* t;      

%% States: [x1;y1;phi1; x2;y2;phi2; x3;y3;phi3]
X       = zeros(9, N);
X(:,1)  = [-50; -50; 0;   -55; 55; 0;   -60; 60; 0];  % initial poses

% logs
v  = zeros(3, N);            % linear speeds
w  = zeros(3, N);            % angular speeds
u  = zeros(6, N);            % virtual input on \tilde p: [u1x;u1y;u2x;u2y;u3x;u3y]


avoid_state1 = [0;0];
avoid_state2 = [0;0];
avoid_state3 = [0;0];
k_spring = 1; m_mass = 50;  

for k = 1:N-1
    
    p1   = X(1:2,k);  phi1 = X(3,k);
    p2   = X(4:5,k);  phi2 = X(6,k);
    p3   = X(7:8,k);  phi3 = X(9,k);

    pt1  = p1 + l*[cos(phi1); sin(phi1)];
    pt2  = p2 + l*[cos(phi2); sin(phi2)];
    pt3  = p3 + l*[cos(phi3); sin(phi3)];
    
    u_dash = [(r(:,k) - pt1);    
    (pt1 - pt2);   
    (pt2 - pt3) ];

    
    [avoid1, avoid_state1, on1] = multi_obsavoidance(p1, obstacle_pos, r_obs, avoid_state1, k_spring, m_mass, dt);
    [avoid2, avoid_state2, on2] = multi_obsavoidance(p2, obstacle_pos, r_obs, avoid_state2, k_spring, m_mass, dt);
    [avoid3, avoid_state3, on3] = multi_obsavoidance(p3, obstacle_pos, r_obs, avoid_state3, k_spring, m_mass, dt);
    
    udot        = 9*u_dash - 18*u(:,k);
    u(:,k+1)  = u(:,k) + dt*udot;

    
    u(1:2,k+1) = u(1:2,k+1) + c_avoid * avoid1;
    u(3:6,k+1) = u(3:6,k+1) + c_avoid * [avoid2; avoid3];

   

    v(1,k) =  cos(phi1)*u(1,k) + sin(phi1)*u(2,k);
    w(1,k) = (-sin(phi1)*u(1,k) + cos(phi1)*u(2,k)) / l;

    v(2,k) =  cos(phi2)*u(3,k) + sin(phi2)*u(4,k);
    w(2,k) = (-sin(phi2)*u(3,k) + cos(phi2)*u(4,k)) /l;

    v(3,k) =  cos(phi3)*u(5,k) + sin(phi3)*u(6,k);
    w(3,k) = (-sin(phi3)*u(5,k) + cos(phi3)*u(6,k)) /l;

    
 
    X(1,k+1) = X(1,k) + dt * v(1,k) * cos(phi1);
    X(2,k+1) = X(2,k) + dt * v(1,k) * sin(phi1);
    X(3,k+1) = wrapToPi( X(3,k) + dt * w(1,k) );

    X(4,k+1) = X(4,k) + dt * v(2,k) * cos(phi2);
    X(5,k+1) = X(5,k) + dt * v(2,k) * sin(phi2);
    X(6,k+1) = wrapToPi( X(6,k) + dt * w(2,k) );

    X(7,k+1) = X(7,k) + dt * v(3,k) * cos(phi3);
    X(8,k+1) = X(8,k) + dt * v(3,k) * sin(phi3);
    X(9,k+1) = wrapToPi( X(9,k) + dt * w(3,k) );
end

%% Animation
figure; hold on; grid on; axis equal;
axis([-100 500 -100 500]);
xlabel('x'); ylabel('y');
title('NI consensus and obstacle avoidance on vehicular platoon');

% Draw obstacles
theta = linspace(0,2*pi,120);
for j = 1:n_obs
    fill(obstacle_pos(1,j)+r_obs(j)*cos(theta), ...
         obstacle_pos(2,j)+r_obs(j)*sin(theta), ...
         [1 0.6 0.6], 'EdgeColor','r','FaceAlpha',0.6);
end

% Vehicle plots
d1_plot  = plot(X(1,1), X(2,1), 'ro','MarkerFaceColor','r','DisplayName','Leader');
d2_plot  = plot(X(4,1), X(5,1), 'go','MarkerFaceColor','g','DisplayName','Follower 1');
d3_plot  = plot(X(7,1), X(8,1), 'bo','MarkerFaceColor','b','DisplayName','Follower 2');

arrow_len = 10;   % length of heading arrow in metres
h1 = quiver(X(1,1), X(2,1), arrow_len*cos(X(3,1)), arrow_len*sin(X(3,1)), 0, ...
            'Color','r','LineWidth',1.5,'MaxHeadSize',2);
h2 = quiver(X(4,1), X(5,1), arrow_len*cos(X(6,1)), arrow_len*sin(X(6,1)), 0, ...
            'Color','g','LineWidth',1.5,'MaxHeadSize',2);
h3 = quiver(X(7,1), X(8,1), arrow_len*cos(X(9,1)), arrow_len*sin(X(9,1)), 0, ...
            'Color','b','LineWidth',1.5,'MaxHeadSize',2);

% Trajectories (no legend entries)
tr1 = animatedline('Color','r','HandleVisibility','off');
tr2 = animatedline('Color','g','HandleVisibility','off');
tr3 = animatedline('Color','b','HandleVisibility','off');

% Reference (no legend entry)
ref_plot = plot(r(1,1), r(2,1), 'ks','MarkerFaceColor','k','HandleVisibility','off');

legend('Location','northwest');

% Animation loop
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
    set(ref_plot,'XData',r(1,k),'YData',r(2,k));
    
    addpoints(tr1, X(1,k), X(2,k));
    addpoints(tr2, X(4,k), X(5,k));
    addpoints(tr3, X(7,k), X(8,k));
    drawnow limitrate;
end

%% Obstacle-avoidance helper (NI "spring-damper" in discrete-time)
function [avoid_vel, avoid_state, avoid_active] = multi_obsavoidance(pos, obs_pos, r_obs, avoid_state, k, m, dt)
    n_obs = size(obs_pos,2);
    total_avoid  = [0;0];
    avoid_active = false;
    safety       = 20;                   % safety margin
    for j = 1:n_obs
        av = pos - obs_pos(:,j);% vector points from obstacle center to the robot, outwards
        d  = norm(av);
        if d < (r_obs(j) + safety)
            avoid_active = true;
            dir      = (av) / d;
            overlap  = (r_obs(j)+safety - d) * dir;     % overlap vector
            avoid_state = 0.1*(1-dt)*avoid_state + dt*(k/m) * overlap;         % NI model (discrete)
            total_avoid = total_avoid + avoid_state;
        end
    end
    avoid_vel = total_avoid;
end
