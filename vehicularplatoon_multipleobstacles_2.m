%% Vehicular Platoon with NI Consensus + Multiple Obstacles
clear; clc; close all;

T  = 1000;
dt = 0.01;
t  = 0:dt:T;
N  = length(t);

X       = zeros(6, N);
X(:,1)  = [-50; -50; -50; -50; -50; -50];%initial position of cars

u       = zeros(6, N);
u(1:2,1)= [0; 0];%initial velocities

udot    = zeros(6, 1);

d1 = 5;
d2 = 50;%inter-distance
v_lead = [10; 10];%velocityoftheleader
r      = v_lead .* t;%trajectory-set simulation purposes

obstacle_pos = [ 53   120  240;
                 50  80  242];%position of the obstacles
r_obs        = [20 10 50];%radius of the obstacles
n_obs        = size(obstacle_pos,2);

k = 3000; 
m = 2000;%chosen for realistic vehicle weight(say an SUV car)
avoid_state1 = [0;0];%avoid states intialised
avoid_state2 = [0;0];
avoid_state3 = [0;0];


c_avoid     = 2e3 ;%scaling of avoid velocities
c_avoid_2   = 1e4;

for i = 1:N-1
    v_des = v_lead;

    u_dash = [
    (X(1,i) - X(3,i));   
    (X(2,i) - X(4,i));        
    (X(3,i) - X(5,i));   
    (X(4,i) - X(6,i))        
];%relative position calculation through consensus graph
    %current graph follows : leader ->follower 1->follower 2---

    udot        = 2*u_dash - 15*u(3:6,i);%consensus NI controller
    %controller(s) = 1/(s+1)
    u(3:6,i+1)  = u(3:6,i) + dt*udot;%adding the change to the followers 

    pos1 = X(1:2,i);
    pos2 = X(3:4,i);
    pos3 = X(5:6,i);

    [avoid1, avoid_state1] = multi_obsavoidance(pos1, obstacle_pos, r_obs, avoid_state1, k, m, dt);%avoidance functions
    [avoid2,  avoid_state2] = multi_obsavoidance(pos2, obstacle_pos, r_obs, avoid_state2, k, m, dt);
    [avoid3,  avoid_state3] = multi_obsavoidance(pos3, obstacle_pos, r_obs, avoid_state3, k, m, dt);
    
    
    
    e_ref = r(:,i) - X(1:2,i);
    u(1:2,i+1) = v_des + c_avoid*avoid1;
   u(3:6,i+1) = u(3:6,i+1) + c_avoid_2*[avoid2; avoid3];%updated velocities with avoid states

    X(1:2,i+1) = X(1:2,i) + dt*u(1:2,i+1);%final position for the instant
    X(3:4,i+1) = X(3:4,i) + dt*u(3:4,i+1);%final position for the instant
    X(5:6,i+1) = X(5:6,i) + dt*u(5:6,i+1);%final position for the instant
end

%%avoid function

function [avoid_vel, avoid_state] = multi_obsavoidance(pos, obs_pos, r_obs, avoid_state, k, m, dt)
    n_obs = size(obs_pos,2);
    total_avoid = [0;0];
    %calculation of the overlap vector
    for j = 1:n_obs
        av = pos - obs_pos(:,j);
        d  = norm(av);
        safety=10;
        if d < (r_obs(j) + safety)%include a safety barrier
            %enter the if statement only if we are in range
            dir        = av / d;
            overlap    = (r_obs(j)+safety - d) * dir;
            
            avoid_state= 0.2*avoid_state + dt * (k/m) * overlap;%modelled as spring damper system(NI)
            total_avoid= total_avoid + avoid_state;
        end
    end

  
    avoid_vel = total_avoid;
end

%%animation 
figure; hold on; grid on; axis equal;
axis([-100 500 -100 500]);
xlabel('x'); ylabel('y');
title('Vehicular Platoon with NI Consensus + Multiple Obstacles');

theta = linspace(0,2*pi,80);
for j = 1:n_obs
    obs_x = obstacle_pos(1,j) + r_obs(j)*cos(theta);
    obs_y = obstacle_pos(2,j) + r_obs(j)*sin(theta);
    fill(obs_x, obs_y, [1 0.6 0.6], 'EdgeColor','r','FaceAlpha',0.6);
end

d1_plot  = plot(X(1,1), X(2,1), 'ro','MarkerFaceColor','r');
d2_plot  = plot(X(3,1), X(4,1), 'go','MarkerFaceColor','g');
d3_plot  = plot(X(5,1), X(6,1), 'bo','MarkerFaceColor','b');
ref_plot = plot(r(1,1), r(2,1), 'ws','MarkerFaceColor','k');

tr1 = animatedline('Color','r');
tr2 = animatedline('Color','g');
tr3 = animatedline('Color','b');

for i = 1:1:N
    set(d1_plot,'XData',X(1,i),'YData',X(2,i));
    set(d2_plot,'XData',X(3,i),'YData',X(4,i));
    set(d3_plot,'XData',X(5,i),'YData',X(6,i));
    set(ref_plot,'XData',r(1,i),'YData',r(2,i));

    addpoints(tr1,X(1,i),X(2,i));
    addpoints(tr2,X(3,i),X(4,i));
    addpoints(tr3,X(5,i),X(6,i));

    drawnow;
end
