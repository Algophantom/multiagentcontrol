%% Vehicular Platoon with NI Consensus + Obstacle Avoidance
clear; clc;

Qa = [1 1; -1 0; 0 -1];
Qr = [1; 0; 0];
Qi = [Qa Qr];
Qik = kron(Qi', eye(2));

Qb = [0 0; -1 0; 0 -1];
Qj = [Qb Qr];
Qjk = kron(Qj, eye(2));

T  = 1000;
dt = 0.01;
t  = 0:dt:T;
N  = length(t);

X       = zeros(6, N);
X(:,1)  = [0; 10; -100; -100; -180; -180];

u       = zeros(6, N);
u(1:2,1)= [1; 5];

udot    = zeros(6, 1);

d1 = 100;
d2 = 100;

v_lead = [1; 5];
r      = v_lead .* t;

obstacle_pos = [50; 300];
r_obs        = 20;
k            = 100;
m            = 300.5;

avoid_state1 = [0; 0];
avoid_state2 = [0; 0];
avoid_state3 = [0; 0];

beta_self   = 0.9;
beta_leader = 0.1;

for i = 1:N-1
    
    v_des = v_lead;
    
    u_dash = [
        (X(1,i) - X(3,i));
        (X(2,i) - X(4,i));
        (X(3,i) - X(5,i));
        (X(4,i) - X(6,i))
    ];
    
    udot        = 5*u_dash - 2*u(3:6,i);
    u(3:6,i+1)  = u(3:6,i) + dt*udot;
    
    pos1 = X(1:2,i);
    pos2 = X(3:4,i);
    pos3 = X(5:6,i);
    
    ref1 = u(1:2,i);
    ref2 = u(3:4,i);
    ref3 = u(5:6,i);
    
    ref1_eff = (norm(ref1) > 1e-9) * ref1 + (norm(ref1) <= 1e-9) * v_des;
    
    [avoid1, avoid_state1] = obsavoidance(pos1, ref1_eff, obstacle_pos, r_obs, avoid_state1, k, m, dt);
    [self2,  avoid_state2] = obsavoidance(pos2, ref2,      obstacle_pos, r_obs, avoid_state2, k, m, dt);
    [self3,  avoid_state3] = obsavoidance(pos3, ref3,      obstacle_pos, r_obs, avoid_state3, k, m, dt);
    
    avoid2 = beta_self*self2 + beta_leader*avoid1;
    avoid3 = beta_self*self3 + beta_leader*avoid1;
    
    k_avoid    = 1000;
    u(1:2,i+1) = v_des + k_avoid*avoid1;
    u(3:6,i+1) = u(3:6,i+1) + k_avoid * [avoid2; avoid3];
    
    X(1:2,i+1) = X(1:2,i) + dt*u(1:2,i+1);
    X(3:4,i+1) = X(3:4,i) + dt*u(3:4,i+1);
    X(5:6,i+1) = X(5:6,i) + dt*u(5:6,i+1);
end

function [avoid_vel, avoid_state] = obsavoidance(pos, ~, obstacle_pos, r_obs, avoid_state, k, m, dt)
    av = pos - obstacle_pos;
    d  = norm(av);

    if d < r_obs
        dir        = av / max(d,1e-9);
        overlap    = (r_obs - d) * dir;
        avoid_state= avoid_state + dt * (k/m) * overlap;
        avoid_vel  = avoid_state;
    else
        avoid_state= [0;0];
        avoid_vel  = [0;0];
    end
end



figure; hold on; grid on; axis equal;
axis([-1000 1000 -1000 1000]);
xlabel('x'); ylabel('y');
title('Vehicular Platoon with NI Consensus + Obstacle Avoidance');

theta  = linspace(0,2*pi,100);
obs_x  = obstacle_pos(1) + r_obs*cos(theta);
obs_y  = obstacle_pos(2) + r_obs*sin(theta);
fill(obs_x, obs_y, [1 0.6 0.6], 'EdgeColor', 'r');

d1_plot  = plot(X(1,1), X(2,1), 'ro', 'MarkerFaceColor','r');
d2_plot  = plot(X(3,1), X(4,1), 'go', 'MarkerFaceColor','g');
d3_plot  = plot(X(5,1), X(6,1), 'bo', 'MarkerFaceColor','b');
ref_plot = plot(r(1,1), r(2,1), 'ks', 'MarkerFaceColor','k');

tr1 = animatedline('Color','r');
tr2 = animatedline('Color','g');
tr3 = animatedline('Color','b');

for i = 1:10:N
    set(d1_plot,'XData',X(1,i),'YData',X(2,i));
    set(d2_plot,'XData',X(3,i),'YData',X(4,i));
    set(d3_plot,'XData',X(5,i),'YData',X(6,i));
    set(ref_plot,'XData',r(1,i),'YData',r(2,i));
    
    addpoints(tr1,X(1,i),X(2,i));
    addpoints(tr2,X(3,i),X(4,i));
    addpoints(tr3,X(5,i),X(6,i));
    
    drawnow;
end
