%% Vehicular Platoon with NI Consensus + Obstacle Avoidance
clear; clc;

% --- Graph structure (3 agents + leader)
Qa=[1 1;-1 0;0 -1];
Qr=[1;0;0];
Qi=[Qa Qr];
Qik=kron(Qi',eye(2));

Qb=[0 0;-1 0;0 -1];
Qj=[Qb Qr];
Qjk=kron(Qj,eye(2));

% --- Simulation setup
T = 1000; dt = 0.01;
t = 0:dt:T;
N = length(t);

% State vector [x1;y1; x2;y2; x3;y3]
X = zeros(6,N);
X(:,1) = [10;0; -5;10; -10;10];

u = zeros(6,N);      % velocity control
udot = zeros(6,1);   % controller dynamics

% Desired inter-vehicle spacings
d1 = 5; d2 = 5;

% Leader trajectory (straight line with slope)
v_lead = [1; 5];
r = v_lead .* t;

% --- Obstacle
obstacle_pos = [200; 100];   % fixed obstacle
r_obs = 15;                  % radius
k = 1; m = 3.5;              % NI virtual spring params

% Avoidance internal states
avoid_state1 = [0;0]; avoid_state2 = [0;0]; avoid_state3 = [0;0];
beta_self=0.7; beta_leader=0.3;

for i=1:N-1
    % Leader follows reference directly
    X(1:2,i+1) = r(:,i);

    % --- Consensus formation error (followers only)
    u_dash = [
        (X(1,i) - X(3,i) - d1);
        (X(2,i) - X(4,i));
        (X(3,i) - X(5,i) - d2);
        (X(4,i) - X(6,i))
    ];

    % --- NI controller dynamics (followers)
    udot = 1*u_dash - u(3:6,i);
    u(3:6,i+1) = u(3:6,i) + dt*udot;

    % --- Obstacle avoidance per agent
    pos1 = X(1:2,i); pos2 = X(3:4,i); pos3 = X(5:6,i);

    ref1 = u(1:2,i);   % leader vel
    ref2 = u(3:4,i);
    ref3 = u(5:6,i);

    [avoid1, avoid_state1] = obsavoidance(pos1, ref1, obstacle_pos, r_obs, avoid_state1, k, m, dt);
    [self2, avoid_state2] = obsavoidance(pos2, ref2, obstacle_pos, r_obs, avoid_state2, k, m, dt);
    [self3, avoid_state3] = obsavoidance(pos3, ref3, obstacle_pos, r_obs, avoid_state3, k, m, dt);

    avoid2 = beta_self*self2 + beta_leader*avoid1;
    avoid3 = beta_self*self3 + beta_leader*avoid1;

    avoid_vel = [avoid1; avoid2; avoid3];

    % --- Apply avoidance to followers
    u(:,i+1) = u(:,i+1) + 5*avoid_vel;

    % --- Update positions (followers)
    X(3:6,i+1) = X(3:6,i) + dt*u(3:6,i);
    X(5:6,i+1) = X(5:6,i) + dt*u(5:6,i);
end

%% Obstacle avoidance function
function [avoid_vel, avoid_state] = obsavoidance(pos, ref_vel, obs_pos, r_obs, avoid_state, k, m, dt)
    if norm(ref_vel)==0
        avoid_vel=[0;0]; return;
    end
    heading = ref_vel/norm(ref_vel);
    ac = obs_pos - pos;
    proj_len = dot(ac, heading);
    proj_v = proj_len*heading;
    closest = pos + proj_v;
    av = closest - obs_pos;
    d = norm(av);
    if d < r_obs
        overlap = (r_obs - d)*(av/d);
        avoid_state = avoid_state + dt*(k/m)*overlap;
        avoid_vel = avoid_state;
    else
        avoid_vel = [0;0];
    end
end

%% Animation
figure; hold on; grid on; axis equal;
axis([-50 600 -50 300]);
xlabel('x'); ylabel('y');
title('Vehicular Platoon with NI Consensus + Obstacle Avoidance');

% Obstacle
theta=linspace(0,2*pi,100);
obs_x=obstacle_pos(1)+r_obs*cos(theta);
obs_y=obstacle_pos(2)+r_obs*sin(theta);
fill(obs_x,obs_y,[1 0.6 0.6],'EdgeColor','r');

% Agents
d1_plot=plot(X(1,1),X(2,1),'ro','MarkerFaceColor','r');
d2_plot=plot(X(3,1),X(4,1),'go','MarkerFaceColor','g');
d3_plot=plot(X(5,1),X(6,1),'bo','MarkerFaceColor','b');
ref_plot=plot(r(1,1),r(2,1),'ks','MarkerFaceColor','k');

% Trails
tr1=animatedline('Color','r'); tr2=animatedline('Color','g'); tr3=animatedline('Color','b');

for i=1:10:N
    set(d1_plot,'XData',X(1,i),'YData',X(2,i));
    set(d2_plot,'XData',X(3,i),'YData',X(4,i));
    set(d3_plot,'XData',X(5,i),'YData',X(6,i));
    set(ref_plot,'XData',r(1,i),'YData',r(2,i));
    addpoints(tr1,X(1,i),X(2,i));
    addpoints(tr2,X(3,i),X(4,i));
    addpoints(tr3,X(5,i),X(6,i));
    drawnow;
end
