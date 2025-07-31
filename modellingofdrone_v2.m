clc; clear;close all;
s = tf('s');

%% === 1. Plant (SNI) ===
Px = (-0.044*s^3+5.16*s^2-1860.11*s+54489.05)/(s^4+86.15*s^3+54490.53*s^2+29510.63*s+1481.6);
Py = (-0.044*s^3+4.24*s^2-1494.74*s+48347.83)/(s^4+88.9*s^3+54483.53*s^2+49242.03*s+1266.25);
P_agent = blkdiag(Px, Py);
P = blkdiag(P_agent, P_agent, P_agent);  % 6x6

%% === 2. Controller (NI) ===
Kx =1000 / (1*s + 1);
Ky = 800 / (1*s + 1);
Kcx =1000 / (1*s + 1);
Kcy = 1000 / (1*s + 1);
K = blkdiag(Kx, Ky);  
K_f=blkdiag(Kcx,Kcy);
K_t=blkdiag(K,K_f,K_f);

%% === 3. Consensus matrices ===
Qa = [1 0; -1 1; 0 -1];
Qr = [1; 0; 0];
Qb = [0 0; -1 0; 0 -1];

Q1 = kron([Qa Qr], eye(2));    % 6x4
Q2 = kron([Qb Qr], eye(2));    % 6x4

%% === 4. Initial conditions and reference ===
x0 = [110; 90; 50; -90; 120; -100];  % initial position
r=[10;10] ; %reference for the leader
oner=[1;1;1];
Pr=kron(oner,r);       % desired position
Xf = zeros(6,1);                    % final formation offsets

%% === 5. Time setup ===
time = 0:10:100;
N = length(time);

%% === 6. Storage ===
y_hist = zeros(N, 6);
u_hist = zeros(N, 6);
e_hist=zeros(N,6);
y = x0;  
y_hist(1,:)=y; % initial output
dt=0.1;
%% === 7. Loop with output feedback ===
for k = 2:N
    % Error from desired position
    er = Pr - y;
    e_hist(k,:) = er';

    % Consensus-based projection
    ydash = Q1 * er;
    ef = Xf - ydash;
    udash = Q2' * er;

    % === Controller: Step response to udash ===
    u_input = repmat(udash', 2, 1);
    u = lsim(K_t, u_input, [0 dt]);
    u = u(end, :)';
    u_hist(k,:) = u';

    % === Plant: Step response to u ===
    u_for_plant = repmat(u', 2, 1);
    y_new = lsim(P, u_for_plant, [0 dt], y);
    y = y_new(end, :)';
    y_hist(k,:) = y';
end
%% === 8. Plot ===
figure;
hold on;
plot(y_hist(:,1), y_hist(:,2), 'r', 'DisplayName', 'Agent 1');
plot(y_hist(:,3), y_hist(:,4), 'g', 'DisplayName', 'Agent 2');
plot(y_hist(:,5), y_hist(:,6), 'b', 'DisplayName', 'Agent 3');
legend;
xlabel('x'); ylabel('y');
title('Agent 2D Trajectories');
axis equal; grid on;

figure;
plot(time, vecnorm(e_hist, 2, 2), 'LineWidth', 2);  % ||er||₂ over time
xlabel('Time (s)');
ylabel('||e_r||_2');
title('Consensus Error Norm vs Time');
grid on;