%% NI Consensus Controller Simulation (Explicit udot Dynamics)

T = 8; dt = 0.001;
t = 0:dt:T;
N = length(t);

% Initial states: [x1; y1; x1dot; y1dot; x2; y2; x2dot; y2dot; x3; y3; x3dot; y3dot]
X = zeros(12, N);
X(:,1) = [10; 10; -10; -10; -4; 20; 10; 10; -32; 12; -10; 10];

u = zeros(6, N);
udot = zeros(6, 1);

r = [100; 100];

for i=1:N-1
    x1 = X(1,i); y1 = X(2,i);
    x2 = X(5,i); y2 = X(6,i);
    x3 = X(9,i); y3 = X(10,i);
    u_dash = [...
        r(1) - x1;
        r(2) - y1;
        x1 - x2;
        y1 - y2;
        x1 - x3;
        y1 - y3];
    udot=2*u_dash - u(:,i);
    u(:,i+1) = max(min(u(:,i+1), 20), -20);
    u(:,i+1) = u(:,i) + dt * udot;
    dx = zeros(12,1);
    dx(1) = X(3,i);        dx(2) = X(4,i);
    dx(3) = u(1,i);        dx(4) = u(2,i);

    dx(5) = X(7,i);        dx(6) = X(8,i);
    dx(7) = u(3,i);        dx(8) = u(4,i);

    dx(9) = X(11,i);       dx(10) = X(12,i);
    dx(11) = u(5,i);       dx(12) = u(6,i);

    X(:,i+1) = X(:,i) + dt * dx;
end

figure; hold on; grid on; axis equal;
plot(X(1,:), X(2,:), 'r', 'LineWidth', 1.5);  % Drone 1
plot(X(5,:), X(6,:), 'g', 'LineWidth', 1.5);  % Drone 2
plot(X(9,:), X(10,:), 'b', 'LineWidth', 1.5); % Drone 3
scatter(r(1), r(2), 100, 'k', 'filled');
legend('Drone 1 (Leader)', 'Drone 2', 'Drone 3', 'Reference')
xlabel('x'); ylabel('y');
title('NI Consensus to Rendezvous (Explicit Controller Dynamics)');