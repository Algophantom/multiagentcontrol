Qa=[1 1;-1 0;0 -1];
Qr=[1;0;0];

Qi=[Qa Qr];
Qik=kron(Qi',eye(2))

Qb=[0 0;-1 0;0 -1];
Qj=[Qb Qr];
Qjk= kron(Qj,eye(2))

%% NI Consensus Controller Simulation (Explicit udot Dynamics)

T = 50; dt = 0.01;
t = 0:dt:T;
N = length(t);

% Initial states: [x1; y1; x1dot; y1dot; x2; y2; x2dot; y2dot; x3; y3; x3dot; y3dot]
X = zeros(6, N);
X(:,1) = [100; 90;-480; 208;-320; 192];

u = zeros(6, N);
udot = zeros(6, 1);

r = [10; 10];

 for i=1:N-1
    x1 = X(1,i); y1 = X(2,i);
    x2 = X(3,i); y2 = X(4,i);
    x3 = X(5,i); y3 = X(6,i);

    u_dash = [...
        r(1) - x1;
        r(2) - y1;
        x1 - x2;
        y1 - y2;
        x1 - x3;
        y1 - y3];

    udot=0.1*u_dash - u(:,i);
    u(:,i+1) = u(:,i) + dt * udot;
    


    X(:,i+1) = X(:,i) + dt * u(:,i);
end

figure; hold on; grid on; axis equal;
plot(X(1,:), X(2,:), 'r', 'LineWidth', 1.5);  % Drone 1
plot(X(3,:), X(4,:), 'g', 'LineWidth', 1.5);  % Drone 2
plot(X(5,:), X(6,:), 'b', 'LineWidth', 1.5); % Drone 3
scatter(r(1), r(2), 100, 'k', 'filled');
legend('Drone 1 (Leader)', 'Drone 2', 'Drone 3', 'Reference')
xlabel('x'); ylabel('y');
title('NI Consensus to Rendezvous (Explicit Controller Dynamics)');

