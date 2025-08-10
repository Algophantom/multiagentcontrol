Qa=[1 1;-1 0;0 -1];
Qr=[1;0;0];

Qi=[Qa Qr];
Qik=kron(Qi',eye(2))

Qb=[0 0;-1 0;0 -1];
Qj=[Qb Qr];
Qjk= kron(Qj,eye(2))

%% NI Consensus Controller Simulation (Explicit udot Dynamics)

T = 100; dt = 0.01;
t = 0:dt:T;
N = length(t);


X = zeros(6, N);
X(:,1) = [100; 100;98; 100;102; 100];

u = zeros(6, N);
udot = zeros(6, 1);

r = [100; 100];

 for i=1:N-1
    x1 = X(1,i); y1 = X(2,i);
    x2 = X(3,i); y2 = X(4,i);
    x3 = X(5,i); y3 = X(6,i);
    
    %pf2 = [1 + cos(0.25 * t(i)); -2.5 + sin(0.25 * t(i))];
    pf3 = [2,];
    pf2 = [1 + cos(0.25 * t(i)); sin(0.5*t(i))];
    %pf2 = [t(i)+2*(cos(t(i)))^2;t(i)];
   
    u_dash = [...
        (r(1) + pf2(1)) - x1;%relative distance + formation
        (r(2) + pf2(2)) - y1;
            x1  - x2;
            y1 - y2 ;
           (x1) - x3 ;
           (y1) - y3];

    udot=2*u_dash - u(:,i);
    u(:,i+1) = u(:,i) + dt * udot;
    
    X(:,i+1) = X(:,i) + dt * u(:,i);
end

% Animation
figure; axis equal; grid on; hold on;
xlim([min(X(1,:))-5, max(X(1,:))+5]);
ylim([min(X(2,:))-5, max(X(2,:))+5]);
xlabel('x'); ylabel('y');
title('NI Consensus Formation Animation');

h1 = animatedline('Color','r','LineWidth',1.5); % Drone 1
h2 = animatedline('Color','g','LineWidth',1.5); % Drone 2
h3 = animatedline('Color','b','LineWidth',1.5); % Drone 3
scatter(r(1), r(2), 100, 'k', 'filled');         % Reference

drone1 = plot(X(1,1), X(2,1), 'ro', 'MarkerFaceColor','r');
drone2 = plot(X(3,1), X(4,1), 'go', 'MarkerFaceColor','g');
drone3 = plot(X(5,1), X(6,1), 'bo', 'MarkerFaceColor','b');

for i = 1:1:N
    % Add points to trail
    addpoints(h1, X(1,i), X(2,i));
    addpoints(h2, X(3,i), X(4,i));
    addpoints(h3, X(5,i), X(6,i));

    % Move drones
    set(drone1, 'XData', X(1,i), 'YData', X(2,i));
    set(drone2, 'XData', X(3,i), 'YData', X(4,i));
    set(drone3, 'XData', X(5,i), 'YData', X(6,i));

    drawnow;
end

legend('Drone 1 (Leader)', 'Drone 2', 'Drone 3', 'Reference');

desired_pf = [1 + cos(0.25 * t); -2.5 + sin(0.25 * t)];
plot(r(1)+desired_pf(1,:), r(2)+desired_pf(2,:), 'k--')