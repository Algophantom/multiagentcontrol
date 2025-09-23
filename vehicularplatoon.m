Qa=[1 1;-1 0;0 -1];
Qr=[1;0;0];

Qi=[Qa Qr];
Qik=kron(Qi',eye(2))

Qb=[0 0;-1 0;0 -1];
Qj=[Qb Qr];
Qjk= kron(Qj,eye(2))

%% NI Consensus Controller Simulation (Explicit udot Dynamics)

T = 10000; dt = 0.01;
t = 0:dt:T;
N = length(t);

% Initial states: [x1; y1; x1dot; y1dot; x2; y2; x2dot; y2dot; x3; y3; x3dot; y3dot]
X = zeros(6, N);
X(:,1) = [10; 0; -5; 10; -10; 10];

u = zeros(6, N);
udot = zeros(6, 1);

v_lead = [1; 5]; 
r = v_lead .* t; 

 for i=1:N-1
    x1 = X(1,i); y1 = X(2,i);
    x2 = X(3,i); y2 = X(4,i);
    x3 = X(5,i); y3 = X(6,i);
    

    pf2 = [1 + cos(0.25 * t(i)); -2.5 + sin(0.25 * t(i))];
    %pf3 = [1 - cos(0.25 * t(i)); 2.5 + sin(0.25 * t(i))];
    %pf2 = [1 + cos(0.25 * t(i)); sin(0.5*t(i))];

    %pf2 = [1 + cos(0.25 * t(i)); -2.5 + sin(0.25 * t(i))];
    %pf3 = [1 - cos(0.25 * t(i)); 2.5 + sin(0.25 * t(i))];
    pf2 = [1 + cos(0.25 * t(i)); sin(0.5*t(i))];

X(1:2,i+1) = r(:,i);  


u_dash = [
    (X(1,i) - X(3,i) - d1);
    (X(2,i) - X(4,i)-d1);
    (X(3,i) - X(5,i) - d2);
    (X(4,i) - X(6,i) - d2)];

udot = 1*u_dash - u(3:6,i);  % only for followers
u(3:6,i+1) = u(3:6,i) + dt*udot;
X(3:6,i+1) = X(3:6,i) + dt*u(3:6,i);


    
end

%% Animation of NI Consensus Platoon
figure; hold on; grid on; axis equal;
axis([-150 600 -100 300]); % adjust as needed
xlabel('x'); ylabel('y');
title('NI Consensus to Rendezvous (Explicit Controller Dynamics)');

% Moving markers for each agent
drone1_plot = plot(X(1,1), X(2,1), 'ro', 'MarkerFaceColor', 'r');
drone2_plot = plot(X(3,1), X(4,1), 'go', 'MarkerFaceColor', 'g');
drone3_plot = plot(X(5,1), X(6,1), 'bo', 'MarkerFaceColor', 'b');

% Reference marker
ref_plot = plot(r(1,1), r(2,1), 'ks', 'MarkerFaceColor','k');

% Animated traces
trace1 = animatedline('Color','r','LineWidth',1);
trace2 = animatedline('Color','g','LineWidth',1);
trace3 = animatedline('Color','b','LineWidth',1);

legend('Drone 1 (Leader)','Drone 2','Drone 3','Reference');

% Animation loop
for i = 1:10:N
    % Update drones
    set(drone1_plot,'XData',X(1,i),'YData',X(2,i));
    set(drone2_plot,'XData',X(3,i),'YData',X(4,i));
    set(drone3_plot,'XData',X(5,i),'YData',X(6,i));
    
    % Update reference
    set(ref_plot,'XData',r(1,i),'YData',r(2,i));
    
    % Update traces
    addpoints(trace1, X(1,i), X(2,i));
    addpoints(trace2, X(3,i), X(4,i));
    addpoints(trace3, X(5,i), X(6,i));
    
    drawnow;
end


