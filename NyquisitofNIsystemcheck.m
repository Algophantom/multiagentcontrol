
num = [-0.044, 5.16, -1860.11, 54489.05];
den = [1, 86.15, 54490.53, 29510.63, 1481.6];

G = tf(num, den);


disp('UGV Transfer Function:')
G


% Nyquist plot
figure;
nyquist(G);
grid on;
title('Nyquist Plot of UGV Transfer Function');


K = tf(-5, [0.1 1]);


disp('Controller Transfer Function:')
K


figure;
nyquist(K);
grid on;
title('Nyquist Plot of Controller Transfer Function');

M0 = dcgain(G);
N0 = dcgain(K);
product_DC = M0 * N0
