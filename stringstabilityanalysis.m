%% Verify DC Gain for different kd values
kp =2;
kd_values = [3, 5, 7, 9, 10];

fprintf('k_p = 1.0\n');
fprintf('k_d\tω_peak\t|G|_max\t|G(0)|\tString Stable?\n');
fprintf('----------------------------------------\n');

for kd = kd_values
    s = tf('s');
    G = kp/(s^2 + kd*s + kp);
    
    % DC gain
    dc_gain = abs(evalfr(G, 0));
    
    % Find maximum magnitude
    w = logspace(-2, 2, 10000);
    mag = squeeze(bode(G, w));
    max_mag = max(mag);
    
    % Find peak frequency
    peak_idx = find(mag == max_mag, 1);
    w_peak = w(peak_idx);
    
    string_stable = (max_mag <= 1);
    
    fprintf('%.3f\t%.3f\t%.3f\t%.3f\t%s\n', ...
            kd, w_peak, max_mag, dc_gain, string_stable);
end