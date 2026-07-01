%% Manual PID Tuning Helper
% This script loads simulation results and provides interactive sliders
% to tune PID gains for each axis.
clear; close all; clc;

% Load simulation results
if exist('gimbal_simulation_results_real_motors.mat', 'file')
    load('gimbal_simulation_results_real_motors.mat');
elseif exist('gimbal_simulation_results.mat', 'file')
    load('gimbal_simulation_results.mat');
else
    error('Please run run_gimbal_simulation.m first to generate simulation results.');
end

% Verify required variables exist
required_vars = {'t', 'angle_log', 'ref_angles', 'J', 'D', 'K_motor', 'pid_gains', 'dt'};
for i = 1:length(required_vars)
    if ~exist(required_vars{i}, 'var')
        error('Variable %s not found in loaded workspace.', required_vars{i});
    end
end

% Select axis to tune (1=Roll, 2=Pitch, 3=Yaw)
axis_idx = 3;  % Change this to tune a different axis
axis_name = {'Roll', 'Pitch', 'Yaw'};

% Create figure
fig = figure('Name', 'PID Tuner', 'Position', [200, 200, 800, 600]);

% Plot initial response
subplot(2,1,1);
hold off;
plot(t, ref_angles(:,axis_idx)*180/pi, 'k--', 'LineWidth', 1.5); hold on;
h_line = plot(t, angle_log(:,axis_idx)*180/pi, 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title(sprintf('%s Axis Step Response - Drag sliders to tune', axis_name{axis_idx}));
legend('Reference', 'Response', 'Location', 'best');
grid on;
ylim([-5, 15]);

% Display current PID gains in title
subtitle_text = sprintf('Current gains: Kp=%.1f, Ki=%.1f, Kd=%.2f', ...
    pid_gains(axis_idx,1), pid_gains(axis_idx,2), pid_gains(axis_idx,3));
title(sprintf('%s Axis Step Response - %s', axis_name{axis_idx}, subtitle_text));

% Create sliders for Kp, Ki, Kd
subplot(2,1,2);
axis off;  % Hide axes for slider area

% Kp slider
uicontrol('Style', 'text', 'String', 'Kp:', 'Position', [50, 80, 30, 20], ...
    'FontWeight', 'bold');
uicontrol('Style', 'slider', 'Min', 0, 'Max', 200, 'Value', pid_gains(axis_idx,1), ...
    'Position', [90, 80, 200, 20], ...
    'Callback', @(src, event) update_plot(src, event, axis_idx, t, ref_angles, J, D, K_motor, dt, h_line));

% Ki slider
uicontrol('Style', 'text', 'String', 'Ki:', 'Position', [320, 80, 30, 20], ...
    'FontWeight', 'bold');
uicontrol('Style', 'slider', 'Min', 0, 'Max', 20, 'Value', pid_gains(axis_idx,2), ...
    'Position', [360, 80, 200, 20], ...
    'Callback', @(src, event) update_plot(src, event, axis_idx, t, ref_angles, J, D, K_motor, dt, h_line));

% Kd slider
uicontrol('Style', 'text', 'String', 'Kd:', 'Position', [590, 80, 30, 20], ...
    'FontWeight', 'bold');
uicontrol('Style', 'slider', 'Min', 0, 'Max', 5, 'Value', pid_gains(axis_idx,3), ...
    'Position', [630, 80, 150, 20], ...
    'Callback', @(src, event) update_plot(src, event, axis_idx, t, ref_angles, J, D, K_motor, dt, h_line));

% Display current values
uicontrol('Style', 'text', 'String', sprintf('Kp=%.1f', pid_gains(axis_idx,1)), ...
    'Position', [90, 50, 200, 20], 'Tag', 'kp_display');
uicontrol('Style', 'text', 'String', sprintf('Ki=%.1f', pid_gains(axis_idx,2)), ...
    'Position', [360, 50, 200, 20], 'Tag', 'ki_display');
uicontrol('Style', 'text', 'String', sprintf('Kd=%.2f', pid_gains(axis_idx,3)), ...
    'Position', [630, 50, 150, 20], 'Tag', 'kd_display');

% Add reset button
uicontrol('Style', 'pushbutton', 'String', 'Reset to Original', ...
    'Position', [350, 20, 150, 30], ...
    'Callback', @(src, event) reset_gains(src, event, axis_idx, h_line, t, ref_angles, J, D, K_motor, dt));

% Display axis info
uicontrol('Style', 'text', 'String', sprintf('Axis: %s    |    J=%.2e kg·m²    |    K_motor=%.4f Nm/V', ...
    axis_name{axis_idx}, J(axis_idx), K_motor(axis_idx)), ...
    'Position', [50, 10, 700, 20], 'FontWeight', 'bold');

% Store original gains for reset
setappdata(fig, 'original_gains', pid_gains(axis_idx, :));
setappdata(fig, 'axis_idx', axis_idx);

disp('========================================');
disp('PID Tuner Started!');
disp('Drag the sliders to adjust gains in real-time.');
disp('The response plot will update automatically.');
disp('Click "Reset to Original" to restore initial gains.');
disp('========================================');

%% Callback function for updating the plot
function update_plot(~, ~, axis_idx, t, ref_angles, J, D, K_motor, dt, h_line)
    % Get current slider values
    Kp = get(findobj('Style', 'slider', 'Position', [90, 80, 200, 20]), 'Value');
    Ki = get(findobj('Style', 'slider', 'Position', [360, 80, 200, 20]), 'Value');
    Kd = get(findobj('Style', 'slider', 'Position', [630, 80, 150, 20]), 'Value');
    
    % Update display text
    set(findobj('Tag', 'kp_display'), 'String', sprintf('Kp=%.1f', Kp));
    set(findobj('Tag', 'ki_display'), 'String', sprintf('Ki=%.1f', Ki));
    set(findobj('Tag', 'kd_display'), 'String', sprintf('Kd=%.2f', Kd));
    
    % Re-simulate with new gains for this axis only
    J_axis = J(axis_idx);
    D_axis = D(axis_idx);
    K_motor_axis = K_motor(axis_idx);
    
    % Preallocate
    angle_sim = zeros(size(t));
    vel_sim = zeros(size(t));
    int_err = 0;
    
    % Simple simulation loop
    for k = 1:length(t)
        ref = ref_angles(k, axis_idx);
        err = ref - angle_sim(k);
        
        % Integral term with anti-windup
        int_err = int_err + err * dt;
        int_err = max(min(int_err, 10), -10);
        
        % Derivative term (using velocity feedback)
        der = -vel_sim(k);
        
        % Control output
        u = Kp * err + Ki * int_err + Kd * der;
        
        % Torque and dynamics
        torque = K_motor_axis * u;
        accel = (torque - D_axis * vel_sim(k)) / J_axis;
        
        % Update state (forward Euler)
        if k < length(t)
            angle_sim(k+1) = angle_sim(k) + vel_sim(k) * dt;
            vel_sim(k+1) = vel_sim(k) + accel * dt;
        end
    end
    
    % Update the plot (convert to degrees)
    set(h_line, 'YData', angle_sim * 180/pi);
    
    % Update title with current gains
    title_str = sprintf('Current gains: Kp=%.1f, Ki=%.1f, Kd=%.2f', Kp, Ki, Kd);
    ax = get(h_line, 'Parent');
    title(ax, title_str);
    
    drawnow;
end

%% Callback function for reset button
function reset_gains(~, ~, axis_idx, h_line, t, ref_angles, J, D, K_motor, dt)
    % Get original gains from appdata
    fig = gcf;
    original_gains = getappdata(fig, 'original_gains');
    
    % Reset sliders
    set(findobj('Style', 'slider', 'Position', [90, 80, 200, 20]), 'Value', original_gains(1));
    set(findobj('Style', 'slider', 'Position', [360, 80, 200, 20]), 'Value', original_gains(2));
    set(findobj('Style', 'slider', 'Position', [630, 80, 150, 20]), 'Value', original_gains(3));
    
    % Update display text
    set(findobj('Tag', 'kp_display'), 'String', sprintf('Kp=%.1f', original_gains(1)));
    set(findobj('Tag', 'ki_display'), 'String', sprintf('Ki=%.1f', original_gains(2)));
    set(findobj('Tag', 'kd_display'), 'String', sprintf('Kd=%.2f', original_gains(3)));
    
    % Force update of plot
    update_plot([], [], axis_idx, t, ref_angles, J, D, K_motor, dt, h_line);
    
    disp('Gains reset to original values.');
end