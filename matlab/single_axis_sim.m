%% SINGLE-AXIS GIMBAL GAIN TUNING SCRIPT
% This script helps you tune the PID gains for your single-axis gimbal.
% It provides:
% 1. Manual tuning with interactive sliders
% 2. Automated tuning using optimization
% 3. Performance analysis and visualization

clear; close all; clc;

%% ========================================================================
% PART 1: SYSTEM PARAMETERS (MEASURE OR ESTIMATE)
% ========================================================================

% Motor and system parameters (update these with your measured values)
params.J = 1.5e-4;          % Total inertia (kg·m²) - estimate from motor datasheet
params.D = 0.013;          % Damping coefficient (N·m·s/rad) - estimate
params.K_motor = 0.0408;   % Motor gain (Nm/V) - from your motor datasheet
params.dt = 0.001;         % Control loop period (1ms)
params.dead_zone_left = 30;   % Left motor dead zone (PWM counts)
params.dead_zone_right = 25;  % Right motor dead zone (PWM counts)

% IMU parameters
params.ASCALE = 2*9.81/32768;
params.ACC_TO_RAD = 0.1176;
params.GSCALE = pi*250/(180*32768);
params.Kf_body = 0.3;      % Kalman filter gain

% Simulation parameters
params.t_end = 5;          % Simulation duration (seconds)
params.max_command = 255;   % Maximum PWM command

% Disturbance parameters (for testing)
params.disturbance_amplitude = 0.1;  % rad (approximately 5.7°)
params.disturbance_freq = 2;         % Hz

%% ========================================================================
% PART 2: INTERACTIVE MANUAL TUNING
% ========================================================================

fprintf('========================================\n');
fprintf('GIMBAL GAIN TUNING TOOL\n');
fprintf('========================================\n\n');

fprintf('Recommended tuning order:\n');
fprintf('1. Start with Kp_wheel and Kd_wheel only (Kp_body = 0)\n');
fprintf('2. Add Kp_body for disturbance rejection\n');
fprintf('3. Fine-tune all gains together\n\n');

% Default gains (starting point)
default_gains = struct(...
    'Kp_wheel', 80, ...
    'Kd_wheel', 12, ...
    'Kp_body', 5, ...
    'Kf_body', 0.3);

% Create interactive tuning figure
manual_tuning_figure(default_gains, params);

fprintf('\nManual tuning figure opened.\n');
fprintf('Drag sliders to adjust gains in real-time.\n');
fprintf('Close the figure to continue to automated tuning.\n');

% Wait for user to close figure
uiwait;

%% ========================================================================
% PART 3: AUTOMATED GAIN TUNING
% ========================================================================

fprintf('\n========================================\n');
fprintf('AUTOMATED GAIN OPTIMIZATION\n');
fprintf('========================================\n\n');

% Ask user if they want to run automated tuning
response = input('Run automated gain optimization? (y/n): ', 's');
if lower(response) ~= 'y'
    fprintf('Skipping automated tuning.\n');
    return;
end

% Define optimization bounds
lb = [10, 1, 0, 0.05];     % Lower bounds: [Kp_wheel, Kd_wheel, Kp_body, Kf_body]
ub = [300, 50, 30, 0.8];   % Upper bounds

% Initial guess (from manual tuning or defaults)
x0 = [default_gains.Kp_wheel, default_gains.Kd_wheel, ...
      default_gains.Kp_body, default_gains.Kf_body];

% Run optimization
fprintf('Optimizing gains... This may take a minute.\n');
options = optimset('Display', 'iter', 'MaxIter', 50, 'TolX', 1e-4);
[x_opt, fval_opt, exitflag] = fminsearch(@(x) cost_function(x, params), x0, options);

% Display results
fprintf('\n========================================\n');
fprintf('OPTIMIZATION RESULTS\n');
fprintf('========================================\n');
fprintf('Optimization exit flag: %d\n', exitflag);
fprintf('Final cost: %.4f\n\n', fval_opt);
fprintf('Optimal gains:\n');
fprintf('  Kp_wheel = %.2f\n', x_opt(1));
fprintf('  Kd_wheel = %.2f\n', x_opt(2));
fprintf('  Kp_body  = %.2f\n', x_opt(3));
fprintf('  Kf_body  = %.3f\n', x_opt(4));

% Compare with manual gains
fprintf('\nComparison with manual gains:\n');
fprintf('Gain        Manual    Optimal\n');
fprintf('Kp_wheel    %8.2f  %8.2f\n', default_gains.Kp_wheel, x_opt(1));
fprintf('Kd_wheel    %8.2f  %8.2f\n', default_gains.Kd_wheel, x_opt(2));
fprintf('Kp_body     %8.2f  %8.2f\n', default_gains.Kp_body, x_opt(3));
fprintf('Kf_body     %8.3f  %8.3f\n', default_gains.Kf_body, x_opt(4));

% Simulate with optimal gains
fprintf('\nSimulating with optimal gains...\n');
[~, ~, ~, metrics_opt] = simulate_gimbal(x_opt, params);

% Simulate with manual gains
x_manual = [default_gains.Kp_wheel, default_gains.Kd_wheel, ...
            default_gains.Kp_body, default_gains.Kf_body];
[~, ~, ~, metrics_manual] = simulate_gimbal(x_manual, params);

% Compare performance
fprintf('\nPerformance comparison:\n');
fprintf('Metric              Manual    Optimal    Improvement\n');
fprintf('RMS Error (deg)     %8.3f  %8.3f  %8.1f%%\n', ...
    metrics_manual.rms_error, metrics_opt.rms_error, ...
    (metrics_manual.rms_error - metrics_opt.rms_error)/metrics_manual.rms_error*100);
fprintf('Max Error (deg)     %8.3f  %8.3f  %8.1f%%\n', ...
    metrics_manual.max_error, metrics_opt.max_error, ...
    (metrics_manual.max_error - metrics_opt.max_error)/metrics_manual.max_error*100);
fprintf('Control Effort RMS  %8.2f  %8.2f  %8.1f%%\n', ...
    metrics_manual.control_rms, metrics_opt.control_rms, ...
    (metrics_manual.control_rms - metrics_opt.control_rms)/metrics_manual.control_rms*100);

%% ========================================================================
% PART 4: CODE GENERATION
% ========================================================================

fprintf('\n========================================\n');
fprintf('ARDUINO CODE GENERATION\n');
fprintf('========================================\n\n');

% Generate Arduino code with optimized gains
generate_arduino_code(x_opt, params);

fprintf('\nCopy the constants above into your gimbal driver.\n');

%% ========================================================================
% SUPPORTING FUNCTIONS
% ========================================================================

% -------------------------------------------------------------------------
% 1. Manual Tuning GUI
% -------------------------------------------------------------------------
function manual_tuning_figure(default_gains, params)
    % Create figure
    fig = figure('Name', 'Gimbal Gain Tuner', 'Position', [200, 200, 900, 700]);
    
    % Current gains
    gains = default_gains;
    
    % Create axes for plots
    ax1 = subplot(2,2,1);
    ax2 = subplot(2,2,2);
    ax3 = subplot(2,2,3);
    ax4 = subplot(2,2,4);
    
    % Initial simulation
    x = [gains.Kp_wheel, gains.Kd_wheel, gains.Kp_body, gains.Kf_body];
    [t, angle_log, vel_log, error_log, command_log, metrics] = ...
        simulate_gimbal(x, params);
    
    % Plot 1: Angle response
    plot_angle_response(ax1, t, angle_log, params);
    
    % Plot 2: Tracking error
    plot_error_response(ax2, t, error_log);
    
    % Plot 3: Control effort
    plot_control_effort(ax3, t, command_log);
    
    % Plot 4: Performance metrics
    plot_performance_metrics(ax4, metrics);
    
    % Create sliders
    slider_panel = uipanel('Title', 'Gain Controls', 'Position', [0.05, 0.02, 0.9, 0.12]);
    
    % Kp_wheel slider
    uicontrol('Parent', slider_panel, 'Style', 'text', 'String', 'Kp_wheel:', ...
        'Position', [10, 30, 70, 20], 'HorizontalAlignment', 'left');
    uicontrol('Parent', slider_panel, 'Style', 'slider', ...
        'Min', 0, 'Max', 300, 'Value', gains.Kp_wheel, ...
        'Position', [90, 30, 120, 20], ...
        'Callback', @(src,~) update_plots());
    uicontrol('Parent', slider_panel, 'Style', 'text', ...
        'String', sprintf('%.1f', gains.Kp_wheel), ...
        'Position', [215, 30, 40, 20], 'Tag', 'Kp_wheel_val');
    
    % Kd_wheel slider
    uicontrol('Parent', slider_panel, 'Style', 'text', 'String', 'Kd_wheel:', ...
        'Position', [270, 30, 70, 20], 'HorizontalAlignment', 'left');
    uicontrol('Parent', slider_panel, 'Style', 'slider', ...
        'Min', 0, 'Max', 50, 'Value', gains.Kd_wheel, ...
        'Position', [350, 30, 120, 20], ...
        'Callback', @(src,~) update_plots());
    uicontrol('Parent', slider_panel, 'Style', 'text', ...
        'String', sprintf('%.1f', gains.Kd_wheel), ...
        'Position', [475, 30, 40, 20], 'Tag', 'Kd_wheel_val');
    
    % Kp_body slider
    uicontrol('Parent', slider_panel, 'Style', 'text', 'String', 'Kp_body:', ...
        'Position', [530, 30, 60, 20], 'HorizontalAlignment', 'left');
    uicontrol('Parent', slider_panel, 'Style', 'slider', ...
        'Min', 0, 'Max', 30, 'Value', gains.Kp_body, ...
        'Position', [600, 30, 120, 20], ...
        'Callback', @(src,~) update_plots());
    uicontrol('Parent', slider_panel, 'Style', 'text', ...
        'String', sprintf('%.1f', gains.Kp_body), ...
        'Position', [725, 30, 40, 20], 'Tag', 'Kp_body_val');
    
    % Kf_body slider
    uicontrol('Parent', slider_panel, 'Style', 'text', 'String', 'Kf_body:', ...
        'Position', [10, 5, 60, 20], 'HorizontalAlignment', 'left');
    uicontrol('Parent', slider_panel, 'Style', 'slider', ...
        'Min', 0, 'Max', 1, 'Value', gains.Kf_body, ...
        'Position', [80, 5, 180, 20], ...
        'Callback', @(src,~) update_plots());
    uicontrol('Parent', slider_panel, 'Style', 'text', ...
        'String', sprintf('%.2f', gains.Kf_body), ...
        'Position', [265, 5, 50, 20], 'Tag', 'Kf_body_val');
    
    % Store handles for update function
    handles = struct('fig', fig, 'ax1', ax1, 'ax2', ax2, 'ax3', ax3, 'ax4', ax4, ...
        'params', params, 'gains', gains);
    guidata(fig, handles);
    
    % Nested update function
    function update_plots()
        % Get current gains from sliders
        slider = findobj('Style', 'slider');
        vals = get(slider, 'Value');
        if iscell(vals), vals = cell2mat(vals); end
        
        % Update gain values
        handles.gains.Kp_wheel = vals(1);
        handles.gains.Kd_wheel = vals(2);
        handles.gains.Kp_body = vals(3);
        handles.gains.Kf_body = vals(4);
        
        % Update text displays
        set(findobj('Tag', 'Kp_wheel_val'), 'String', sprintf('%.1f', vals(1)));
        set(findobj('Tag', 'Kd_wheel_val'), 'String', sprintf('%.1f', vals(2)));
        set(findobj('Tag', 'Kp_body_val'), 'String', sprintf('%.1f', vals(3)));
        set(findobj('Tag', 'Kf_body_val'), 'String', sprintf('%.2f', vals(4)));
        
        % Run simulation with new gains
        x = [vals(1), vals(2), vals(3), vals(4)];
        [t_new, angle_log_new, ~, error_log_new, command_log_new, metrics_new] = ...
            simulate_gimbal(x, handles.params);
        
        % Update plots
        plot_angle_response(handles.ax1, t_new, angle_log_new, handles.params);
        plot_error_response(handles.ax2, t_new, error_log_new);
        plot_control_effort(handles.ax3, t_new, command_log_new);
        plot_performance_metrics(handles.ax4, metrics_new);
        
        drawnow;
    end
end

% -------------------------------------------------------------------------
% 2. Simulation Function
% -------------------------------------------------------------------------
function [t, angle_log, vel_log, error_log, command_log, metrics] = ...
    simulate_gimbal(x, params)
    % x = [Kp_wheel, Kd_wheel, Kp_body, Kf_body]
    
    % Extract gains
    Kp_wheel = x(1);
    Kd_wheel = x(2);
    Kp_body = x(3);
    Kf_body = x(4);
    
    % Simulation time
    dt = params.dt;
    t = 0:dt:params.t_end;
    N = length(t);
    
    % Initialize states
    wheel_angle = 0;
    wheel_velocity = 0;
    body_angle = 0;
    last_body_angle = 0;
    last_gx = 0;
    
    % Log arrays
    angle_log = zeros(N, 1);
    vel_log = zeros(N, 1);
    error_log = zeros(N, 1);
    command_log = zeros(N, 1);
    body_angle_log = zeros(N, 1);
    
    % Generate disturbance (body angle motion - like hand movement)
    disturbance_amp = params.disturbance_amplitude;
    disturbance_freq = params.disturbance_freq;
    
    for k = 1:N
        % Body angle (disturbance) - realistic hand motion
        body_angle = disturbance_amp * sin(2*pi*disturbance_freq*t(k)) + ...
                     0.3*disturbance_amp * sin(2*pi*10*t(k));  % Add tremor
        
        % Simulate Kalman filter (simplified)
        % In real system, this would use gyro and accel data
        body_angle_estimate = body_angle;  % Assume perfect estimate for simulation
        
        % Control law: wheel should be at -body_angle
        desired_wheel = -body_angle_estimate;
        error = desired_wheel - wheel_angle;
        error_velocity = -wheel_velocity;
        
        % Calculate command
        command_float = Kp_wheel * error + Kd_wheel * error_velocity;
        
        % Add disturbance rejection term (simplified)
        command_float = command_float + Kp_body * body_angle;
        
        % Saturate
        command = max(min(command_float, params.max_command), -params.max_command);
        
        % Apply dead zone (simplified - using average)
        dead_zone = (params.dead_zone_left + params.dead_zone_right) / 2;
        if abs(command) < dead_zone
            command = 0;
        end
        
        % Motor dynamics
        torque = params.K_motor * command;
        accel = (torque - params.D * wheel_velocity) / params.J;
        
        % Update state (forward Euler)
        wheel_angle = wheel_angle + wheel_velocity * dt;
        wheel_velocity = wheel_velocity + accel * dt;
        
        % Log data
        angle_log(k) = wheel_angle;
        vel_log(k) = wheel_velocity;
        error_log(k) = error;
        command_log(k) = command;
        body_angle_log(k) = body_angle;
    end
    
    % Calculate performance metrics
    angle_deg = angle_log * 180/pi;
    body_deg = body_angle_log * 180/pi;
    error_deg = error_log * 180/pi;
    
    metrics.rms_error = sqrt(mean(error_deg.^2));
    metrics.max_error = max(abs(error_deg));
    metrics.mean_error = mean(abs(error_deg));
    metrics.control_rms = rms(command_log);
    metrics.settling_time = calculate_settling_time(t, error_deg);
    
    % Add disturbance rejection metric
    relative_angle = (angle_log - body_angle_log) * 180/pi;
    metrics.rejection_ratio = 1 - (sqrt(mean(relative_angle.^2)) / sqrt(mean(body_deg.^2)));
end

% -------------------------------------------------------------------------
% 3. Cost Function for Optimization
% -------------------------------------------------------------------------
function cost = cost_function(x, params)
    % Weighted cost function for optimization
    [~, ~, ~, ~, ~, metrics] = simulate_gimbal(x, params);
    
    % Weights for different objectives
    w_rms = 1.0;      % RMS error
    w_max = 0.5;      % Max error
    w_control = 0.01; % Control effort
    w_rejection = 0.2; % Disturbance rejection
    
    cost = w_rms * metrics.rms_error + ...
           w_max * metrics.max_error + ...
           w_control * metrics.control_rms + ...
           w_rejection * (1 - metrics.rejection_ratio);
end

% -------------------------------------------------------------------------
% 4. Plotting Functions
% -------------------------------------------------------------------------
function plot_angle_response(ax, t, angle_log, params)
    axes(ax);
    cla(ax);
    
    angle_deg = angle_log * 180/pi;
    body_angle = params.disturbance_amplitude * sin(2*pi*params.disturbance_freq*t) * 180/pi;
    
    plot(t, angle_deg, 'b', 'LineWidth', 1.5); hold on;
    plot(t, -body_angle, 'r--', 'LineWidth', 1.5);
    plot(t, body_angle, 'g--', 'LineWidth', 1);
    
    xlabel('Time (s)'); ylabel('Angle (deg)');
    title('Gimbal Response');
    legend('Wheel Angle', 'Desired', 'Body (disturbance)', 'Location', 'best');
    grid on;
    xlim([0, params.t_end]);
    ylim([-20, 20]);
end

function plot_error_response(ax, t, error_log)
    axes(ax);
    cla(ax);
    
    error_deg = error_log * 180/pi;
    
    plot(t, error_deg, 'b', 'LineWidth', 1.5);
    hold on;
    plot(t, 2*ones(size(t)), 'r--', 'LineWidth', 1);
    plot(t, -2*ones(size(t)), 'r--', 'LineWidth', 1);
    
    xlabel('Time (s)'); ylabel('Error (deg)');
    title('Tracking Error');
    grid on;
    xlim([0, max(t)]);
    ylim([-10, 10]);
end

function plot_control_effort(ax, t, command_log)
    axes(ax);
    cla(ax);
    
    plot(t, command_log, 'b', 'LineWidth', 1.5);
    
    xlabel('Time (s)'); ylabel('PWM Command');
    title('Control Effort');
    grid on;
    xlim([0, max(t)]);
    ylim([-255, 255]);
end

function plot_performance_metrics(ax, metrics)
    axes(ax);
    cla(ax);
    
    % Create bar plot
    metrics_names = {'RMS Error', 'Max Error', 'Mean Error', 'Control RMS'};
    metrics_values = [metrics.rms_error, metrics.max_error, ...
                      metrics.mean_error, metrics.control_rms];
    
    bar(metrics_values);
    set(gca, 'XTickLabel', metrics_names);
    ylabel('Value');
    title('Performance Metrics');
    grid on;
    
    % Add text for settling time
    if ~isnan(metrics.settling_time)
        text(0.5, 0.8, sprintf('Settling: %.2f s', metrics.settling_time), ...
             'Units', 'normalized', 'FontSize', 10);
    end
end

% -------------------------------------------------------------------------
% 5. Utility Functions
% -------------------------------------------------------------------------
function settling_time = calculate_settling_time(t, error_deg)
    % Calculate 2% settling time
    tolerance = 2;  % degrees
    settled_idx = find(abs(error_deg) < tolerance, 1, 'last');
    if ~isempty(settled_idx)
        settling_time = t(settled_idx);
    else
        settling_time = NaN;
    end
end

% -------------------------------------------------------------------------
% 6. Arduino Code Generator
% -------------------------------------------------------------------------
function generate_arduino_code(x_opt, params)
    % Generate Arduino constants with optimal gains
    
    fprintf('\n/* ============================================================\n');
    fprintf('   OPTIMIZED GIMBAL GAINS - Copy these to your Arduino code\n');
    fprintf('   ============================================================ */\n\n');
    
    fprintf('// Control gains (optimized)\n');
    fprintf('#define Kp_wheel  %.2f\n', x_opt(1));
    fprintf('#define Kd_wheel  %.2f\n', x_opt(2));
    fprintf('#define Kp_body   %.2f\n', x_opt(3));
    fprintf('#define Kf_body   %.3f\n', x_opt(4));
    
    fprintf('\n// Dead zone compensation\n');
    fprintf('#define DEAD_ZONE_LEFT  %d\n', params.dead_zone_left);
    fprintf('#define DEAD_ZONE_RIGHT %d\n', params.dead_zone_right);
    fprintf('#define BOOST_LEFT      %d\n', ceil(params.dead_zone_left/2));
    fprintf('#define BOOST_RIGHT     %d\n', ceil(params.dead_zone_right/2));
    
    fprintf('\n/* ============================================================ */\n');
end

%% ========================================================================
% PART 5: TESTING THE TUNED GAINS
% ========================================================================

% This section runs a comprehensive test with the optimal gains

fprintf('\n========================================\n');
fprintf('GAIN VALIDATION\n');
fprintf('========================================\n\n');

% Run validation test
response = input('Run validation test with optimal gains? (y/n): ', 's');
if lower(response) == 'y'
    % Test with different disturbance scenarios
    test_scenarios(params, x_opt);
end

fprintf('\nTuning complete!\n');

%% ========================================================================
% VALIDATION FUNCTION
% ========================================================================

function test_scenarios(params, x_opt)
    % Test different disturbance scenarios
    
    scenarios = {
        'Step disturbance', @(t) 5*pi/180 * (t > 1);
        'Sinusoidal (1Hz)', @(t) 5*pi/180 * sin(2*pi*1*t);
        'Sinusoidal (5Hz)', @(t) 5*pi/180 * sin(2*pi*5*t);
        'Random (tremor)', @(t) 2*pi/180 * randn(size(t));
        'Complex motion', @(t) 5*pi/180*sin(2*pi*0.5*t) + ...
                              2*pi/180*sin(2*pi*10*t);
    };
    
    scenario_names = {'Step', '1Hz', '5Hz', 'Tremor', 'Complex'};
    
    figure('Name', 'Gain Validation - Multiple Scenarios', 'Position', [100, 100, 1200, 800]);
    
    for s = 1:length(scenarios)
        % Override disturbance function
        params.disturbance_func = scenarios{s};
        
        % Simulate
        [t, angle_log, ~, error_log, ~, metrics] = simulate_gimbal(x_opt, params);
        
        % Plot
        subplot(3, 2, s);
        angle_deg = angle_log * 180/pi;
        error_deg = error_log * 180/pi;
        
        plot(t, angle_deg, 'b', 'LineWidth', 1.5); hold on;
        plot(t, error_deg, 'r', 'LineWidth', 1);
        xlabel('Time (s)'); ylabel('Angle (deg)');
        title(sprintf('%s - RMS: %.2f°', scenario_names{s}, metrics.rms_error));
        legend('Wheel', 'Error', 'Location', 'best');
        grid on;
    end
    
    sgtitle('Gimbal Performance with Optimized Gains');
end