%% 3‑Axis Gimbal Simulation with Real Motor Parameters
clear; close all; clc;

%% Motor Parameters (from datasheets)
% ===== Gimbal Motor (DM3505/3505EC) =====
% Used for pitch and roll axes
J_rotor_gimbal = 5.6e-6;          % kg·m² (rotor inertia from datasheet)
Kt_gimbal = 0.081;                 % Nm/A (torque constant)
R_gimbal = 6.34;                   % Ω (phase-to-phase resistance)
K_motor_gimbal = Kt_gimbal / R_gimbal;  % 0.0128 Nm/V

% ===== Geared DC Motor (GA37-520) =====
% Used for yaw axis (needs more torque)
Kt_geared = 0.1225;                % Nm/A (derived from stall torque/current)
R_geared = 3.0;                    % Ω (estimated)
K_motor_geared = Kt_geared / R_geared;  % 0.0408 Nm/V

% Gear ratio for yaw motor
gear_ratio = 30;   % From datasheet: 1:30

%% System Inertia (including loads and motor weights)
% These need to be estimated based on your actual setup
% Include: motor rotor inertia + payload/camera + arm lengths

% Example payload: small camera + mounting hardware (~300g at 10cm distance)
m_payload = 0.300;                 % kg (300g camera + mount)
L_payload = 0.10;                  % m (distance from rotation axis)
J_payload = m_payload * L_payload^2;  % kg·m²
% J_payload = 0.300 * 0.10^2 = 0.003 kg·m²

% Axis-specific inertias
% Roll axis: typically the smallest inertia
J_roll = J_rotor_gimbal + 0.5 * J_payload;  % Partially loaded
% Pitch axis: carries the full payload
J_pitch = J_rotor_gimbal + J_payload;
% Yaw axis: carries everything + geared motor rotor (estimated)
% Geared motor rotor inertia (estimate from weight)
m_geared = 0.150;                  % kg (150g from datasheet)
r_geared = 0.015;                  % m (approx radius of motor, 15mm)
J_rotor_geared_est = 0.5 * m_geared * r_geared^2;  % ~1.7e-5 kg·m²
J_yaw = J_rotor_geared_est * gear_ratio^2 + J_payload + J_rotor_gimbal;

% Assemble inertia array
J = [J_roll; J_pitch; J_yaw];  % kg·m²

%% Damping Coefficients (estimated)
% These are hard to get from datasheets, typically estimated experimentally
D = [0.005;  % Roll damping
     0.008;  % Pitch damping (higher due to payload)
     0.015]; % Yaw damping (higher due to gearbox friction)

%% Motor Gains (K_motor in Nm/V)
% Roll and pitch: use gimbal motor
% Yaw: use geared motor (with gear ratio, torque is multiplied)
K_motor = [K_motor_gimbal;          % Roll
           K_motor_gimbal;          % Pitch
           K_motor_geared * gear_ratio];  % Yaw (gear ratio multiplies torque)

%% Display motor constants
% fprintf('====== Motor Constants ======\n');
% fprintf('Gimbal Motor (DM3505):\n');
% fprintf('  Kt = %.4f Nm/A\n', Kt_gimbal);
% fprintf('  R = %.2f Ω\n', R_gimbal);
% fprintf('  K_motor = %.4f Nm/V\n', K_motor_gimbal);
% fprintf('  Rotor Inertia = %.2e kg·m²\n', J_rotor_gimbal);
% fprintf('\n');
% fprintf('Geared Motor (GA37-520):\n');
% fprintf('  Kt = %.4f Nm/A\n', Kt_geared);
% fprintf('  R = %.1f Ω\n', R_geared);
% fprintf('  K_motor = %.4f Nm/V\n', K_motor_geared);
% fprintf('  Gear Ratio = %d:1\n', gear_ratio);
% fprintf('  Effective K_motor (after gearbox) = %.4f Nm/V\n', K_motor(3));

%% PID Gains (tuned for these specific motors)
% Start with these and tune based on simulation results
pid_gains = [80,  5, 0.8;   % Roll axis (light, fast)
             100, 8, 1.0;   % Pitch axis (heavier, needs more P gain)
             60,  3, 0.6];  % Yaw axis (geared, slower response)

%% Simulation Time Settings
dt = 0.001;            % Time step (1 ms)
t_end = 10;            % Total simulation time (seconds)
t = 0:dt:t_end;
N = length(t);

%% Target Angles
ref_angles = zeros(N, 3);
ref_angles(:,1) = 5 * pi/180;   % Roll setpoint (rad)
ref_angles(:,2) = 10 * pi/180;  % Pitch setpoint (rad)
ref_angles(:,3) = 45 * pi/180;  % Yaw setpoint (rad)

%% State Initialization
% States: [angle, angular_velocity] for each axis
state = zeros(2, 3);   % row1 = angle, row2 = velocity
integral_error = zeros(3,1);
prev_error = zeros(3,1);

% Preallocate logs
angle_log = zeros(N, 3);
velocity_log = zeros(N, 3);
torque_log = zeros(N, 3);
control_log = zeros(N, 3);

%% Simulation Loop
for k = 1:N
    % Current reference and state
    ref = ref_angles(k, :);
    angle = state(1, :);
    velocity = state(2, :);
    
    % Compute error
    error = ref - angle;
    
    % PID control for each axis (independent)
    for axis = 1:3
        Kp = pid_gains(axis, 1);
        Ki = pid_gains(axis, 2);
        Kd = pid_gains(axis, 3);
        
        % Integral term (with anti‑windup clamp)
        integral_error(axis) = integral_error(axis) + error(axis) * dt;
        % Clamp integral to avoid excessive windup
        integral_error(axis) = max(min(integral_error(axis), 10), -10);
        
        % Derivative term (using velocity feedback to avoid derivative kick)
        derivative = -velocity(axis);   % Negative because velocity = d(angle)/dt
        
        % Control output (voltage command)
        u = Kp * error(axis) + Ki * integral_error(axis) + Kd * derivative;
        control_log(k, axis) = u;
        
        % Torque = motor gain * voltage
        torque = K_motor(axis) * u;
        torque_log(k, axis) = torque;
        
        % Dynamics update (forward Euler)
        % acceleration = (torque - damping * velocity) / inertia
        accel = (torque - D(axis) * velocity(axis)) / J(axis);
        
        % Update state
        state(1, axis) = angle(axis) + velocity(axis) * dt;
        state(2, axis) = velocity(axis) + accel * dt;
        
        % Store logs
        angle_log(k, axis) = state(1, axis);
        velocity_log(k, axis) = state(2, axis);
    end
end

%% Convert angles to degrees for plotting
angle_deg = angle_log * 180/pi;
ref_deg = ref_angles * 180/pi;

%% Plotting
figure('Position', [100, 100, 1200, 800]);

% Subplot 1: Angle response
subplot(2,2,1);
plot(t, ref_deg(:,1), 'k--', 'LineWidth', 1.5); hold on;
plot(t, angle_deg(:,1), 'r', 'LineWidth', 1.2);
plot(t, angle_deg(:,2), 'g', 'LineWidth', 1.2);
plot(t, angle_deg(:,3), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angle (deg)');
title('Gimbal Angle Response');
legend('Reference (Roll)', 'Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on;

% Subplot 2: Angular velocity
subplot(2,2,2);
plot(t, velocity_log(:,1)*180/pi, 'r', 'LineWidth', 1.2); hold on;
plot(t, velocity_log(:,2)*180/pi, 'g', 'LineWidth', 1.2);
plot(t, velocity_log(:,3)*180/pi, 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Angular Velocity (deg/s)');
title('Gimbal Angular Velocity');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on;

% Subplot 3: Control effort (voltage)
subplot(2,2,3);
plot(t, control_log(:,1), 'r', 'LineWidth', 1.2); hold on;
plot(t, control_log(:,2), 'g', 'LineWidth', 1.2);
plot(t, control_log(:,3), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Control Voltage (V)');
title('PID Control Effort');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on;

% Subplot 4: Tracking error
subplot(2,2,4);
error_deg = (ref_angles - angle_log) * 180/pi;
plot(t, error_deg(:,1), 'r', 'LineWidth', 1.2); hold on;
plot(t, error_deg(:,2), 'g', 'LineWidth', 1.2);
plot(t, error_deg(:,3), 'b', 'LineWidth', 1.2);
xlabel('Time (s)'); ylabel('Error (deg)');
title('Tracking Error');
legend('Roll', 'Pitch', 'Yaw', 'Location', 'best');
grid on;

%% Save key results for later analysis
save('gimbal_simulation_results.mat', 'dt', 't', 'angle_log', 'velocity_log', ...
     'control_log', 'ref_angles', 'pid_gains', 'J', 'D', 'K_motor');
disp('Simulation complete. Results saved to gimbal_simulation_results.mat');