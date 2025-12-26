clc, clearvars


% Physical Parameters
params.m = 0.468; % kg
params.I = diag([4.865e-3, 4.865e-3, 8.801e-3]); % Inertia matrix Ixx, Iyy, Izz

% Initial State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x0 = zeros(12, 1);
%x0(3) = 1.0; % Start at 1m height

% Add a slight initial tilt (10 degrees)
tilt_deg = 10;
x0(4) = 20 * (pi/180); % Convert Roll to Radians
x0(5) = 30 * (pi/180); % Convert pitch to Radians
x0(6) = 40 * (pi/180); % Convert yaw to Radians

% Desired State: []
x_desired = zeros(8, 1);
x_desired(1) = 15; % Desired 5m height

% Controller Gains
K_p = [1.5 6];
K_d = [2.5 1.75];

% Simulate for 50 seconds
tspan = [0 10];
[t, states] = ode45(@(t, x) quad_dynamics(t, x, controller(x_desired, x, params,  K_p, K_d), params), tspan, x0);

% --- Post-Processing: Reconstruct Control Inputs ---
num_steps = length(t);
u_history = zeros(num_steps, 8); 
for i = 1:num_steps
    u_history(i, :) = controller(x_desired, states(i, :)', params, K_p, K_d)';
end

% --- Unified Plotting Window ---
figure('Color', 'w', 'Name', 'Quadrotor Simulation Summary', 'Units', 'Normalized', 'OuterPosition', [0.1 0.1 0.8 0.8]);

% 1. Position Plot
subplot(2,2,1);
plot(t, states(:, 1:3), 'LineWidth', 1.5);
title('3D Position (m)'); xlabel('Time (s)'); ylabel('Pos (m)');
legend('x', 'y', 'z'); grid on;

% 2. Attitude Plot
subplot(2,2,2);
plot(t, rad2deg(states(:, 4:6)), 'LineWidth', 1.5);
title('Euler Angles (deg)'); xlabel('Time (s)'); ylabel('Angle (deg)');
legend('Roll', 'Pitch', 'Yaw'); grid on;

% 3. Control Inputs (Thrust and Moments)
subplot(2,2,3);
yyaxis left
plot(t, u_history(:, 1), 'LineWidth', 1.5); ylabel('Thrust (N)');
yyaxis right
plot(t, u_history(:, 2:4), '--', 'LineWidth', 1.2); ylabel('Moments (Nm)');
title('Control Inputs'); xlabel('Time (s)');
legend('F (Thrust)', 'M_{phi}', 'M_{theta}', 'M_{psi}', 'Location', 'best'); grid on;

% 4. Motor Speeds
subplot(2,2,4);
plot(t, u_history(:, 5:8), 'LineWidth', 1.5);
title('Motor Speeds (rad/s)'); xlabel('Time (s)'); ylabel('w (rad/s)');
legend('w1', 'w2', 'w3', 'w4'); grid on;

% --- 3D Animation ---
figure('Color', 'w', 'Name', '3D Quadrotor Animation');
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3); % Set 3D view

% Define quadrotor arm length for visualization
L = 0.225; 

% Animation loop
for i = 1:5:length(t) % Step by 5 to speed up rendering
    cla; % Clear previous frame
    
    % Current position
    pos = states(i, 1:3)';
    
    % Current orientation (Euler angles)
    phi = states(i, 4);
    theta = states(i, 5);
    psi = states(i, 6);
    
    % Rotation Matrix (Body to Inertial)
    R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_psi * R_theta * R_phi;
    
    % Define arm endpoints in body frame
    arm1 = [L, 0, 0; -L, 0, 0]';
    arm2 = [0, L, 0; 0, -L, 0]';
    
    % Rotate and translate arms to inertial frame
    arm1_world = R * arm1 + pos;
    arm2_world = R * arm2 + pos;
    
    % Draw the quadcopter arms
    plot3(arm1_world(1,:), arm1_world(2,:), arm1_world(3,:), 'r-o', 'LineWidth', 2);
    plot3(arm2_world(1,:), arm2_world(2,:), arm2_world(3,:), 'b-o', 'LineWidth', 2);
    
    % Draw the trajectory path
    plot3(states(1:i, 1), states(1:i, 2), states(1:i, 3), 'k--', 'LineWidth', 0.5);
    
    % Update plot limits to follow quadcopter
    xlim([pos(1)-2, pos(1)+2]);
    ylim([pos(2)-2, pos(2)+2]);
    zlim([0, max(pos(3)+2, 6)]);
    
    drawnow;
end

% --- Slower, Detailed 3D Animation ---
fig = figure('Color', 'w', 'Name', 'Quadrotor 3D Visualization');
hold on; grid on; axis equal;
xlabel('X (m)'); ylabel('Y (m)'); zlabel('Z (m)');
view(3); 

% Define quadrotor arm length
L = 0.25; 

% Pre-calculate the trajectory for a "ghost" path
plot3(states(:,1), states(:,2), states(:,3), 'Color', [0.8 0.8 0.8], 'LineStyle', ':');

% Animation loop
for i = 1:length(t) % Processing every single time step for maximum detail
    if ~ishandle(fig), break; end % Stop if window is closed
    
    cla; % Clear the frame
    
    % Current position and orientation
    pos = states(i, 1:3)';
    phi = states(i, 4);
    theta = states(i, 5);
    psi = states(i, 6);
    
    % Reconstruct Rotation Matrix (Body to Inertial)
    R_phi   = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_psi * R_theta * R_phi;
    
    % Define arm endpoints in body frame
    % Arm 1: X-axis (Front/Back), Arm 2: Y-axis (Left/Right)
    arm1 = [L, 0, 0; -L, 0, 0]';
    arm2 = [0, L, 0; 0, -L, 0]';
    
    % Rotate and translate to world coordinates
    arm1_w = R * arm1 + pos;
    arm2_w = R * arm2 + pos;
    
    % Draw Arms
    plot3(arm1_w(1,:), arm1_w(2,:), arm1_w(3,:), 'r-o', 'LineWidth', 3, 'MarkerFaceColor', 'r'); % Front/Back (Red)
    plot3(arm2_w(1,:), arm2_w(2,:), arm2_w(3,:), 'b-o', 'LineWidth', 3, 'MarkerFaceColor', 'b'); % Left/Right (Blue)
    
    % Draw a "heading" indicator (a small line pointing forward)
    heading = R * [L; 0; 0] + pos;
    plot3([pos(1) heading(1)], [pos(2) heading(2)], [pos(3) heading(3)], 'k', 'LineWidth', 2);

    % Keep the view focused on the drone
    xlim([pos(1)-1, pos(1)+1]);
    ylim([pos(2)-1, pos(2)+1]);
    zlim([pos(3)-1, pos(3)+1]);
    
    title(sprintf('Time: %.2f s | Altitude: %.2f m', t(i), pos(3)));
    
    % CONTROL SPEED: Change this value to make it slower or faster
    pause(0.01); 
    
    drawnow;
end
function dxdt = quad_dynamics(t, x_state, u, params)
    % Extract states
    % x_state = [x; y; z; phi; theta; psi; vx; vy; vz; p; q; r]
    phi = x_state(4);   % Roll
    theta = x_state(5); % Pitch
    psi = x_state(6);   % Yaw
    v = x_state(7:9);   % Linear velocities
    omega = x_state(10:12); % Angular velocities (p, q, r)

    % Parameters
    m = params.m;       % Mass
    g = 9.81;           % Gravity
    I = params.I;       % Inertia Matrix
    
    % Control Inputs 
    F_total = u(1);
    M = u(2:4);         % [M_phi; M_theta; M3_psi]

    % 1. Rotation Matrix (Body to Inertial FoR)
    % Roll-Pitch-Yaw convention
    R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_psi * R_theta * R_phi;

    % 2. Translational Acceleration
    accel = [0; 0; -g] + (R * [0; 0; F_total] / m);

    % Angular Kinematics (Euler rates)
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_rates = T * omega;

    % 3. Angular Acceleration
    % omega_dot = I^-1 * (-omega x I*omega + M)
    omega_dot = I \ (-cross(omega, I * omega) + M);

    % 4. Assemble state derivative
    dxdt = [v;             % dot_pos
            euler_rates;   % dot_euler
            accel;         % dot_v
            omega_dot];    % dot_omega
end

function controll_input = controller(x_desired, x_state, params,  K_p, K_d)
    % Control Gains for PD controller
    K_pos_p = K_p(1);
    K_pos_d = K_d(1);
    K_att_p = K_p(2);
    K_att_d = K_d(2);
 
    % Desired State
    z_d = x_desired(1);
    z_dot_d = x_desired(2);
    phi_d = x_desired(3);
    theta_d = x_desired(4);
    psi_d = x_desired(5);
    phi_dot_d = x_desired(6);
    theta_dot_d = x_desired(7);
    psi_dot_d = x_desired(8);

    % Parameters
    m = params.m;       % Mass
    g = 9.81;           % Gravity
    Ix = params.I(1,1);
    Iy = params.I(2,2);
    Iz = params.I(3,3);

    % Extract states
    % x_state = [x; y; z; phi; theta; psi; vx; vy; vz; p; q; r]
    phi = x_state(4);   % Roll
    theta = x_state(5); % Pitch
    psi = x_state(6);   % Yaw
    z = x_state(3);     % Altitude
    z_dot = x_state(9);  %Vz
    omega = x_state(10:12); % Angular velocities (p, q, r)

    % Euler rates
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_rates = T * omega;

    phi_dot = euler_rates(1);
    theta_dot = euler_rates(2);
    psi_dot = euler_rates(3);

    % Roll, Pitch, and Yaw Moments
    M_phi = Ix * (K_att_d * (phi_dot_d - phi_dot) + K_att_p * (phi_d - phi));
    M_theta = Iy * (K_att_d * (theta_dot_d - theta_dot) + K_att_p * (theta_d - theta));
    M_psi = Iz * (K_att_d * (psi_dot_d - psi_dot) + K_att_p * (psi_d - psi));

    %Thrust and Motor Speeds (w1-w4)
    % Total Thrust F
    F = (g + K_pos_d * (z_dot_d - z_dot) + K_pos_p * (z_d - z)) * (m / (cos(phi) * cos(theta)));
    
    % Motor angular velocities (w)
    k = 2.980e-6;   % Lift coefficient
    b = 1.140e-7;   % Drag coefficient
    l = 0.225;       % Arm length (Ensure 'l' is defined)
    w1 = (F/(4*k) - M_theta/(2*k*l) - M_psi/(4*b))^0.5;
    w2 = (F/(4*k) - M_phi/(2*k*l)   + M_psi/(4*b))^0.5;
    w3 = (F/(4*k) + M_theta/(2*k*l) - M_psi/(4*b))^0.5;
    w4 = (F/(4*k) + M_phi/(2*k*l)   + M_psi/(4*b))^0.5;
    
    w = [w1; w2; w3; w4;];

    controll_input = [F; M_phi; M_theta; M_psi; w];
end
