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

% 0. Data Pre-processing (Interpolation for Real-Time)
FPS = 30; % Frames per second
t_realtime = 0:(1/FPS):t(end); % Create a uniform time vector

% Interpolate states to match the uniform time vector
states_interp = interp1(t, states, t_realtime);

% 1. FIRST ANIMATION: Standard View (Real-Time)
v1 = VideoWriter('animation1_realtime.mp4', 'MPEG-4');
v1.FrameRate = FPS; 
open(v1);

fig1 = figure('Color', 'w', 'Position', [100 100 800 600]);
hold on; grid on; axis equal; view(3);

L = 0.225; 
for i = 1:length(t_realtime)
    cla;
    pos = states_interp(i, 1:3)';
    R = build_rotation(states_interp(i,4), states_interp(i,5), states_interp(i,6));
    
    arm1_world = R * [L, 0, 0; -L, 0, 0]' + pos;
    arm2_world = R * [0, L, 0; 0, -L, 0]' + pos;
    
    plot3(arm1_world(1,:), arm1_world(2,:), arm1_world(3,:), 'r-o', 'LineWidth', 2);
    plot3(arm2_world(1,:), arm2_world(2,:), arm2_world(3,:), 'b-o', 'LineWidth', 2);
    plot3(states_interp(1:i, 1), states_interp(1:i, 2), states_interp(1:i, 3), 'k--', 'LineWidth', 0.5);
    
    xlim([pos(1)-2, pos(1)+2]); ylim([pos(2)-2, pos(2)+2]); zlim([0, max(pos(3)+2, 6)]);
    title(sprintf('Real-Time Animation: %.2f s', t_realtime(i)));
    
    drawnow;
    writeVideo(v1, getframe(fig1));
end
close(v1);

% 2. SECOND ANIMATION: Detailed Close-up (Real-Time)
v2 = VideoWriter('animation2_realtime_detailed.mp4', 'MPEG-4');
v2.FrameRate = FPS;
open(v2);

fig2 = figure('Color', 'w', 'Position', [150 150 800 600]);
hold on; grid on; axis equal; view(3);
L = 0.25;

for i = 1:length(t_realtime)
    cla;
    pos = states_interp(i, 1:3)';
    R = build_rotation(states_interp(i,4), states_interp(i,5), states_interp(i,6));
    
    arm1_w = R * [L, 0, 0; -L, 0, 0]' + pos;
    arm2_w = R * [0, L, 0; 0, -L, 0]' + pos;
    
    plot3(arm1_w(1,:), arm1_w(2,:), arm1_w(3,:), 'r-o', 'LineWidth', 3, 'MarkerFaceColor', 'r');
    plot3(arm2_w(1,:), arm2_w(2,:), arm2_w(3,:), 'b-o', 'LineWidth', 3, 'MarkerFaceColor', 'b');
    
    xlim([pos(1)-1, pos(1)+1]); ylim([pos(2)-1, pos(2)+1]); zlim([pos(3)-1, pos(3)+1]);
    title(sprintf('Real-Time Detailed: %.2f s', t_realtime(i)));
    
    drawnow;
    writeVideo(v2, getframe(fig2));
end
close(v2);

% 3. THIRD ANIMATION: Path Growth (Real-Time)
v3 = VideoWriter('animation3_realtime_path.mp4', 'MPEG-4');
v3.FrameRate = FPS;
open(v3);

fig3 = figure('Color', 'w', 'Position', [200 200 800 600]);
hold on; grid on; axis equal; view(3);
path_handle = plot3(nan, nan, nan, 'k', 'LineWidth', 1);

for i = 1:length(t_realtime)
    pos = states_interp(i, 1:3)';
    set(path_handle, 'XData', states_interp(1:i, 1), 'YData', states_interp(1:i, 2), 'ZData', states_interp(1:i, 3));
    
    R = build_rotation(states_interp(i,4), states_interp(i,5), states_interp(i,6));
    arm1_w = R * [L, 0, 0; -L, 0, 0]' + pos;
    arm2_w = R * [0, L, 0; 0, -L, 0]' + pos;
    
    delete(findobj(gca, 'Tag', 'drone')); 
    plot3(arm1_w(1,:), arm1_w(2,:), arm1_w(3,:), 'r-o', 'LineWidth', 3, 'Tag', 'drone'); 
    plot3(arm2_w(1,:), arm2_w(2,:), arm2_w(3,:), 'b-o', 'LineWidth', 3, 'Tag', 'drone'); 
    
    xlim([min(states_interp(:,1))-0.5, max(states_interp(:,1))+0.5]);
    ylim([min(states_interp(:,2))-0.5, max(states_interp(:,2))+0.5]);
    zlim([0, max(states_interp(:,3))+1]);
    
    title(sprintf('Real-Time Path: %.2f s', t_realtime(i)));
    drawnow;
    writeVideo(v3, getframe(fig3));
end
close(v3);

% Helper Function 
function R = build_rotation(phi, theta, psi)
    R_phi   = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi   = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_psi * R_theta * R_phi;
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
