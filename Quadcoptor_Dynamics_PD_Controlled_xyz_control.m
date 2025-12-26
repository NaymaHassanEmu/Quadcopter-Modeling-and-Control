clc, clearvars, close all

% Physical Parameters
params.m = 0.468; % kg
params.I = diag([4.865e-3, 4.865e-3, 8.801e-3]); % Inertia matrix Ixx, Iyy, Izz
params.g = 9.81;

% Initial State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x0 = zeros(12, 1);
tilt_deg = 10;
x0(4:6) = deg2rad(tilt_deg); % Initial tilt in Roll, Pitch, Yaw

% Desired State: [z_d, z_dot_d, phi_d, theta_d, psi_d, p_d, q_d, r_d]
x_desired = [1, 2, 3, 0];


% Controller Gains: [Position_P, Position_D; Attitude_P, Attitude_D]
% Note: Increased Attitude P gain for better stability
K_p = [1.5, 3]; 
K_d = [2.5, 0.75];

% Simulate
tspan = [0 12];
[t, states] = ode45(@(t, x) quad_dynamics(t, x_desired, x, params,  K_p, K_d), tspan, x0);

% --- Plotting ---
figure('Color', 'w');
subplot(2,1,1);
plot(t, states(:, 1:3), 'LineWidth', 1.5);
title('3D Position (m)'); legend('x', 'y', 'z'); grid on;

subplot(2,1,2);
plot(t, rad2deg(states(:, 4:6)), 'LineWidth', 1.5);
title('Euler Angles (degrees)'); legend('roll', 'pitch', 'yaw'); grid on;



function dxdt = quad_dynamics(~, x_desired, x_state, params,  K_p, K_d)
    % Extract states
    % x_state = [x; y; z; phi; theta; psi; vx; vy; vz; p; q; r]
    x = x_state(3);     % x - position
    y = x_state(3);     % y - position
    z = x_state(3);     % Altitude
    phi = x_state(4);   % Roll
    theta = x_state(5); % Pitch
    psi = x_state(6);   % Yaw
    v = x_state(7:9);   % Linear velocities
    x_dot = v(1);       %Vx
    y_dot = v(2);       %Vy
    z_dot = v(3);       %Vz
    omega = x_state(10:12); % Angular velocities (p, q, r)

    % Parameters
    m = params.m;       % Mass
    g = 9.81;           % Gravity
    I = params.I;       % Inertia Matrix
    Ix = params.I(1,1);
    Iy = params.I(2,2);
    Iz = params.I(3,3);


    % Control Gains for PD controller
    K_pos_p = K_p(1);
    K_pos_d = K_d(1);
    K_att_p = K_p(2);
    K_att_d = K_d(2);
 
    % Desired State
    x_d = x_desired(1);
    y_d = x_desired(2);
    z_d = x_desired(3);
    psi_d = x_desired(4);



    % Euler rates
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_rates = T * omega;

    phi_dot = euler_rates(1);
    theta_dot = euler_rates(2);
    psi_dot = euler_rates(3);

    % Control Inputs

    % Total Thrust F
    F_total = (g + K_pos_d * (0 - z_dot) + K_pos_p * (z_d - z)) * (m / (cos(phi) * cos(theta)));
    %if F_total < 0, F_total = 0; end % Physical saturation
   
    
 
    % Rotation Matrix (Body to Inertial FoR)
    % Roll-Pitch-Yaw convention
    R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_psi * R_theta * R_phi;

    % Translational Acceleration
    accel = [0; 0; -g] + (R * [0; 0; F_total] / m);
    x_ddot = accel(1);
    y_ddot = accel(2);
    z_ddot = accel(3);

    % Desired Roll and Pitch Calculation
    dx = 1.85 * (x_d - x) + 0.75 * (0 - x_dot) + 1 * (0 - x_ddot);
    dy = 8.55 * (y_d - y) + 0.75 * (0 - y_dot) + 1 * (0 - y_ddot);
    dz = 1.85 * (z_d - z) + 0.75 * (0 - z_dot) + 1 * (0 - z_ddot);
    

    % Desired Roll (phi)
    phi_d = asin((dx * sin(psi) - dy * cos(psi)) / sqrt(dx^2 + dy^2 + (dz + g)^2));
    
    % Desired Pitch (theta)
    theta_d = atan((dx * cos(psi) + dy * sin(psi)) / (dz + g));
    
    % Desired Thrust (T)
    F_d = m * (dx * (sin(theta) * cos(psi) * cos(phi) + sin(psi) * sin(phi)) + ...
             dy * (sin(theta) * sin(psi) * cos(phi) - cos(psi) * sin(phi)) + ...
             (dz + g) * cos(theta) * cos(phi));


    % Roll, Pitch, and Yaw Moments
    M_phi = Ix * (K_att_d * (0 - phi_dot) + K_att_p * (phi_d - phi));
    M_theta = Iy * (K_att_d * (0 - theta_dot) + K_att_p * (theta_d - theta));
    M_psi = Iz * (K_att_d * (0 - psi_dot) + K_att_p * (psi_d - psi));
    M = [M_phi; M_theta; M_psi];

    % Motor angular velocities (w)
    k = 2.980e-6;   % Lift coefficient
    b = 1.140e-7;   % Drag coefficient
    l = 0.225;       % Arm length (Ensure 'l' is defined)
    w1 = (F_d/(4*k) - M_theta/(2*k*l) - M_psi/(4*b))^0.5;
    w2 = (F_d/(4*k) - M_phi/(2*k*l)   + M_psi/(4*b))^0.5;
    w3 = (F_d/(4*k) + M_theta/(2*k*l) - M_psi/(4*b))^0.5;
    w4 = (F_d/(4*k) + M_phi/(2*k*l)   + M_psi/(4*b))^0.5;
    
    w = [w1; w2; w3; w4;];

    % 3. Angular Acceleration
    % omega_dot = I^-1 * (-omega x I*omega + M)
    omega_dot = I \ (-cross(omega, I * omega) + M);

    % 4. Assemble state derivative
    dxdt = [v;             % dot_pos
            euler_rates;   % dot_euler
            accel;         % dot_v
            omega_dot];    % dot_omega
end

