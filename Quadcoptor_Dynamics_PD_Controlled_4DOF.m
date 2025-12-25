clc, clearvars


% Physical Parameters
params.m = 0.468; % kg
params.I = diag([4.865e-3, 4.865e-3, 8.801e-3]); % Inertia matrix Ixx, Iyy, Izz

% Initial State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x0 = zeros(12, 1);
%x0(3) = 1.0; % Start at 1m height

% Add a slight initial tilt (10 degrees)
tilt_deg = 10;
x0(4) = tilt_deg * (pi/180); % Convert Roll to Radians
x0(5) = tilt_deg * (pi/180); % Convert pitch to Radians
x0(6) = tilt_deg * (pi/180); % Convert yaw to Radians

% Desired State: []
x_desired = zeros(8, 1);
x_desired(1) = 5; % Desired 5m height

% Controller Gains
K_p = [1.5 6 6 6]
K_d = [2.5 1.75 1.75 1.75]

% Simulate for 50 seconds
tspan = [0 10];
[t, states] = ode45(@(t, x) quad_dynamics(t, x, controller(x_desired, x, params,  K_p, K_d), params), tspan, x0);

% Plot Results
subplot(2,1,1);
plot(t, states(:, 1:3));
title('3D Position'); legend('x', 'y', 'z'); grid on;

subplot(2,1,2);
plot(t, states(:, 4:6));
title('Euler Angles'); legend('roll', 'pitch', 'yaw'); grid on;




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
    K_z_p = K_p(1);
    K_z_d = K_d(1);
    K_phi_p = K_p(2);
    K_theta_p = K_p(3);
    K_psi_p = K_p(4);
    K_phi_d = K_d(2);
    K_theta_d = K_d(3);
    K_psi_d = K_d(4);

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
    z_dot = x_state(9)  %Vz
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
    M_phi = Ix * (K_phi_d * (phi_dot_d - phi_dot) + K_phi_p * (phi_d - phi));
    M_theta = Iy * (K_theta_d * (theta_dot_d - theta_dot) + K_theta_p * (theta_d - theta));
    M_psi = Iz * (K_psi_d * (psi_dot_d - psi_dot) + K_psi_p * (psi_d - psi));

    %Thrust and Motor Speeds (w1-w4)
    % Total Thrust F
    F = (g + K_z_d * (z_dot_d - z_dot) + K_z_p * (z_d - z)) * (m / (cos(phi) * cos(theta)));
    
    controll_input = [F; M_phi; M_theta; M_psi];
end
