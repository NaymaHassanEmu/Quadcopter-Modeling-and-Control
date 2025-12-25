clc, clearvars


% Physical Parameters
params.m = 1.0; % kg
params.I = diag([0.01, 0.01, 0.02]); % Inertia matrix Ixx, Iyy, Izz

% Initial State: [x, y, z, phi, theta, psi, vx, vy, vz, p, q, r]
x0 = zeros(12, 1);
x0(3) = 1.0; % Start at 1m height

% Add a slight initial tilt (10 degrees)
%tilt_deg = 10;
%x0(4) = tilt_deg * (pi/180); % Convert Roll to Radians

% Nominal Control for Hover
u_hover = [params.m * 9.81; 0; 0; 0]; 

% Simulate for 50 seconds
tspan = [0 50];
[t, states] = ode45(@(t, x) quad_dynamics(t, x, u_hover, params), tspan, x0);

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
    M = u(2:4);         % [M1; M2; M3]

    % 1. Rotation Matrix (Body to Inertial FoR)
    % Roll-Pitch-Yaw convention
    R_phi = [1 0 0; 0 cos(phi) -sin(phi); 0 sin(phi) cos(phi)];
    R_theta = [cos(theta) 0 sin(theta); 0 1 0; -sin(theta) 0 cos(theta)];
    R_psi = [cos(psi) -sin(psi) 0; sin(psi) cos(psi) 0; 0 0 1];
    R = R_phi * R_theta * R_psi;

    % 2. Translational Acceleration
    accel = [0; 0; -g] + (R * [0; 0; F_total] / m);

    % 3. Angular Kinematics (Euler rates)
    T = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
         0, cos(phi),            -sin(phi);
         0, sin(phi)/cos(theta), cos(phi)/cos(theta)];
    euler_rates = T * omega;

    % 4. Angular Acceleration
    % omega_dot = I^-1 * (-omega x I*omega + M)
    omega_dot = inv(I) * (-cross(omega, I * omega) + M);

    % Assemble state derivative
    dxdt = [v;             % dot_pos
            euler_rates;   % dot_euler
            accel;         % dot_v
            omega_dot];    % dot_omega
end



