% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: var_dot.m
% Created: 3/3/26
%
% Objective: Create a simuulation of a quadrotor using ode45 to simulate
% the full nonlinear equations of motion. 
%
% Inputs: t - time, var - 12 x 1 aircraft state vector, g - accel due to
% gravity, m - mass, I - inertia matrix, d, km, nu, mu are remaining
% quadrotor parameters, motor_forces = [f1; f2; f3; f4] 4 x 1 vector of
% motor forces
%
% Outputs: var_dot - 12 x 1 derivative of the state vector


function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

    % Extract state variables
    pos = var(1:3);        % Position (x, y, z)
    euler = var(4:6);        % Velocity (vx, vy, vz)
    vel = var(7:9);      % Euler angles (phi, theta, psi)
    omega = var(10:12);    % Angular velocity (p, q, r)
    I_inverse_diag = diag(I).^-1;
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    T_euler = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi), -sin(phi);
               0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

    [F_aero, M_aero] = computeAerodynamicForces(vel, omega, nu, mu);



    % Compute forces and moments
    thrust = sum(motor_forces);  % Total thrust
    F = [0; 0; thrust];     % Force vector in body frame
    F_g = [0; 0; -m*g]; % gravity vector in body frame            
    % Compute rotational dynamics
    R = eul2rotm([psi theta phi], 'ZYX');          % Rotation matrix from Euler angles
    torque_with_z = computeTorque(motor_forces', d, km); % Compute control moments
    M_torque = torque_with_z(2:4);
    

    % Derivatives
    pos_dot = vel;                % Position derivative
    vel_dot = cross(vel,omega) +  (1/m) *((R * F_g ) + F + F_aero);   % Velocity derivative
    euler_dot = T_euler*omega;            % Euler angle derivative
    omega_dot = I \ (M_torque - cross(omega, I * omega) + M_aero); % Angular velocity derivative
    % Compute the thrust vector in the body frame
    % thrust_vector = R * [0; 0; thrust];  % DCM to inertial frame
    
    



    % Update the state derivatives with thrust vector
    var_dot = [pos_dot; euler_dot; vel_dot; omega_dot];
end