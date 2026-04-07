% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: QuadrotorEOMwithRateFeedback.m
% Created: 3/10/26

function var_dot = QuadrotorEOMwithRateFeedback(t, var, g, m, I, nu, mu)
% Inputs: t - time column vector (not used in this function)
%         var - 12x1 aircraft state vector
%         g - gravitational acceleration
%         m - mass of the quadrotor
%         I - 3x3 inertia matrix of the quadrotor
%         nu - aerodynamic force coefficient
%         mu - aerodynamic moment coefficient
% Output: var_dot - 12x1 vector of state derivatives
% Methodology: The function computes the nonlinear equations of motion for a quadrotor with feedback control based on the current state of the system. The feedback controller computes the necessary forces and moments to stabilize the quadrotor, and these are incorporated into the equations of motion to compute the state derivatives.

    % Extract state variables
    pos = var(1:3);        % Position (x, y, z)
    euler = var(4:6);        % Euler angles (phi, theta, psi)
    vel = var(7:9);      % Velocity (vx, vy, vz)
    omega = var(10:12);    % Angular velocity (p, q, r)

    phi = euler(1);
    theta = euler(2);
    psi = euler(3);
    T_euler = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi), -sin(phi);
               0, sin(phi)*sec(theta), cos(phi)*sec(theta)];

    % Compute forces and moments
    [Fc, Gc] = RotationDerivativeFeedback(var, m, g);
    [F_aero, M_aero] = computeAerodynamicForces(vel, omega, nu, mu);
    a_g = g*[-sin(theta); cos(theta)*sin(phi); cos(theta)*cos(phi)]; % gravity vector in body frame

    % Compute rotational dynamics
    R = eul2rotm([psi theta phi], 'ZYX');          % Rotation matrix from body frame to inertial frame

    % Derivatives   
    pos_dot = R*vel;                % Position derivative
    vel_dot = cross(vel,omega) + (1/m) *(-Fc + F_aero) + a_g;   % Velocity derivative
    euler_dot = T_euler*omega;            % Euler angles derivative
    omega_dot = I \ (Gc - cross(omega, I * omega) + M_aero); % Angular velocity derivative

    % Pack the state derivative vector
    var_dot = [pos_dot; euler_dot; vel_dot; omega_dot];
end