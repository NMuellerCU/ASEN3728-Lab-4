% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: Closed_Loop_Nonlinear_RefTrack.m
% Created: 4/3/26

% Inputs: t - time column vector
%         var - 12x1 aircraft state vector
%         g - gravitational acceleration
%         m - mass of the quadrotor
%         I_mat - 3x3 inertia matrix of the quadrotor
%         mode - string specifying the tracking case:
%                'longitudinal', 'lateral', or 'both'
% Output: var_dot - 12x1 vector of state derivatives
%
% Methodology: This function computes the full nonlinear equations of motion
% for the quadrotor under closed-loop control with velocity reference tracking.
% The control inputs are obtained from the VelocityReferenceFeedback function,
% and the resulting forces and moments are applied to the nonlinear translational
% and rotational dynamics to compute the time derivatives of the state vector.

function var_dot = Closed_Loop_Nonlinear_RefTrack(t, var, g, m, I_mat, mode)

    % Unpack states
    x     = var(1);
    y     = var(2);
    z     = var(3);
    phi   = var(4);
    theta = var(5);
    psi   = var(6);
    u     = var(7);
    v     = var(8);
    w     = var(9);
    p     = var(10);
    q     = var(11);
    r     = var(12);

    % Control inputs
    [Fc, Gc] = VelocityReferenceFeedback(t, var, mode);

    Xc = Fc(1);
    Yc = Fc(2);
    Zc = Fc(3);

    Lc = Gc(1);
    Mc = Gc(2);
    Nc = Gc(3);

    Ix = I_mat(1,1);
    Iy = I_mat(2,2);
    Iz = I_mat(3,3);

    % Position kinematics
    x_dot =  cos(theta)*cos(psi)*u + ...
             (sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi))*v + ...
             (cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi))*w;

    y_dot =  cos(theta)*sin(psi)*u + ...
             (sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi))*v + ...
             (cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi))*w;

    z_dot = -sin(theta)*u + ...
             sin(phi)*cos(theta)*v + ...
             cos(phi)*cos(theta)*w;

    % Euler angle kinematics
    phi_dot   = p + sin(phi)*tan(theta)*q + cos(phi)*tan(theta)*r;
    theta_dot = cos(phi)*q - sin(phi)*r;
    psi_dot   = sin(phi)*sec(theta)*q + cos(phi)*sec(theta)*r;

    % Translational dynamics
    u_dot = r*v - q*w - g*sin(theta) + (1/m)*Xc;
    v_dot = p*w - r*u + g*cos(theta)*sin(phi) + (1/m)*Yc;
    w_dot = q*u - p*v + g*cos(theta)*cos(phi) + (1/m)*Zc;

    % Rotational dynamics
    p_dot = ((Iy - Iz)/Ix)*q*r + (1/Ix)*Lc;
    q_dot = ((Iz - Ix)/Iy)*p*r + (1/Iy)*Mc;
    r_dot = ((Ix - Iy)/Iz)*p*q + (1/Iz)*Nc;

    % Pack state derivative vector
    var_dot = [x_dot; y_dot; z_dot; ...
               phi_dot; theta_dot; psi_dot; ...
               u_dot; v_dot; w_dot; ...
               p_dot; q_dot; r_dot];
end