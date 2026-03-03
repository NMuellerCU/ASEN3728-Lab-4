% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: QuadrotorEOM_Linearized.m
% Created: 3/03/26

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, deltaFc, deltaGc)
% Inputs: t - time column vector (not used in this function)
%         var - 12x1 aircraft state vector
%         g - gravitational acceleration
%         m - mass of the quadrotor
%         I - 3x3 inertia matrix of the quadrotor
%         deltaFc - 3x1 vector of force deviations from the steady hover trim condition
%         deltaGc - 3x1 vector of moment deviations from the steady hover trim condition
% Output: var_dot - 12x1 vector of state derivatives
% Methodology: The function computes the linearized equations of motion for a quadrotor and packs the resulting state derivatives into a column vector.
    
    % Unpack the state vector
    x = var(1);
    y = var(2);
    z = var(3);
    phi = var(4);
    theta = var(5);
    psi = var(6);
    u = var(7);
    v = var(8);
    w = var(9);
    p = var(10);
    q = var(11);
    r = var(12);

    % Linearized equations of motion
    x_dot = u;
    y_dot = v;
    z_dot = w;
    phi_dot = p;
    theta_dot = q;
    psi_dot = r;
    u_dot = -g * theta + deltaFc(1) / m;
    v_dot = g * phi + deltaFc(2) / m;
    w_dot = deltaFc(3) / m;
    p_dot = deltaGc(1) / I(1,1);
    q_dot = deltaGc(2) / I(2,2);
    r_dot = deltaGc(3) / I(3,3);

    % Pack the state derivative vector
    var_dot = [x_dot; y_dot; z_dot; phi_dot; theta_dot; psi_dot; u_dot; v_dot; w_dot; p_dot; q_dot; r_dot];
end