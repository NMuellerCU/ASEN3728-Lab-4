% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: Closed_Loop_Linearized.m
% Created: 3/03/26

% Inputs: t - time column vector (not used in this function)
%         var - 12x1 aircraft state vector
%         g - gravitational acceleration
%         m - mass of the quadrotor
%         I - 3x3 inertia matrix of the quadrotor
%         deltaFc - 3x1 vector of force deviations from the steady hover trim condition
%         deltaGc - 3x1 vector of moment deviations from the steady hover trim condition
% Output: var_dot - 12x1 vector of state derivatives
% Methodology: The function computes the linearized equations of motion for a quadrotor and packs the resulting state derivatives into a column vector.

function var_dot = Closed_Loop_Linearized(t, var, g, m, I)

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

    % Closed-loop control
    [Fc, Gc] = InnerLoopFeedback(var);

    % Deviations from hover trim
    deltaFc = Fc - [0; 0; -m*g];
    deltaGc = Gc;

    % Linearized equations of motion
    x_dot = u;
    y_dot = v;
    z_dot = w;

    phi_dot   = p;
    theta_dot = q;
    psi_dot   = r;

    u_dot = -g * theta;
    v_dot =  g * phi;
    w_dot = (1/m) * deltaFc(3);

    p_dot = (1/I(1,1)) * deltaGc(1);
    q_dot = (1/I(2,2)) * deltaGc(2);
    r_dot = (1/I(3,3)) * deltaGc(3);

    var_dot = [x_dot; y_dot; z_dot; ...
               phi_dot; theta_dot; psi_dot; ...
               u_dot; v_dot; w_dot; ...
               p_dot; q_dot; r_dot];
end
