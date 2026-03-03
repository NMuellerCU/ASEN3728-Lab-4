% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: RotationDerivativeFeedback.m
% Created: 3/03/26

function [Fc, Gc] = RotationDerivativeFeedback(var, m, g)
% Inputs: var - 12x1 aircraft state vector
%         m - mass of the quadrotor
%         g - gravitational acceleration
% Outputs: Fc - 3x1 vector of contol forces
%          Gc - 3x1 vector of control moments
% Methodology: The function computes the control forces based on the weight of the quadrotor and the moments based on the angular velocity components of the state vector. It applies a 0.004 Nm/(rad/sec) gain to the angular velocity components to compute the control moments, which stabilizes the quadrotor's rotation.

    % Unpack the state vector
    p = var(10);
    q = var(11);
    r = var(12);

    % Control forces and moments
    Fc = [0; 0; m * g];
    Gc = -0.004 * [p; q; r];
end