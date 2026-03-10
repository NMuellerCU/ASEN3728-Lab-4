% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: ComputeMotorForces.m
% Created: 3/10/26

function motor_forces = ComputeMotorForces(Fc, Gc, d, km)
% Inputs: Fc - 3x1 vector of contol forces
%         Gc - 3x1 vector of contol moments
%         d - distance from the quadrotor's CG to each motor
%         km - control moment coefficient
% Output: motor_forces - 4x1 vector of forces produced by each motor
% Methodology: The function computes the motor thrust forces required to achieve the desired control forces and moments. It constructs a matrix that relates the control forces and moments to the motor forces, and then solves for the motor forces using the inverse of matrix.

    % Create matrix A that relates control forces and moments to motor forces
    A = [-1, -1, -1, -1;
         -d/sqrt(2), -d/sqrt(2), d/sqrt(2), d/sqrt(2);
         d/sqrt(2), -d/sqrt(2), -d/sqrt(2), d/sqrt(2);
         km, -km, km, -km];

    % Solve for the motor forces using the inverse of matrix A
    motor_forces = A \ [Fc(3); Gc];

end