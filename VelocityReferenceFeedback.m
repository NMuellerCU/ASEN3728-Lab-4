% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: VelocityReferenceFeedback.m
% Created: 4/3/26

% Inputs: t - current simulation time
%         var - 12x1 aircraft state vector
%         mode - string specifying the tracking case:
%                'longitudinal', 'lateral', or 'both'
% Output: Fc - 3x1 vector of control forces
%         Gc - 3x1 vector of control moments
%
% Methodology: This function implements the closed-loop control law for
% velocity reference tracking. The controller combines inner-loop attitude
% stabilization with outer-loop velocity feedback to command roll and pitch
% moments. Time-based reference velocities are used to achieve a desired
% displacement, and the resulting control forces and moments are returned.

function [Fc, Gc] = VelocityReferenceFeedback(t, var, mode)

    % Parameters
    Ix = 5.8e-5;
    Iy = 7.2e-5;
    m  = 0.068;
    g  = 9.81;

    % Inner-loop gains from Task 3.1
    k1_lat  = 12*Ix;
    k2_lat  = 20*Ix;
    k1_long = 12*Iy;
    k2_long = 20*Iy;
    k_r     = 0.004;

    % Outer-loop gains from Task 3.5
    k3_lat  = 5.273e-05;
    k3_long = 6.548e-05;

    % Unpack states
    phi   = var(4);
    theta = var(5);
    u     = var(7);
    v     = var(8);
    p     = var(10);
    q     = var(11);
    r     = var(12);

    % Default references
    ur = 0;
    vr = 0;

    % Reference commands by mode
    if t < 2
        switch lower(mode)
            case 'longitudinal'
                ur = 0.5;
                vr = 0;
            case 'lateral'
                ur = 0;
                vr = 0.5;
            case 'both'
                ur = 0.5;
                vr = 0.5;
            otherwise
                error('Invalid mode. Use ''longitudinal'', ''lateral'', or ''both''.')
        end
    end

    % Control laws
    Lc = -k1_lat*p - k2_lat*phi + k3_lat*(vr - v);

    % Longitudinal uses sign option 2
    Mc = -k1_long*q - k2_long*theta - k3_long*(ur - u);

    Nc = -k_r*r;

    % Hover thrust
    Fc = [0; 0; -m*g];
    Gc = [Lc; Mc; Nc];
end