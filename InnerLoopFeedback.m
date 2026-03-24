% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: InnerLoopFeedback.m
% Created: 3/10/26
%
% Objective: 
% Calculate the control vectors Fc and Gc The control force in the body 
% z-direction is equal to the weight of the quadrotor
%
% Inputs: 12 x 1 Aircraft State - var
%
% Outputs: Fc and Gc control vectors

function [Fc, Gc] = InnerLoopFeedback(var)

    % constant parameters of the quadcoptor
    Ix = 5.8e-5;
    Iy = 7.2e-5;
    m = 0.068;
    g = 9.81;

    % gains
    k1_lat = 12*Ix;
    k2_lat = 20*Ix;

    k1_long = 12*Iy;
    k2_long = 20*Iy;

    k_r = 0.004; % from Task 2.3

    % states
    phi = var(7);
    theta = var(8);
    p = var(10);
    q = var(11);
    r = var(12);

    % control laws
    Lc = -k1_lat*p - k2_lat*phi;
    Mc = -k1_long*q - k2_long*theta;
    Nc = -k_r*r;

    Fc = [0;0;m*g];
    Gc = [Lc; Mc; Nc];
 
end