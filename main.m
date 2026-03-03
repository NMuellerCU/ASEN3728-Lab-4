% Contributors: Nate Mueller
% Course Number: ASEN 3801
% File Name: main.m
% Created On: 3/3/2026
clear; clc; close all;


m = 0.068; % Quadrotor mass [kg]
d = 0.060; %Radial distance from CG to propellor [m]
km = 0.0024; % Control moment coefficient [N*m/(N)]
Ix = 5.8E-5; % Body x-axis Moment of Inertia [kg*m^2]
Iy =7.2E-5; % Body y-axis Moment of Inertia [kg*m^2]
Iz = 1.0E-4; % Body z-axis Moment of Inertia [kg*m^2]
nu =1E-3; % Aerodynamic force coefficient [N/(m/s)^2]
mu = 2E-6; % Aerodynamic moment coefficient [N*m/(rad/s)^2]
g = 9.81; % [m/s^2]




% Inputs: t - time, var - 12 x 1 aircraft state vector, g - accel due to
% gravity, m - mass, I - inertia matrix, d, km, nu, mu are remaining
% quadrotor parameters, motor_forces = [f1; f2; f3; f4] 4 x 1 vector of
% motor forces
%
% Outputs: var_dot - 12 x 1 derivative of the state vector


% function var_dot = QuadrotorEOM(t, var, g, m, I, d, km, nu, mu, motor_forces)

I_mat = [Ix 0 0;
         0 Iy 0;
         0 0 Iz]; %matrix Principal MOI [kg*m^2]


data_flightpath = load("RSdata_nocontrol.mat");

t_span = 0:0.1:10; % s
eom_0 = data_flightpath.rt_estim.signals.values(1, :);
motor_forces_0 = data_flightpath.rt_motor.signals.values(1,:);
[t, x] = ode45(@(t, x) QuadrotorEOM(t, x, g, m, I_mat, d, km, nu, mu,motor_forces_0), t_span, eom_0);