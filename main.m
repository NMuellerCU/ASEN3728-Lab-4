clear; clc; close all;


m = 0.068; % Quadrotor mass [kg]
d = 0.060; %Radial distance from CG to propellor [m]
km = 0.0024; % Control moment coefficient [N*m/(N)]
Ix = 5.8E-5; % Body x-axis Moment of Inertia [kg*m^2]
Iy =7.2E-5; % Body y-axis Moment of Inertia [kg*m^2]
Iz = 1.0E-4; % Body z-axis Moment of Inertia [kg*m^2]
v =1E-3; % Aerodynamic force coefficient [N/(m/s)^2]
mu = 2E-6; % Aerodynamic moment coefficient [N*m/(rad/s)^2]