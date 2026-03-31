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

time_data = data_flightpath.rt_estim.time; %extract time data (Nx1)
state_data = data_flightpath.rt_estim.signals.values; %extract state data (Nx12)
motor_data = data_flightpath.rt_motor.signals.values; %extract motor data (Nx4)


t_span = [0 10]; % time pspan for ode45 test [s]
eom_0 = state_data(1, :)'; 
eom_hover = zeros(12,1);% initial conditions for 12x1 eom variables;
eom_hover(8) = 5;
motor_forces_hover = m*g/4*ones(4,1);
% trim_forces = f_trim*ones(4,1);
% motor_forces_0 = motor_data(1,:).';
[t, x] = ode45(@(t, x)  QuadrotorEOM(t, x, g, m, I_mat, d, km, nu, mu,motor_forces_hover), t_span, eom_hover);


%% plots



vel_x = x(:,7);
vel_y = x(:,8);
vel_z = x(:,9);
ang_rate_p = x(:,10);
ang_rate_q = x(:,11);
ang_rate_r = x(:,12);

% figure(2);
% plot3(vel_x, vel_y, vel_z);
% xlabel("v_x (m/s)"); ylabel("v_y (m/s)"); zlabel("v_z (m/s)");
% title("Velocity of Quadrotor in Trim State with Aerodynamic Forces and Moments")
% 
% figure(3);
% plot3(ang_rate_p, ang_rate_q, ang_rate_r);
% xlabel("p (rad/s)"); ylabel("q (rad/s)"); zlabel("r (rad/s)");
% title("Angular Rate of Quadrotor in Trim State with Aerodynamic Forces and Moments")

% t_span = linspace(0, 10, length(time_data)); % time pspan for ode45 test [s]
% eom_0 = state_data(1, :)'; % initial conditions for 12x1 eom variables;
% trim_state = zeros(12,1);
% f_trim = m*g/4;
% trim_forces = f_trim*on   es(4,1);
% motor_forces_0 = motor_data(1,:).';
% [t, x] = ode45(@(t, x)  QuadrotorEOM(t, x, g, m, I_mat, d, km, nu, mu,trim_forces), t_span, trim_state);

% Extract the final state and plot the results using PlotAircraftSim.m
fig = [1 2 3 4 5 6];
motor_forces_hover_length = m*g/4*ones(69,4);
PlotAircraftSim(t, x, motor_forces_hover_length, fig, 'b');

