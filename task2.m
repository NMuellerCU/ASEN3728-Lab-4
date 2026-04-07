% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: task2.m
% Created: 3/24/26

clc;
clear;
close all;

% Initialize variables
m = 0.068; % Quadrotor mass [kg]
d = 0.060; %Radial distance from CG to propellor [m]
km = 0.0024; % Control moment coefficient [N*m/(N)]
Ix = 5.8E-5; % Body x-axis Moment of Inertia [kg*m^2]
Iy =7.2E-5; % Body y-axis Moment of Inertia [kg*m^2]
Iz = 1.0E-4; % Body z-axis Moment of Inertia [kg*m^2]
nu =1E-3; % Aerodynamic force coefficient [N/(m/s)^2]
mu = 2E-6; % Aerodynamic moment coefficient [N*m/(rad/s)^2]
g = 9.81; % [m/s^2]

I_mat = [Ix 0 0;
         0 Iy 0;
         0 0 Iz]; %matrix Principal MOI [kg*m^2]

motor_forces = [m*g/4; m*g/4; m*g/4; m*g/4]; % Steady hover trim motor forces [N]
deltaFc = [0; 0; 0]; % Force deviations from the steady hover trim condition [N]
deltaGc = [0; 0; 0]; % Moment deviations from the steady hover trim condition [N*m]

initial_conditions = [0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0;
                        5*pi/180, 0, 0, 0, 0, 0;
                        0, 5*pi/180, 0, 0, 0, 0;
                        0, 0, 5*pi/180, 0, 0, 0;
                        0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0, 0, 0;
                        0, 0, 0, 0.1, 0, 0;
                        0, 0, 0, 0, 0.1, 0;
                        0, 0, 0, 0, 0, 0.1]; % 6 columns of initial condition state vectors for the 6 deviations

fig = [1, 2, 3, 4, 5, 6;
         7, 8, 9, 10, 11, 12;
         13, 14, 15, 16, 17, 18;
         19, 20, 21, 22, 23, 24;
         25, 26, 27, 28, 29, 30;
         31, 32, 33, 34, 35, 36;
         37, 38, 39, 40, 41, 42;
         44, 45, 46, 47, 48, 49;
         51, 52, 53, 54, 55, 56;
         43, 50, 57, 0, 0, 0];

titles = ["DEVIATION BY +5 DEG IN ROLL";
          "DEVIATION BY +5 DEG IN PITCH";
          "DEVIATION BY +5 DEG IN YAW";
          "DEVIATION BY +0.1 RAD/SEC IN ROLL RATE";
          "DEVIATION BY +0.1 RAD/SEC IN PITCH RATE";
          "DEVIATION BY +0.1 RAD/SEC IN YAW RATE"];

tspan = 0:0.01:10;

% Plot (3.1 & 3.2)
for i = 1:6
    % Simulate and plot the deviation with the nonlinear EOMs
    [t_1,x_1] = ode45(@(t_1,x_1) QuadrotorEOM(t_1, x_1, g, m, I_mat, d, km, nu, mu, motor_forces), tspan, initial_conditions(:,i));
    control_input_array_1 = [-sum(motor_forces); 0; 0; 0]*ones(1, length(t_1));
    PlotAircraftSim(t_1, x_1, control_input_array_1, fig(i,:), 'b');
    
    % Simulate and plot the deviation with the linearized EOMs
    [t_2,x_2] = ode45(@(t_2,x_2) QuadrotorEOM_Linearized(t_2, x_2, g, m, I_mat, deltaFc, deltaGc), tspan, initial_conditions(:,i));
    control_input_array_2 = [-sum(motor_forces); 0; 0; 0]*ones(1, length(t_2));
    PlotAircraftSim(t_2, x_2, control_input_array_2, fig(i,:), '--r');

    % Add title and legend to the first plot of each set
    figure(fig(i,1));
    subplot(311);
    legend('Nonlinear EOM', 'Linearized EOM', 'Location', 'northoutside', 'Orientation', 'horizontal');
    sgtitle(titles(i));
end

% Plot (3.5)
for j = 4:6
    % Simulate and plot the deviation with the nonlinear EOMs without rate feedback control
    [t_3,x_3] = ode45(@(t_3,x_3) QuadrotorEOM(t_3, x_3, g, m, I_mat, d, km, nu, mu, motor_forces), tspan, initial_conditions(:,j));
    control_input_array_3 = [-sum(motor_forces); 0; 0; 0]*ones(1, length(t_3));
    PlotAircraftSim(t_3, x_3, control_input_array_3, fig(3+j,:), 'b');
    
    % Simulate and plot the deviation with the nonlinear EOMs with rate feedback control
    [t_4,x_4] = ode45(@(t_4,x_4) QuadrotorEOMwithRateFeedback(t_4, x_4, g, m, I_mat, nu, mu), tspan, initial_conditions(:,j));
    control_input_array_4 = ones(4, length(t_4));
    motor_forces_control = ones(4, length(t_4));
    for k = 1:length(t_4)
        [Fc, Gc] = RotationDerivativeFeedback(transpose(x_4(k,:)), m, g);
        control_input_array_4(:,k) = [-Fc(3); Gc(1); Gc(2); Gc(3)];
        motor_forces_control(:,k) = ComputeMotorForces(Fc, Gc, d, km);
    end
    PlotAircraftSim(t_4, x_4, control_input_array_4, fig(3+j,:), '--r');

    % Plot each motor's thrust force for both cases
    figure(fig(10, j-3));
    subplot(411);
    hold on;
    plot(t_3, -motor_forces(1)*ones(1, length(t_3)), 'b');
    plot(t_4, motor_forces_control(1,:), '--r');
    ylabel('Motor 1 Force (N)');
    subplot(412);
    hold on;
    plot(t_3, -motor_forces(2)*ones(1, length(t_3)), 'b');
    plot(t_4, motor_forces_control(2,:), '--r');
    ylabel('Motor 2 Force (N)');
    subplot(413);
    hold on;
    plot(t_3, -motor_forces(3)*ones(1, length(t_3)), 'b');
    plot(t_4, motor_forces_control(3,:), '--r');
    ylabel('Motor 3 Force (N)');
    subplot(414);
    hold on;
    plot(t_3, -motor_forces(4)*ones(1, length(t_3)), 'b');
    plot(t_4, motor_forces_control(4,:), '--r');
    ylabel('Motor 4 Force (N)');
    xlabel('Time (s)');
    grid on;
    hold off;

    % Add title and legend to the first plot of each set
    figure(fig(j+3,1));
    subplot(311);
    legend('Nonlinear EOM', 'Nonlinear EOM Rate Feedback', 'Location', 'northoutside', 'Orientation', 'horizontal');
    sgtitle(titles(j));
end