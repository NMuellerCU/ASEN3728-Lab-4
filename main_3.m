% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: main_task3.m
% Created: 3/24/26

clc;
clear;
close all;

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

t = [0 10];

% State order:
% [x y z phi theta psi u v w p q r]^T

% Case 1: +5 deg roll
case_1 = zeros(12,1);
case_1(4) = deg2rad(5); 

% Case 2: +5 deg pitch
case_2 = zeros(12,1);
case_2(5) = deg2rad(5); 

% Case 3: +0.1 rad/s roll rate
case_3 = zeros(12,1);
case_3(10) = 0.1;

% Case 4: +0.1 rad/s pitch rate
case_4 = zeros(12,1);
case_4(11) = 0.1;

initial_conditions = [case_1, case_2, case_3, case_4];

% fig # vector for later: 

fig = reshape(1:48, 6, 8)';

% colors to differentiate each case: 

col = ['b', 'k', 'r', 'g']; % 1 - blue, 2 - black, 3 - red, 4 - green

% Task 3.3: Linear closed-loop

for k = 1:4
    x_lin = initial_conditions(:,k);

    [t_lin, x_lin] = ode45(@(t,x) Closed_Loop_Linearized(t,x,g,m,I_mat), t, x_lin);

    % Compute controls over trajectory
    Fc_lin = zeros(length(t_lin),3);
    Gc_lin = zeros(length(t_lin),3);

    for i = 1:length(t_lin)
        [Fc_temp, Gc_temp] = InnerLoopFeedback(x_lin(i,:)');
        Fc_lin(i,:) = Fc_temp';
        Gc_lin(i,:) = Gc_temp';
    end

    % control forces
    
    motor_data_lin = [Fc_lin(:,3), Gc_lin(:,1), Gc_lin(:,2), Gc_lin(:,3)];

    % plots
    
    PlotAircraftSim(t_lin, x_lin, motor_data_lin, fig(k,:), col(k));
    

end


% Task 3.4: Nonlinear closed-loop

for k = 1:4
    x_non = initial_conditions(:,k);

    [t_non, x_non] = ode45(@(t,var) Closed_Loop_Nonlinear(t,var,g,m,I_mat), t, x_non);

    % Compute controls over trajectory
    Fc_non = zeros(length(t_non),3);
    Gc_non = zeros(length(t_non),3);

    for i = 1:length(t_non)
        [Fc_temp, Gc_temp] = InnerLoopFeedback(x_non(i,:)');
        Fc_non(i,:) = Fc_temp';
        Gc_non(i,:) = Gc_temp';
    end

    % control forces

    motor_data_non = [Fc_non(:,3), Gc_non(:,1), Gc_non(:,2), Gc_non(:,3)];

    % plots

    PlotAircraftSim(t_non, x_non, motor_data_non, fig(k+4,:), col(k));


end

