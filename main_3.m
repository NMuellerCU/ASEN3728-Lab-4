% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: main_task3.m
% Created: 3/24/26

clc;
clear;
close all;

%% Set-up for Task 3

m  = 0.068;    % Quadrotor mass [kg]
d  = 0.060;    % Radial distance from CG to propellor [m]
km = 0.0024;   % Control moment coefficient [N*m/N]
Ix = 5.8E-5;   % Body x-axis moment of inertia [kg*m^2]
Iy = 7.2E-5;   % Body y-axis moment of inertia [kg*m^2]
Iz = 1.0E-4;   % Body z-axis moment of inertia [kg*m^2]
nu = 1E-3;     % Aerodynamic force coefficient [N/(m/s)^2]
mu = 2E-6;     % Aerodynamic moment coefficient [N*m/(rad/s)^2]
g  = 9.81;     % Gravitational acceleration [m/s^2]

I_mat = [Ix 0  0;
         0  Iy 0;
         0  0  Iz];   % Principal inertia matrix [kg*m^2]

tspan = [0 10];

% State order:
% [x y z phi theta psi u v w p q r]^T

%% Initial conditions for Tasks 3.3 and 3.4

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

case_names = { ...
    'Case 1: +5 deg Roll', ...
    'Case 2: +5 deg Pitch', ...
    'Case 3: +0.1 rad/s Roll Rate', ...
    'Case 4: +0.1 rad/s Pitch Rate'};

% Figure number matrix: 8 groups total, 6 figures per group
fig = reshape(1:48, 6, 8)';

% Plot colors
col = {'b', 'k', 'r', 'g'};

%% Task 3.3: Linear closed-loop

for k = 1:4
    x0_lin = initial_conditions(:,k);

    [t_lin, x_lin] = ode45(@(t,var) Closed_Loop_Linearized(t, var, g, m, I_mat), tspan, x0_lin);

    % Compute controls over trajectory
    Fc_lin = zeros(length(t_lin),3);
    Gc_lin = zeros(length(t_lin),3);

    for i = 1:length(t_lin)
        [Fc_temp, Gc_temp] = InnerLoopFeedback(x_lin(i,:)');
        Fc_lin(i,:) = Fc_temp';
        Gc_lin(i,:) = Gc_temp';
    end

    % Build control input array: [Fc_z, Lc, Mc, Nc]
    control_data_lin = [Fc_lin(:,3), Gc_lin(:,1), Gc_lin(:,2), Gc_lin(:,3)];

    % Plot
    PlotAircraftSim(t_lin, x_lin, control_data_lin, fig(k,:), col{k});

    % Titles
    figure(fig(k,1)); sgtitle(['Task 3.3 Linear Closed-Loop: Inertial Position, ', case_names{k}]);
    figure(fig(k,2)); sgtitle(['Task 3.3 Linear Closed-Loop: Euler Angles, ', case_names{k}]);
    figure(fig(k,3)); sgtitle(['Task 3.3 Linear Closed-Loop: Body Velocities, ', case_names{k}]);
    figure(fig(k,4)); sgtitle(['Task 3.3 Linear Closed-Loop: Angular Velocities, ', case_names{k}]);
    figure(fig(k,5)); sgtitle(['Task 3.3 Linear Closed-Loop: Control Inputs, ', case_names{k}]);
    figure(fig(k,6)); title(['Task 3.3 Linear Closed-Loop: 3D Path, ', case_names{k}]);
end

%% Task 3.4: Nonlinear closed-loop

for k = 1:4
    x0_non = initial_conditions(:,k);

    [t_non, x_non] = ode45(@(t,var) Closed_Loop_Nonlinear(t, var, g, m, I_mat), tspan, x0_non);

    % Compute controls over trajectory
    Fc_non = zeros(length(t_non),3);
    Gc_non = zeros(length(t_non),3);

    for i = 1:length(t_non)
        [Fc_temp, Gc_temp] = InnerLoopFeedback(x_non(i,:)');
        Fc_non(i,:) = Fc_temp';
        Gc_non(i,:) = Gc_temp';
    end

    % Build control input array: [Fc_z, Lc, Mc, Nc]
    control_data_non = [Fc_non(:,3), Gc_non(:,1), Gc_non(:,2), Gc_non(:,3)];

    % Plot
    PlotAircraftSim(t_non, x_non, control_data_non, fig(k+4,:), col{k});

    % Titles
    figure(fig(k+4,1)); sgtitle(['Task 3.4 Nonlinear Closed-Loop: Inertial Position, ', case_names{k}]);
    figure(fig(k+4,2)); sgtitle(['Task 3.4 Nonlinear Closed-Loop: Euler Angles, ', case_names{k}]);
    figure(fig(k+4,3)); sgtitle(['Task 3.4 Nonlinear Closed-Loop: Body Velocities, ', case_names{k}]);
    figure(fig(k+4,4)); sgtitle(['Task 3.4 Nonlinear Closed-Loop: Angular Velocities, ', case_names{k}]);
    figure(fig(k+4,5)); sgtitle(['Task 3.4 Nonlinear Closed-Loop: Control Inputs, ', case_names{k}]);
    figure(fig(k+4,6)); title(['Task 3.4 Nonlinear Closed-Loop: 3D Path, ', case_names{k}]);
end

%% Task 3.7: Nonlinear reference tracking

x0_track = zeros(12,1);
tspan_track = [0 8];

% Case 1: Longitudinal Tracking
mode = 'longitudinal';

[t_long, x_long] = ode45(@(t,var) Closed_Loop_Nonlinear_RefTrack(t, var, g, m, I_mat, mode), tspan_track, x0_track);

control_data_long = zeros(length(t_long),4);

for i = 1:length(t_long)
    [Fc, Gc] = VelocityReferenceFeedback(t_long(i), x_long(i,:)', mode);
    control_data_long(i,:) = [Fc(3), Gc(1), Gc(2), Gc(3)];
end

fig_long = [49 50 51 52 53 54];
PlotAircraftSim(t_long, x_long, control_data_long, fig_long, 'c')

figure(fig_long(1)); sgtitle('Task 3.7 Longitudinal Tracking: Inertial Position');
figure(fig_long(2)); sgtitle('Task 3.7 Longitudinal Tracking: Euler Angles');
figure(fig_long(3)); sgtitle('Task 3.7 Longitudinal Tracking: Body Velocities');
figure(fig_long(4)); sgtitle('Task 3.7 Longitudinal Tracking: Angular Velocities');
figure(fig_long(5)); sgtitle('Task 3.7 Longitudinal Tracking: Control Inputs');
figure(fig_long(6)); title('Task 3.7 Longitudinal Tracking: 3D Path');

% Case 2: Lateral Tracking
mode = 'lateral';

[t_lat, x_lat] = ode45(@(t,var) Closed_Loop_Nonlinear_RefTrack(t, var, g, m, I_mat, mode), tspan_track, x0_track);

control_data_lat = zeros(length(t_lat),4);

for i = 1:length(t_lat)
    [Fc, Gc] = VelocityReferenceFeedback(t_lat(i), x_lat(i,:)', mode);
    control_data_lat(i,:) = [Fc(3), Gc(1), Gc(2), Gc(3)];
end

fig_lat = [55 56 57 58 59 60];
PlotAircraftSim(t_lat, x_lat, control_data_lat, fig_lat, 'c')

figure(fig_lat(1)); sgtitle('Task 3.7 Lateral Tracking: Inertial Position');
figure(fig_lat(2)); sgtitle('Task 3.7 Lateral Tracking: Euler Angles');
figure(fig_lat(3)); sgtitle('Task 3.7 Lateral Tracking: Body Velocities');
figure(fig_lat(4)); sgtitle('Task 3.7 Lateral Tracking: Angular Velocities');
figure(fig_lat(5)); sgtitle('Task 3.7 Lateral Tracking: Control Inputs');
figure(fig_lat(6)); title('Task 3.7 Lateral Tracking: 3D Path');