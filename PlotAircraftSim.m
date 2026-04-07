% Contributors: Emma Haas
% Course number: ASEN 3801
% File name: PlotAircraftSim.m
% Created: 3/3/26
%
% Objective: Write a function to plot the full aircraft states and control
% inputs that result from a simulation run. This function should plot 6
% figures, four figures with three subplots each for the inertial position,
% Euler angles, inertial velocity in body frame, and angular velocity. The
% fifth figure should be one figure with four subplots for each control
% input variable. The last figure should be the 3D path of the aircraft
% with positive hight upward in the figure, with the start in green and the
% end in red with different colored markers.
%
% Inputs: a vector of length n with time, the 12 x n aircraft state array,
% and the 4 x n array of control inputs, the 6 x 1 vector of figure numbers to plot over,
% and the string col indicating the color/plotting option  
%
% Outputs: the 6 figures mentioned above

function PlotAircraftSim(time, aircraft_state_array, control_input_array, fig, col)

    %% Figures 1 - 4: 3 subplots each
    
    % Plot inertial position
    figure(fig(1));
    subplot(311);
    plot(time, aircraft_state_array(:,1), col); hold on;
    ylabel('Inertial Position X [m]');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,2), col); hold on;
    ylabel('Inertial Position Y [m]');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,3), col); hold on;
    ylabel('Inertial Position Z [m]');
    xlabel('Time [s]');
    grid on;

    % Plot Euler angles
    figure(fig(2));
    subplot(311);
    plot(time, aircraft_state_array(:,4), col); hold on;
    ylabel('\phi [rad]');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,5), col); hold on;
    ylabel('\theta [rad]');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,6), col); hold on;
    ylabel('\psi [rad]');
    xlabel('Time [s]');
    grid on;

    % Plot body-frame velocities
    figure(fig(3));
    subplot(311);
    plot(time, aircraft_state_array(:,7), col); hold on;
    ylabel('u [m/s]');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,8), col); hold on;
    ylabel('v [m/s]');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,9), col); hold on;
    ylabel('w [m/s]');
    xlabel('Time [s]');
    grid on;

    % Plot angular velocities
    figure(fig(4));
    subplot(311);
    plot(time, aircraft_state_array(:,10), col); hold on;
    ylabel('p [rad/s]');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,11), col); hold on;
    ylabel('q [rad/s]');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,12), col); hold on;
    ylabel('r [rad/s]');
    xlabel('Time [s]');
    grid on;

    %% Figure 5: control inputs

    figure(fig(5));
    subplot(411);
    plot(time, control_input_array(:,1), col); hold on;
    ylabel('F_{c,z} [N]');
    grid on;

    subplot(412);
    plot(time, control_input_array(:,2), col); hold on;
    ylabel('L_c [N m]');
    grid on;

    subplot(413);
    plot(time, control_input_array(:,3), col); hold on;
    ylabel('M_c [N m]');
    grid on;

    subplot(414);
    plot(time, control_input_array(:,4), col); hold on;
    ylabel('N_c [N m]');
    xlabel('Time [s]');
    grid on;

    %% Figure 6: 3D path

    figure(fig(6));
    plot3(aircraft_state_array(:,1), aircraft_state_array(:,2), aircraft_state_array(:,3), col);
    hold on;
    scatter3(aircraft_state_array(1,1), aircraft_state_array(1,2), aircraft_state_array(1,3), 40, 'g', 'filled');
    scatter3(aircraft_state_array(end,1), aircraft_state_array(end,2), aircraft_state_array(end,3), 40, 'r', 'filled');
    xlabel('X Position [m]');
    ylabel('Y Position [m]');
    zlabel('Z Position [m]');
    title('3D Path of the Aircraft');
    grid on;
    view(3);

end