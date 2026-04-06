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
    ylabel('Inertial Position (X)');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,2), col); hold on;
    ylabel('Inertial Position (Y)');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,3), col); hold on;
    ylabel('Inertial Position (Z)');
    xlabel('Time (s)');
    grid on;

    % Plot Euler angles
    figure(fig(2));
    subplot(311);
    plot(time, aircraft_state_array(:,4), col); hold on;
    ylabel('Roll (phi)');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,5), col); hold on;
    ylabel('Pitch (theta)');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,6), col); hold on;
    ylabel('Yaw (psi)');
    xlabel('Time (s)');
    grid on;

    % Plot inertial velocity in body frame
    figure(fig(3));
    subplot(311);
    plot(time, aircraft_state_array(:,7), col); hold on;
    ylabel('Velocity (U)');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,8), col); hold on;
    ylabel('Velocity (V)');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,9), col); hold on;
    ylabel('Velocity (W)');
    xlabel('Time (s)');
    grid on;

    % Plot angular velocity
    figure(fig(4));
    subplot(311);
    plot(time, aircraft_state_array(:,10), col); hold on;
    ylabel('P (Roll Rate)');
    grid on;

    subplot(312);
    plot(time, aircraft_state_array(:,11), col); hold on;
    ylabel('Q (Pitch Rate)');
    grid on;

    subplot(313);
    plot(time, aircraft_state_array(:,12), col); hold on;
    ylabel('R (Yaw Rate)');
    xlabel('Time (s)');
    grid on;

    %% Figure 5: control inputs - w/ 4 subplots 

    % Plot control inputs
    figure(fig(5));
    subplot(411);
    plot(time, control_input_array(:,1), col); hold on;
    ylabel('Zc (Thrust)');
    grid on;

    subplot(412);
    plot(time, control_input_array(:,2), col); hold on;
    ylabel('Lc (Roll Moment)');
    grid on;

    subplot(413);
    plot(time, control_input_array(:,3), col); hold on;
    ylabel('Mc (Pitch Moment)');
    grid on;

    subplot(414);
    plot(time, control_input_array(:,4), col); hold on;
    ylabel('Nc (Yaw Moment)');
    xlabel('Time (s)');
    grid on;

    %% Figure 6: 3D path of aircraft  

    % Plot 3D path of the aircraft
    figure(fig(6));
    plot3(aircraft_state_array(1,:), aircraft_state_array(2,:), aircraft_state_array(3,:), col);
    hold on;
    scatter3(aircraft_state_array(1,1), aircraft_state_array(2,1), aircraft_state_array(3,1), 'g', 'filled'); % Start point
    scatter3(aircraft_state_array(1,end), aircraft_state_array(2,end), aircraft_state_array(3,end), 'r', 'filled'); % End point
    xlabel('X Position');
    ylabel('Y Position');
    zlabel('Z Position');
    title('3D Path of the Aircraft');
    grid on;
    view(3);

end

    
