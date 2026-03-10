% Contributors: Wiro Martin Gasau
% Course number: ASEN 3801
% File name: QuadrotorEOM_Linearized.m
% Created: 3/03/26

function var_dot = QuadrotorEOM_Linearized(t, var, g, m, I, d, km, nu, mu, motor_forces)
% Inputs: t - time column vector (not used in this function)
%         var - 12x1 aircraft state vector
%         g - gravitational acceleration
%         m - mass of the quadrotor
%         I - 3x3 inertia matrix of the quadrotor
%         deltaFc - 3x1 vector of force deviations from the steady hover trim condition
%         deltaGc - 3x1 vector of moment deviations from the steady hover trim condition
% Output: var_dot - 12x1 vector of state derivatives
% Methodology: The function computes the linearized equations of motion for a quadrotor and packs the resulting state derivatives into a column vector.
    
    % Unpack the state vector
    pos = var(1:3);        % Position (x, y, z)
    euler = var(4:6);        % Velocity (vx, vy, vz)
    vel = var(7:9);      % Euler angles (phi, theta, psi)
    omega = var(10:12);    % Angular velocity (p, q, r)
    p = omega(1);
    q = omega(2); 
    r = omega(3);
    u = vel(1);
    v = vel(2);
    w = vel(3);
    phi = euler(1);
    theta = euler(2);
    psi = euler(3);

    Ix = I(1,1);
    Iy = I(2,2);
    Iz = I(3,3);

    Q_E = [cos(theta)*cos(psi), sin(phi)*sin(theta)*cos(psi) - cos(phi)*sin(psi), cos(phi)*sin(theta)*cos(psi) + sin(phi)*sin(psi);
           cos(theta)*sin(psi), sin(phi)*sin(theta)*sin(psi) + cos(phi)*cos(psi), cos(phi)*sin(theta)*sin(psi) - sin(phi)*cos(psi);
           -sin(theta), sin(phi)*cos(theta), cos(phi)*cos(theta)];
    
    x_dot = Q_E*vel;

    T_euler = [1, sin(phi)*tan(theta), cos(phi)*tan(theta);
               0, cos(phi), -sin(phi);
               0, sin(phi)*sec(theta), cos(phi)*sec(theta)];
    
    euler_dot = T_euler*omega;

    g_vec = g*[-sin(theta);
               cos(theta)*sin(phi);
               cos(theta)*cos(phi)];
    rot_v_vec = [r*v - q*w;
                 p*w - r*u;
                 q*u - p*v];
    control_matrix = [-1 -1 -1 -1;
                      -d/sqrt(2) -d/sqrt(2) d/sqrt(2) d/sqrt(2);
                      d/sqrt(2) -d/sqrt(2) -d/sqrt(2) d/sqrt(2)
                      km -km km -km];
    deltaFc = control_matrix*motor_forces(:);
    Z_c_vec = [0; 0; deltaFc(1)/m];
    LMN = deltaFc(2:4);
    
    vel_dot = rot_v_vec + g_vec + Z_c_vec;

    inertia_rot_moments = [(Iy - Iz)/Ix*q*r;
                          (Iz - Ix)/Iy*p*r;
                          (Ix - Iy)/Iz*p*q];

    Control_moments = [LMN(1)/Ix;
                       LMN(2)/Iy;
                       LMN(3)/Iz];

    
    omega_dot = inertia_rot_moments + Control_moments;
    % Pack the state derivative vector
    var_dot = [x_dot;euler_dot; vel_dot; omega_dot];
end