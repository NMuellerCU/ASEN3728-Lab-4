function torque = computeTorque(motor_forces, d, km)
d_q = d/sqrt(2);

quadrotor_mat = [-1 -1 -1 -1;
          -d_q, -d_q, d_q, d_q;
          d_q, -d_q, -d_q, d_q;
          km, -km, km, -km];
torque = quadrotor_mat*motor_forces;
end