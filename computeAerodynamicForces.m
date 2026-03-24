function [aerodynamic_force, aerodynamic_moment] = computeAerodynamicForces(vel, omega, nu, mu)
    V_a = sqrt(sum(vel.^2));
    aerodynamic_force = -nu*V_a*vel;
    
    Ang_a = sqrt(sum(omega.^2));
    aerodynamic_moment = -mu * Ang_a * omega;
end