function [pos, vel, acc] = cubicTrajectoryPlanner(t, T, p0, pf, v0, vf)
%CUBICTRAJECTORYPLANNER Computes position, velocity, and acceleration at time t
%   using a cubic polynomial trajectory.
%
%   [pos, vel, acc] = cubicTrajectoryPlanner(t, T, p0, pf, v0, vf)
%
%   Inputs:
%       t  - Current time (seconds). It is expected that 0 <= t <= T.
%       T  - Total duration of the trajectory (seconds)
%       p0 - Initial position at t = 0
%       pf - Final position at t = T
%       v0 - Initial velocity at t = 0
%       vf - Final velocity at t = T
%
%   Outputs:
%       pos - Position at time t
%       vel - Velocity at time t
%       acc - Acceleration at time t
%
%   Example:
%       [pos, vel, acc] = cubicTrajectoryPlanner(2.5, 5, 0, 10, 0, 0);

    % Clamp time to [0, T]
    if t < 0
        t = 0;
    elseif t > T
        t = T;
    end

    % Compute the coefficients of the cubic polynomial
    a0 = p0;
    a1 = v0;
    a2 = (3*(pf - p0) / T^2) - ((2*v0 + vf) / T);
    a3 = (-2*(pf - p0) / T^3) + ((v0 + vf) / T^2);

    % Evaluate the polynomial and its derivatives at time t
    pos = a0 + a1*t + a2*t^2 + a3*t^3;
    vel = a1 + 2*a2*t + 3*a3*t^2;
    acc = 2*a2 + 6*a3*t;
end
