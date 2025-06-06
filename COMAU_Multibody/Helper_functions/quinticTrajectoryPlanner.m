function [pos, vel, acc] = quinticTrajectoryPlanner(t, T, P0, Pf, V0, Vf, A0, Af)
%QUINTICTRAJECTORYPLANNER Computes position, velocity, and acceleration at time t
%   using a quintic polynomial trajectory.
%
%   [pos, vel, acc] = quinticTrajectoryPlanner(t, T, P0, Pf, V0, Vf, A0, Af)
%
%   Inputs:
%       t  - Current time (seconds). It is expected that 0 <= t <= T.
%       T  - Total duration of the trajectory (seconds)
%       P0 - Initial position at t = 0
%       Pf - Final position at t = T
%       V0 - Initial velocity at t = 0
%       Vf - Final velocity at t = T
%       A0 - Initial acceleration at t = 0
%       Af - Final acceleration at t = T
%
%   Outputs:
%       pos - Position at time t
%       vel - Velocity at time t
%       acc - Acceleration at time t
%
%   Example:
%       [pos, vel, acc] = quinticTrajectoryPlanner(2.5, 5, 0, 10, 0, 0, 0, 0);

    % Clamp time to [0, T]
    if t < 0
        t = 0;
    elseif t > T
        t = T;
    end

    % Set up the coefficients that come directly from the boundary conditions
    c0 = P0;
    c1 = V0;
    c2 = A0 / 2;

    % Build the matrix for t=T for c3, c4, c5
    Tmat = [T^3,    T^4,     T^5;
            3*T^2,  4*T^3,   5*T^4;
            6*T,   12*T^2,  20*T^3];
    b = [Pf - (c0 + c1*T + c2*T^2);
         Vf - (c1 + 2*c2*T);
         Af - (2*c2)];

    % Solve for the unknown coefficients [c3; c4; c5]
    coeff = Tmat \ b;
    c3 = coeff(1);
    c4 = coeff(2);
    c5 = coeff(3);

    % Evaluate the polynomial and its derivatives at time t
    pos = c0 + c1*t + c2*t^2 + c3*t^3 + c4*t^4 + c5*t^5;
    vel = c1 + 2*c2*t + 3*c3*t^2 + 4*c4*t^3 + 5*c5*t^4;
    acc = 2*c2 + 6*c3*t + 12*c4*t^2 + 20*c5*t^3;
end
