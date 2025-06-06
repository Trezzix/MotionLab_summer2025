function [p, vDot, wDot] = RTCartesianTrajectory(t, Xstart, Xend, Tf)

%Work in progress

% RealTimeCartesianTrajectory - Computes the instantaneous end-effector
% configuration along a Cartesian trajectory as well as its translational
% and rotational velocities.
%This function is an extension of the CartesianTrajectory function from the
%modern robotics software suite
%
% Inputs:
%   t      : Current simulation time (seconds). Typically provided by a Clock.
%   Xstart : The initial end-effector configuration (4x4 homogeneous matrix).
%   Xend   : The final end-effector configuration (4x4 homogeneous matrix).
%   Tf     : Total time for the motion (seconds).
%   method : Time-scaling method indicator:
%              3 for cubic, 5 for quintic.
%
% Outputs:
%   X      : The current end-effector configuration (4x4 homogeneous matrix).
%   v      : The translational velocity (3x1 vector).
%   w      : The rotational (angular) velocity (3x1 vector).
%
% Dependencies:
%   TransToRp, MatrixExp3, MatrixLog3, vee, CubicTimeScaling, QuinticTimeScaling,
%   CubicTimeScalingDerivative, QuinticTimeScalingDerivative.
%
% Note: Make sure that the helper functions are available on the MATLAB path.
%
% Clamp time so that t does not exceed Tf.

%Define qubic (3) or quintic (5) time scaling
method = 5;

if t >= Tf
    t = Tf;
end

% Extract rotation and position components from the start and end configurations.
[Rstart, pstart] = TransToRp(Xstart);
[Rend,   pend]   = TransToRp(Xend);

% Compute the time scaling s(t) and its derivative ds/dt.
if method == 3
    s     = CubicTimeScaling(Tf, t);
    s_dot = CubicTimeScalingDerivative(Tf, t);
elseif method == 5
    s     = QuinticTimeScaling(Tf, t);
    s_dot = QuinticTimeScalingDerivative(Tf, t);
else
    % Default to quintic scaling if method is unrecognized.
    s     = QuinticTimeScaling(Tf, t);
    s_dot = QuinticTimeScalingDerivative(Tf, t);
end

% Compute the rotation
% omega_hat is the matrix logarithm of Rstart' * Rend.
omega_hat = MatrixLog3(Rstart' * Rend);
% The rotational interpolation:
R_current = Rstart * MatrixExp3(omega_hat * s);

% Compute the translational interpolation.
p_current = pstart + s * (pend - pstart);
p = p_current;

% Assemble the homogeneous transformation.
%X = [R_current, p_current;
%     0, 0, 0, 1];

% Compute translational velocity:
vDot = s_dot * (pend - pstart);

% Compute rotational (angular) velocity: 
% Convert omega_hat to a vector using the vee operator.
omega = vee(omega_hat);
wDot = s_dot * omega;
end


function v = vee(so3mat)
%Converts a 3×3 skew-symmetric matrix to a 3×1 vector.
    v = [so3mat(3,2); so3mat(1,3); so3mat(2,1)];
end