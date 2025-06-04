function [pos, vel, acc] = cubicTrajectoryPlanner(t, T, p0, pf, v0, vf)
%#codegen
% Clamp time to [0, T]
if t < 0
    t = 0;
elseif t > T
    t = T;
end


a0 = p0;
a1 = v0;
a2 = (3*(pf - p0) / T^2) - ((2*v0 + vf) / T);
a3 = (-2*(pf - p0) / T^3) + ((v0 + vf) / T^2);


pos = a0 + a1*t + a2*t^2 + a3*t^3;
vel = a1 + 2*a2*t + 3*a3*t^2;
acc = 2*a2 + 6*a3*t;
end