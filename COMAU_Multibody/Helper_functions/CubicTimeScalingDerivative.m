function s_dot = CubicTimeScalingDerivative(Tf, t)
    tau = t / Tf;
    s_dot = (6*tau - 6*tau^2) / Tf;
end