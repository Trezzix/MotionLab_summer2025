function s_dot = QuinticTimeScalingDerivative(Tf, t)
    tau = t / Tf;
    s_dot = (30*tau^2 - 60*tau^3 + 30*tau^4) / Tf;
end