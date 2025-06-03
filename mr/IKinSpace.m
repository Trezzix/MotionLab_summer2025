function [thetalist, success] = IKinSpace(Slist, M, T, thetalist0, eomg, ev)
% IKinSpace: Iterative Newton-Raphson inverse kinematics in the space frame.
% All inputs are expected to be single precision.
%
% Slist      : 6xn matrix of joint screw axes (in single precision)
% M          : 4x4 home configuration of the end-effector (single)
% T          : Desired end-effector configuration (4x4 single)
% thetalist0 : Initial guess for the joint angles (single nx1)
% eomg, ev   : Tolerances for orientation and position error (single)
%
% Returns:
% thetalist  : Joint angles (single nx1)
% success    : Logical flag (true if solution found)

    thetalist = thetalist0;
    i = 0;
    maxiterations = 20;
    
    Tsb = FKinSpace(M, Slist, thetalist);
    Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
    err = (norm(Vs(1:3)) > eomg) || (norm(Vs(4:6)) > ev);
    
    while err && (i < maxiterations)
        % Update joint angles using pseudoinverse of the Jacobian
        thetalist = thetalist + pinv(JacobianSpace(Slist, thetalist)) * Vs;
        i = i + 1;
        Tsb = FKinSpace(M, Slist, thetalist);
        Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
        err = (norm(Vs(1:3)) > eomg) || (norm(Vs(4:6)) > ev);
    end
    
    success = ~err;
end
