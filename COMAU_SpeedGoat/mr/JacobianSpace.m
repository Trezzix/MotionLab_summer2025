function Js = JacobianSpace(Slist, thetalist)
% JacobianSpace: Computes the space Jacobian given joint screw axes and angles.
% All inputs are expected to be single precision.
%
% Slist     : 6xn matrix of joint screw axes (single)
% thetalist : nx1 vector of joint angles (single)
%
% Returns:
% Js        : 6xn space Jacobian (single)

    Js = Slist;
    % Create a 4x4 identity matrix in single precision using 'like'
    T = eye(4, 'like', Slist);
    for i = 2:length(thetalist)
        T = T * MatrixExp6(VecTose3(Slist(:, i - 1) * thetalist(i - 1)));
        Js(:, i) = Adjoint(T) * Slist(:, i);
    end
end
