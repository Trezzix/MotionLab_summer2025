function T = FKinSpace(M, Slist, thetalist)
% FKinSpace: Computes forward kinematics in the space frame.
% All inputs are expected to be  precision.
%
% M         : 4x4 home configuration of the end-effector ()
% Slist     : 6xn matrix of joint screw axes ()
% thetalist : nx1 vector of joint angles ()
%
% Returns:
% T         : 4x4 end-effector configuration in SE(3) ()

    T = M;  % M is  already.
    % Loop from the last joint to the first.
    for i = size(thetalist,1):-1:1
        T = MatrixExp6(VecTose3(Slist(:, i) * thetalist(i))) * T;
    end
end
