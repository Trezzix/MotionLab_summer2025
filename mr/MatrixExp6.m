function T = MatrixExp6(se3mat)
% MatrixExp6: Computes the matrix exponential for a 4x4 se(3) matrix.
%   T = MatrixExp6(se3mat) returns the homogeneous transformation matrix T in SE(3) 
%   corresponding to the exponential coordinates in se3mat.
%
%   Example:
%     se3mat = [0,      0,       0,      0;
%                      0,      0, -1.5708, 2.3562;
%                      0,  1.5708,       0, 2.3562;
%                      0,      0,       0,      0];
%     T = MatrixExp6(se3mat);
%

se3mat = double(se3mat);
    omgtheta = so3ToVec(se3mat(1:3, 1:3));
    if NearZero(norm(omgtheta))
        % Build T so that its parts match the type of se3mat.
        T = [eye(3, 'like', se3mat), se3mat(1:3, 4);
             zeros(1, 3, 'like', se3mat), 1];
    else
        [omghat, theta] = AxisAng3(omgtheta);
        omgmat = se3mat(1:3, 1:3) / theta;
        T = [ MatrixExp3(se3mat(1:3, 1:3)), ...
              (eye(3, 'like', se3mat) * theta + (1 - cos(theta)) * omgmat + (theta - sin(theta)) * (omgmat * omgmat)) * se3mat(1:3, 4) / theta;
              zeros(1, 3, 'like', se3mat), 1 ];
    end
end
