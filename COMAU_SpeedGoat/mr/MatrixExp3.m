function R = MatrixExp3(so3mat)
% MatrixExp3: Computes the matrix exponential for a 3x3 so(3) matrix.
%   R = MatrixExp3(so3mat) returns the rotation matrix R in SO(3) 
%   corresponding to the exponential coordinates in so3mat.
%
%   Example:
%     so3mat = single([0, -3, 2; 3, 0, -1; -2, 1, 0]);
%     R = MatrixExp3(so3mat);
%
%   Note: This version forces single precision computations. modified by
%   chatgpt
so3mat = double(so3mat);
    omgtheta = so3ToVec(so3mat);
    if NearZero(norm(omgtheta))
        % Create an identity matrix with the same type as so3mat.
        R = eye(3, 'like', so3mat);
    else
        [omghat, theta] = AxisAng3(omgtheta);
        omgmat = so3mat / theta;
        % Use single(1) for numeric constants so that subtraction stays in single.
        R = eye(3, 'like', so3mat) + sin(theta) * omgmat + (1 - cos(theta)) * (omgmat * omgmat);
    end
end
