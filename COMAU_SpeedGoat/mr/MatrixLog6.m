function se3mat = MatrixLog6(T)
% MatrixLog6: Computes the matrix logarithm of a homogeneous transformation T in SE(3)
% and returns the corresponding se(3) matrix.
% This single-precision version assumes that T is a 4x4 single precision matrix.
%
% Example:
%   T = single([1, 0, 0, 0;
%               0, 0, -1, 3;
%               0, 1, 0, 2;
%               0, 0, 0, 1]);
%   se3mat = MatrixLog6(T)

    % Ensure T is single precision.
    T = single(T);
    R = T(1:3, 1:3);
    p = T(1:3, 4);
    
    % Compute the log of the rotational part.
    so3mat = MatrixLog3(R);
    epsilon = single(1e-6);
    
    if norm(so3mat, 'fro') < epsilon
        % When rotation is negligible, return pure translation.
        se3mat = [zeros(3, 'like', T), p; zeros(1, 3, 'like', T), single(0)];
    else
        % Compute theta from the rotation matrix.
        theta = acos((trace(R) - single(1)) / single(2));
        % Define omgmat as the so(3) matrix from the logarithm.
        omgmat = so3mat;  % Now omgmat is single precision.
        
        % Create a 3x3 identity matrix matching T.
        I3 = eye(3, 'like', T);
        % Compute the inverse of the matrix G that appears in the log formula.
        % Here we use the formula: G_inv = I - 0.5*omgmat + (1/theta - 1/(2*tan(theta/2)))*(omgmat^2)
        factor = (1/theta) - (1/(2 * tan(theta/2)));
        G_inv = I3 - 0.5 * omgmat + factor * (omgmat * omgmat);
        v = G_inv * p;
        se3mat = [so3mat, v; zeros(1, 3, 'like', T), single(0)];
    end
end
