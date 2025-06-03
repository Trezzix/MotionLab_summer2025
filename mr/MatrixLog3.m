function so3mat = MatrixLog3(R)
% MatrixLog3: Computes the matrix logarithm of a rotation matrix R in SO(3)
% This single-precision version assumes that R is a 3x3 single precision matrix.
%
% Example:
%   R = single([ -0.6949  0.7135  0.0893;
%                -0.1920 -0.3038  0.9332;
%                 0.6930  0.6313  0.3481 ]);
%   so3mat = MatrixLog3(R)

    % Ensure input is single precision.
    R = single(R);
    epsilon = single(1e-6);
    
    % If R is (nearly) the identity, the log is zero.
    if abs(trace(R) - 3) < epsilon
        so3mat = zeros(3, 'like', R);
    else
        % Compute the rotation angle.
        theta = acos((trace(R) - single(1)) / single(2));
        % If sin(theta) is very small, avoid dividing by zero.
        if abs(sin(theta)) < epsilon
            so3mat = zeros(3, 'like', R);
        else
            so3mat = theta/(2 * sin(theta)) * (R - R');
        end
    end
end
