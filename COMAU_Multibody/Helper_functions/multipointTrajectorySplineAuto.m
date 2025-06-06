function [pos, vel, acc] = multipointTrajectorySplineAuto(t, t_points, pos_points, method, varargin)
% multipointTrajectorySplineAuto Evaluates a multi-segment trajectory at time t,
% automatically computing intermediate velocities and accelerations if not provided.
%
%   [pos, vel, acc] = multipointTrajectorySplineAuto(t, t_points, pos_points, method)
%   [pos, vel, acc] = multipointTrajectorySplineAuto(t, t_points, pos_points, method, V_points)
%   [pos, vel, acc] = multipointTrajectorySplineAuto(t, t_points, pos_points, method, V_points, A_points)
%
%   Inputs:
%       t         - Current time (scalar).
%       t_points  - Vector of time stamps for each waypoint (must be strictly increasing).
%       pos_points- Vector of positions corresponding to each waypoint.
%       method    - String: 'cubic' or 'quintic' (determines the interpolation method).
%
%       V_points  - (Optional) Vector of velocities at each waypoint. If not provided,
%                   the function will compute them using finite differences.
%
%       A_points  - (Optional, for quintic) Vector of accelerations at each waypoint.
%                   If not provided (or empty), they are computed automatically.
%
%   Outputs:
%       pos - Position at time t.
%       vel - Velocity at time t.
%       acc - Acceleration at time t.
%
%   Example:
%       % Define waypoints:
%       t_points   = [0, 2, 4, 6];
%       pos_points = [0, 5, 3, 10];
%
%       % Using cubic interpolation with automatic velocity computation:
%       t_query = 3.0;
%       [pos, vel, acc] = multipointTrajectorySplineAuto(t_query, t_points, pos_points, 'cubic');
%
%       % Using quintic interpolation with automatic velocity and acceleration computation:
%       [pos, vel, acc] = multipointTrajectorySplineAuto(t_query, t_points, pos_points, 'quintic');
%
%   Note: This function calls the functions cubicTrajectoryPlanner.m and
%         quinticTrajectoryPlanner.m, so ensure those files are on your MATLAB path.

    % Validate method input
    if ~(ischar(method) || isstring(method))
        error('The "method" input must be a string: ''cubic'' or ''quintic''.');
    end
    method = lower(method);

    nPoints = length(t_points);
    if length(pos_points) ~= nPoints
        error('t_points and pos_points must have the same length.');
    end

    %% Automatically compute derivatives if not provided
    switch method
        case 'cubic'
            % If velocities are not provided, compute them using finite differences.
            if nargin < 5 || isempty(varargin{1})
                V_points = zeros(size(t_points));
                for i = 1:nPoints
                    if i == 1
                        % Forward difference for the first point
                        V_points(i) = 0;
                    elseif i == nPoints
                        % Backward difference for the last point
                        V_points(i) = 0;
                    else
                        % Central difference for interior points
                        V_points(i) = (pos_points(i+1)-pos_points(i-1))/(t_points(i+1)-t_points(i-1));
                    end
                end
            else
                V_points = varargin{1};
                if length(V_points) ~= nPoints
                    error('V_points must have the same length as t_points.');
                end
            end

        case 'quintic'
            % For quintic, we need both velocities and accelerations.
            if nargin < 6 || isempty(varargin{1}) || isempty(varargin{2})
                V_points = zeros(size(t_points));
                A_points = zeros(size(t_points));
                for i = 1:nPoints
                    if i == 1
                        % For the first point: use a forward difference for velocity and assume zero acceleration.
                        V_points(i) = 0;
                        A_points(i) = 0;
                    elseif i == nPoints
                        % For the last point: use a backward difference for velocity and assume zero acceleration.
                        V_points(i) = 0;
                        A_points(i) = 0;
                    else
                        % Central differences for interior points:
                        V_points(i) = (pos_points(i+1)-pos_points(i-1))/(t_points(i+1)-t_points(i-1));
                        % A simple second-order finite difference for acceleration:
                        A_points(i) = 2 * ( (pos_points(i+1)-pos_points(i))/(t_points(i+1)-t_points(i)) - ...
                                             (pos_points(i)-pos_points(i-1))/(t_points(i)-t_points(i-1)) ) ...
                                      / (t_points(i+1)-t_points(i-1));
                    end
                end
            else
                V_points = varargin{1};
                A_points = varargin{2};
                if length(V_points) ~= nPoints || length(A_points) ~= nPoints
                    error('V_points and A_points must have the same length as t_points.');
                end
            end

        otherwise
            error('Unsupported method. Use ''cubic'' or ''quintic''.');
    end

    %% Determine which segment the current time falls into
    % Clamp t to the time bounds
    if t <= t_points(1)
        t = t_points(1);
        segment = 1;
    elseif t >= t_points(end)
        t = t_points(end);
        segment = nPoints - 1;
    else
        % Find the index such that t_points(i) <= t < t_points(i+1)
        segment = find(t_points <= t, 1, 'last');
        if segment == nPoints
            segment = nPoints - 1;
        end
    end

    % Compute the relative time in the current segment
    T_start = t_points(segment);
    T_end   = t_points(segment+1);
    T_seg   = T_end - T_start;
    t_rel   = t - T_start;

    %% Evaluate the current segment using the selected method
    switch method
        case 'cubic'
            p0 = pos_points(segment);
            pf = pos_points(segment+1);
            v0 = V_points(segment);
            vf = V_points(segment+1);
            [pos, vel, acc] = cubicTrajectoryPlanner(t_rel, T_seg, p0, pf, v0, vf);
        case 'quintic'
            P0 = pos_points(segment);
            Pf = pos_points(segment+1);
            V0 = V_points(segment);
            Vf = V_points(segment+1);
            A0 = A_points(segment);
            Af = A_points(segment+1);
            [pos, vel, acc] = quinticTrajectoryPlanner(t_rel, T_seg, P0, Pf, V0, Vf, A0, Af);
    end

end
