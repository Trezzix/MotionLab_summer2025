function [pos, vel, acc, t_points] = multipointTrajectorySplineAuto_rev2(t, pos_points, v_max, method, varargin)
% multipointTrajectorySplineAutoWithVmax Evaluates a multi-segment trajectory at time t,
% automatically computing time stamps based on a given maximum velocity.
%
%   [pos, vel, acc, t_points] = multipointTrajectorySplineAutoWithVmax(t, pos_points, v_max, method)
%   [pos, vel, acc, t_points] = multipointTrajectorySplineAutoWithVmax(t, pos_points, v_max, method, V_points)
%   [pos, vel, acc, t_points] = multipointTrajectorySplineAutoWithVmax(t, pos_points, v_max, method, V_points, A_points)
%
%   Inputs:
%       t         - Current time (scalar).
%       pos_points- Vector of positions corresponding to each waypoint.
%       v_max     - Maximum allowable velocity (scalar). This is used to compute
%                   the time needed to move between each consecutive waypoint.
%       method    - String: 'cubic' or 'quintic' (determines the interpolation method).
%
%       V_points  - (Optional) Vector of velocities at each waypoint. If not provided,
%                   the function will compute them using finite differences.
%
%       A_points  - (Optional, for quintic) Vector of accelerations at each waypoint.
%                   If not provided (or empty), they are computed automatically.
%
%   Outputs:
%       pos      - Position at time t.
%       vel      - Velocity at time t.
%       acc      - Acceleration at time t.
%       t_points - The computed time stamps for each waypoint.
%
%   The time stamps are computed as:
%       t_points(1) = 0;
%       t_points(i+1) = t_points(i) + abs(pos_points(i+1)-pos_points(i)) / v_max;
%
%   This ensures that the time duration for each segment is just enough to move
%   from one waypoint to the next without exceeding v_max. The finite differences
%   are then used to compute intermediate velocities (and accelerations, if needed)
%   to ensure smooth, continuous trajectories.

    % Number of waypoints
    nPoints = length(pos_points);
    
    % Compute time stamps for each segment based on the max velocity
    min_dt = 1e-3;  % Set a minimum dt to avoid division by zero.
    t_points = zeros(1, nPoints);
    for i = 1:nPoints-1
        dt = abs(pos_points(i+1) - pos_points(i)) / v_max;
        % Ensure dt is at least min_dt:
        dt = max(dt, min_dt);
        t_points(i+1) = t_points(i) + dt;
    end


    % Validate method input
    if ~(ischar(method) || isstring(method))
        error('The "method" input must be a string: ''cubic'' or ''quintic''.');
    end
    method = lower(method);

    %% Automatically compute derivatives if not provided
    switch method
        case 'cubic'
            % If velocities are not provided, compute them using finite differences.
            if nargin < 5 || isempty(varargin{1})
                V_points = zeros(size(t_points));
                for i = 1:nPoints
                    if i == 1
                        % Forward difference (or set to 0) for the first point
                        V_points(i) = 0;
                    elseif i == nPoints
                        % Backward difference (or set to 0) for the last point
                        V_points(i) = 0;
                    else
                        % Central difference for interior points
                        V_points(i) = (pos_points(i+1) - pos_points(i-1)) / (t_points(i+1) - t_points(i-1));
                    end
                end
            else
                V_points = varargin{1};
                if length(V_points) ~= nPoints
                    error('V_points must have the same length as pos_points.');
                end
            end

        case 'quintic'
            % For quintic interpolation, we need both velocities and accelerations.
            if nargin < 6 || isempty(varargin{1}) || isempty(varargin{2})
                V_points = zeros(size(t_points));
                A_points = zeros(size(t_points));
                for i = 1:nPoints
                    if i == 1
                        % Use forward difference for the first point and assume zero acceleration.
                        V_points(i) = 0;
                        A_points(i) = 0;
                    elseif i == nPoints
                        % Use backward difference for the last point and assume zero acceleration.
                        V_points(i) = 0;
                        A_points(i) = 0;
                    else
                        % Central differences for interior points:
                        V_points(i) = (pos_points(i+1) - pos_points(i-1)) / (t_points(i+1) - t_points(i-1));
                        % Second-order finite difference for acceleration:
                        A_points(i) = 2 * ( (pos_points(i+1) - pos_points(i)) / (t_points(i+1) - t_points(i)) - ...
                                            (pos_points(i) - pos_points(i-1)) / (t_points(i) - t_points(i-1)) ) ...
                                     / (t_points(i+1) - t_points(i-1));
                    end
                end
            else
                V_points = varargin{1};
                A_points = varargin{2};
                if length(V_points) ~= nPoints || length(A_points) ~= nPoints
                    error('V_points and A_points must have the same length as pos_points.');
                end
            end

        otherwise
            error('Unsupported method. Use ''cubic'' or ''quintic''.');
    end

    %% Determine which segment the current time falls into
    % Clamp t to the bounds defined by t_points
    if t <= t_points(1)
        t = t_points(1);
        segment = 1;
    elseif t >= t_points(end)
        t = t_points(end);
        segment = nPoints - 1;
    else
        % Find the index such that t_points(segment) <= t < t_points(segment+1)
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
