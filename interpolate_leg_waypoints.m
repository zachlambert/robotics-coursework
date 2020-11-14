function config_cmd = interpolate_leg_waypoints(waypoints, u)
    % Linear interpolation between closest two waypoints
    % is fine.
    N = waypoints.shape(1);
    u_min = waypoints(1,4);
    u_max = waypoints(N,4);
    if u < u_min
        config_cmd = waypoints(1,1:3);
    elseif u > u_max
        config_cmd = waypoints(-1,1:3);
    else
        % Assume u is linearly spaced
        % For a given waypoint, (xi, ui):
        % command(u) = x for ui < u < u(i+1)
        position = 1 + N*((u-u_min)/(u_max-u_min));
        i_lower = floor(position);
        i_upper = ceil(position);
        cmd_lower = waypoints(i_lower,1:3);
        cmd_upper = waypoints(i_upper,1:3);
        config_cmd = cmd_lower + (cmd_upper-cmd_lower)*(position-i_lower);
    end
end

