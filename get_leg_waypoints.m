function waypoints = get_leg_waypoints(start_config, goal_config, N, u_max)
    % Returns a trajectory of leg waypoints, where each element
    % specifies time and position
    % Time is normalised and goes from 0 to u_max
    
    % Fit waypoints to cubic function:
    % x = k_3*t^3 + k_2*T_2 + k_1*T + k_0
    % (T = u_max)
    
    waypoints = zeros(N, 4);
    u = linspace(0, u_max, N+1);
    u = u(1:N); % ie: Neglect final waypoint
    waypoints(:, 4) = u;
    
    % Copied from get_leg_waypoints_symb
    for i=1:3
        x0 = start_config(i);
        xT = goal_config(i);
        k0 = x0;
        k1 = 0;
        k2 = -(3*(x0 - xT))/u_max^2;
        k3 = (2*(x0 - xT))/u_max^3;
        waypoints(:,i) = k3*u^3 + k2*u^2 + k1*u + k0;
    end
end