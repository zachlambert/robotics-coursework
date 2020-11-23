function waypoints = get_leg_waypoints(start_config, mid_config, goal_config, N, T)
    % Returns a trajectory of leg waypoints, where each element
    % specifies time and position
    % Max time is given as T. Can be normalised time.
    
    % Fit waypoints to cubic function:
    % x = k4*t^4 + k3*t^3 + k2*T_2 + k1*T + k0
    
    waypoints = zeros(N, 4);
    t = linspace(0, T, N);
    waypoints(:, 4) = t;
    
    % Copied from get_leg_waypoints_symb
    for i=1:3
        x0 = start_config(i);
        xMid = mid_config(i);
        xT = goal_config(i);
        k0 = x0;
        k1 = 0;
        k2 = -(11*x0 - 16*xMid + 5*xT)/T^2;
        k3 = (2*(9*x0 - 16*xMid + 7*xT))/T^3;
        k4 = -(8*(x0 - 2*xMid + xT))/T^4;
        waypoints(:,i) = k4*t.^4 + k3*t.^3 + k2*t.^2 + k1*t + k0;
    end
end