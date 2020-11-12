function waypoints = leg_waypoints(start_config, goal_config, N)
    % TODO: Use some trapozoidal trajectory
    % For now, just making it linear for simplicity
    waypoints = linspace(start_config, goal_config, N);
end