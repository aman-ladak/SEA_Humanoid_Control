function trajectory_state = trajectory_planner(dir, waypoints, max_iter, waypoint_times, dt, vy)
% Output parameters
%
%   trajectory_sate: [6 x max_iter] output trajectory as a matrix of states:
%   [ydes; ydes_dot; ydes_dotdot; thetades; thetades_dot; thetades_dotdot];

trajectory_state = zeros(6,max_iter);
current_waypoint_number = 1;

if dir ==1
    for iter = 1:max_iter
    if (current_waypoint_number<length(waypoint_times))
        if((iter*dt)>(waypoint_times(current_waypoint_number+1) - waypoint_times(1)))
            current_waypoint_number = current_waypoint_number + 1;
        end
    end
    trajectory_state(1,iter) = waypoints(1,current_waypoint_number); %ydes
    trajectory_state(2,iter) = vy(1,current_waypoint_number); %ydes_dot
    end
end

if dir ==2
    for iter = 1:max_iter
    if (current_waypoint_number<length(waypoint_times))
        if((iter*time_step)>(waypoint_times(current_waypoint_number+1) - waypoint_times(1)))
            current_waypoint_number = current_waypoint_number + 1;
        end
    end
    trajectory_state(1,iter) = waypoints(1,current_waypoint_number); %ydes
    end
end

end