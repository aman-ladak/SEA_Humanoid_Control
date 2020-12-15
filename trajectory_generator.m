function [trajectory, time] = trajectory_generator(dt, ydes, dir)
%trajectory = generated trajectory for state
%time = time surpassed
%dt = time step
%ydes = goal y position
%dir = trajectory tracking type (dir=1 -> quadratic; dir=2 -> impulse)

yinit = 1; %initial y position


if dir==1
%     h = abs(ydes - yinit); %change in y
%     r = 20; %design choice: achieve 1m per 20sec in y direction (20sec/1m)
%     time = h*r;
    time_1 = 6;
    waypoint_times_1 = 0:dt:time_1;

    if mod((length(waypoint_times_1)-1),2) == 0
        t_crit = (length(waypoint_times_1)-1)*0.5;
    else
        t_crit = ((length(waypoint_times_1)-1)*0.5) -0.5;
    end

    for i=1:t_crit
        A = [(-time_1/2)^2 (-time_1/2) 1; 0 0 1; (time_1/2)^2 (time_1/2) 1];
        B = [(ydes+yinit)/2; yinit; (ydes+yinit)/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        c = cof(3,1);
        t = waypoint_times_1(i);
        waypoints(:,i) = a*(t^2) + b*t + c;
        waypoints(:,i) = round(waypoints(:,i),10);
        vy1(:,i) = 2*a*t + b;
    end
    for j = (t_crit + 1):(length(waypoint_times_1))
        A = [(time_1/2)^2 (time_1/2) 1; (time_1)^2 time_1 1; (3*time_1/2)^2 (3*time_1/2) 1];
        B = [(ydes+yinit)/2; ydes; (ydes+yinit)/2];
        cof = A\B;
        a = cof(1,1);
        b = cof(2,1);
        c = cof(3,1);
        t = waypoint_times_1(j);
        waypoints(:,j) = a*(t^2) + b*t + c;
        waypoints(:,j) = round(waypoints(:,j),10);
        vy1(:,j) = 2*a*t + b;
    end
    t_step_1 = (time_1/dt) + 1;
    t_step_1 = round(t_step_1);
    trajectory_1 = trajectory_planner(1, waypoints, t_step_1, waypoint_times_1, dt, vy1);
    % final_waypoint = waypoints(:,length(waypoints));
    
    time_2 = 4;
    waypoint_times_2 = [time_2];
    waypoints_2(:,1) = ydes;
    vy2 = 0;
    t_step_2 = (time_2/dt) + 1;
    t_step_2 = round(t_step_2);
    trajectory_2 = trajectory_planner(2, waypoints_2, t_step_2, waypoint_times_2, dt, vy2);
    
    trajectory = horzcat(trajectory_1, trajectory_2);
    time = time_1+time_2;

end

if dir==2
%     h = abs(ydes - yinit); %change in y
%     r = 20; %design choice: achieve 1m per 20sec in y direction (20sec/1m)
%     time = h*r;
    time = 10;
    waypoint_times = [time];
    waypoints(:,1) = ydes;
    vy = 0;
    t_step = (time/dt) + 1;
    t_step = round(t_step);
    trajectory = trajectory_planner(2, waypoints, t_step, waypoint_times, dt, vy);
end
end
    