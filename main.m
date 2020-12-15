function [] = main(ydes, dir)

warning off;

params = struct(...
    'mass',                   80, ... %kg
    'gravity',                9.81, ... %m/s^2
    'stiffness',              (20*10^3), ... %N/m
    'radius',                 0.05, ... %m
    'gear_ratio',             40, ...
    'motor_inertia',          0.000506); %kg*m^2

dt = 0.005;
y0 = 1;

[trajectory, time] = trajectory_generator(dt, ydes, dir);

%Sim Params
time_initial = 0;
time_vec = time_initial:dt:time;
max_iter = length(time_vec);

state = zeros(4,1);
state(1,1)=y0;

actual_state_matrix = zeros(4, max_iter);
actual_state_matrix(:,1) = state;

actual_desired_state_matrix = zeros(6, max_iter);
actual_desired_state_matrix(1,1) = y0;

current_state.y = state(1);

F_tot = zeros(1,max_iter);
T_tot = zeros(1,max_iter);
% F_actual = zeros(1,max_iter);
% T_actual = zeros(1,max_iter);


for iter = 1:max_iter-1
    %Convert current state to stuct for control functions
    current_state.y = state(1);
    current_state.ydot = state(2);
    current_state.theta = state(3);
    current_state.thetadot = state(4);

    desired_state.y = trajectory(1,iter);
    desired_state.ydot = trajectory(2,iter);
    desired_state.ydotdot = trajectory(3,iter);
    desired_state.theta = trajectory(4,iter);
    desired_state.thetadot = trajectory(5,iter);
    desired_state.thetadotdot = trajectory(6,iter);
    
    % Get desired force from behaviour controller
    [F, desired_state.ydotdot, desired_state.theta] = behaviour_controller(current_state, desired_state, params,y0);
    F_tot(1,iter) = F;
    
    % Get desired torque from sea controller
    [T, desired_state.thetadotdot] = sea_controller(current_state, desired_state, params,y0);
    
    %Limit max torque based on motor maximum continuous torque (Tmax)
    if T > 1.3633019
        T = 1.3633019;
    end
    if T < -1.3633019
        T = -1.3633019;
    end
    T_tot(1,iter) = T;


    % Get the change in state
    timeint = time_vec(iter:iter+1);
    tmp = dynamics(params, state, F, T,y0);
    [tsave, xsave] = ode45(@(t,s) dynamics(params, s, F, T,y0), timeint, state);
    state    = xsave(end, :)';
    lin_acc  = (xsave(end,2)' - xsave(end-1,2)')/(tsave(end) - tsave(end-1));
    ang_acc  = (xsave(end,4)' - xsave(end-1,4)')/(tsave(end) - tsave(end-1));
        
    % Update desired state matrix
    actual_desired_state_matrix(1,iter+1) =  desired_state.y;
    actual_desired_state_matrix(2, iter+1) = desired_state.ydot;
    actual_desired_state_matrix(3, iter+1) = desired_state.ydotdot;
    actual_desired_state_matrix(4, iter+1) = desired_state.theta;
    actual_desired_state_matrix(5, iter+1) = desired_state.thetadot;
    actual_desired_state_matrix(6, iter+1) = desired_state.thetadotdot;

    % Update actual state matrix
    actual_state_matrix(1:4, iter+1) = state(1:4);
    actual_state_matrix(5, iter+1) = lin_acc;
    actual_state_matrix(6, iter+1) = ang_acc;
end

plotter(actual_state_matrix, actual_desired_state_matrix, T_tot, time_vec)

end
    
