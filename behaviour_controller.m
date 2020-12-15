function [F_des, acc, theta_des] = behaviour_controller(current_state, desired_state, params, y0)

% Input parameters
% 
%   current_state: The current state of the robot with the following fields:
%   current_state.y = y, 
%   current_state.ydot = ydot,
%   current_state.theta = theta, 
%   current_state.thetadot = thetadot;
%
%   desired_state: The desired states are:
%   desired_state.y = y, 
%   desired_state.ydot = ydot,
%   desired_state.ydotdot = ydotdot,
%   desired_state.theta = theta, 
%   desired_state.thetadot = thetadot,
%   desired_state.thetadotdot = thetadotdot;
%
%   params: Robot parameters
%
%   y0 = desired y position
%
% Output parameters
%
%   F: Desired Leg Force
%   acc: Desired linear acceleration
%   theta_des: Desired motor position for use in sea_controller
%
%************  BEHAVIOUR CONTROLLER ************************

% PD gains
Kp = 70;  
Kd = 13;


e_y = current_state.y - desired_state.y; %position error
edot_y = current_state.ydot - desired_state.ydot; %velocity error

edotdot_y = -Kp*e_y - Kd*edot_y;

acc = desired_state.ydotdot + edotdot_y;

F_des = params.mass*(acc) + params.mass*params.gravity;

theta_des = (params.gear_ratio/params.radius)*((F_des/params.stiffness) - y0 + desired_state.y);

end
