function [Tm_des, motor_acc] = sea_controller(current_state, desired_state, params, y0)

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
%   Tm_des: Desired Motor Torque
%   motor_acc = Desired motor acceleration
%
%************  SEA CONTROLLER ************************

% PD gains
% Kp = 88; 
Kp = 80;
Kd = 20;


e_theta = current_state.theta - desired_state.theta;
edot_theta = current_state.thetadot - desired_state.thetadot;

edotdot_theta = -Kp*e_theta - Kd*edot_theta;

motor_acc = desired_state.thetadotdot + edotdot_theta;

N = params.gear_ratio;
Jm = params.motor_inertia;
k = params.stiffness;
r = params.radius;

Tm_des = Jm*motor_acc + (k*r/N)*(y0 - desired_state.y + (r/N)*desired_state.theta);

% if Tm_des > 1.3633019
%     Tm_des = 1.3633019;
% end
% if Tm_des < -1.3633019
%     Tm_des = -1.3633019;
% end

end
