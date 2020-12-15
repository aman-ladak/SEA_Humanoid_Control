function [state_dot] = dynamics(params, state, F, T, y0)
% Input parameters
% 
%   state: current state
% 
%   params: Quadcopter parameters
%
% Output parameters
%
%   state_dot: change in state
%
%************  DYNAMICS ************************

N = params.gear_ratio;
Jm = params.motor_inertia;
k = params.stiffness;
r = params.radius;
m = params.mass;
g = params.gravity;

theta_dotdot = (T/Jm) - ((k*r)/(N*Jm))*(y0 - state(1,1) + (r/N)*state(3,1));

y_dotdot = (F - m*g)/m;


state_dot(1,1) = state(2,1);
state_dot(2,1) = y_dotdot;
state_dot(3,1) = state(4,1);
state_dot(4,1) = theta_dotdot;

    
end