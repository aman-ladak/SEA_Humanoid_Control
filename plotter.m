function [] = plotter(state, state_des, torque, time_vector)

%% Get individual plots from state

y = state(1,:);
ydot = state(2,:);
theta = state(3,:);
thetadot = state(4,:);
ydotdot = state(5,:);
thetadotdot = state(6,:);

y_des = state_des(1,:);
ydot_des = state_des(2,:);
ydotdot_des = state_des(3,:);
theta_des = state_des(4,:);
thetadot_des = state_des(5,:);
thetadotdot_des = state_des(6,:);

%% Get error from desired and actual

error_y = y-y_des;
error_ydot = ydot-ydot_des;
error_theta = theta-theta_des;
error_thetadot = thetadot-thetadot_des;
error_linacc = ydotdot - ydotdot_des;
error_angacc = thetadotdot - thetadotdot_des;


%% Plot error

% % Plot y error
% labels = {'y [m]'};
% title_name = {'Error in y'};
% str = 'Plot of Y Error';
% figure('Name',str);
% 
% subplot(2, 2, 1)
% plot(time_vector,error_y(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% 
% 
% % Plot ydot error
% 
% labels = {'\ydot'};
% title_name = {'Error in \ydot'};
% 
% subplot(2, 2, 2)
% plot(time_vector,error_ydot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% 
% 
% % Plot theta error
% 
% labels = {'theta'};
% title_name = {'Error in Theta'};
% 
% subplot(2, 2, 3)
% plot(time_vector,error_theta(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% 
% 
% % Plot thetadot error
% 
% labels = {'\theta_dot [rad/s]'};
% title_name = {'Error in \theta_dot'};
% 
% subplot(2, 2, 4)
% plot(time_vector,error_thetadot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% 
% 
% % Plot linacc error
% 
% labels = {'\linacc [m/s^2]'};
% title_name = {'Error in \linacc'};
% 
% subplot(3, 2, 5)
% plot(time_vector,error_linacc(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% 
% % Plot angacc error
% 
% labels = {'\angacc [rad/s^2]'};
% title_name = {'Error in \angacc'};
% 
% subplot(3, 2, 6)
% plot(time_vector,error_angacc(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})


%% Plot desired and actual

% Plot y
labels = {'y [m]'};
title_name = {'Plot of y'};

str = 'Position';
figure('Name',str);

subplot(1, 2, 1)
plot(time_vector, y_des(1,:),'b',time_vector, y(1,:),'r');
grid on
xlabel('time [s]')
ylabel(labels{1})
title(title_name{1})
legend('Desired','Actual');

% Plot Tm
labels = {'Tm [Nm]'};
title_name = {'Plot of Motor Torque'};

% str = 'Torque';
% figure('Name',str);

subplot(1, 2, 2)
plot(time_vector, torque(1,:),'r');
grid on
xlabel('time [s]')
ylabel(labels{1})
title(title_name{1})


% % Plot ydot
% labels = {'ydot [m/s]'};
% title_name = {'Plot of ydot'};
% 
% subplot(2, 2, 2)
% plot(time_vector, ydot_des(1,:),'b',time_vector, ydot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% legend('Desired','Actual');
% 
% 
% % Plot theta
% labels = {'theta [rad]'};
% title_name = {'Plot of theta'};
% 
% subplot(2, 2, 3)
% plot(time_vector, theta_des(1,:),'b',time_vector, theta(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% legend('Desired','Actual');
% 
% 
% % Plot thetadot
% labels = {'thetadot [rad/s]'};
% title_name = {'Plot of thetadot'};
% 
% subplot(2, 2, 4)
% plot(time_vector, thetadot_des(1,:),'b',time_vector, thetadot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% legend('Desired','Actual');


% % Plot ydotdot
% labels = {'ydotdot [m/s^2]'};
% title_name = {'Plot of ydotdot'};
% 
% subplot(3, 2, 5)
% plot(time_vector, ydotdot_des(1,:),'b',time_vector, ydotdot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% legend('Desired','Actual');
% 
% 
% % Plot thetadotdot
% labels = {'thetadotdot [rad/s^2]'};
% title_name = {'Plot of thetadotdot'};
% 
% subplot(3, 2, 6)
% plot(time_vector, thetadotdot_des(1,:),'b',time_vector, thetadotdot(1,:),'r');
% grid on
% xlabel('time [s]')
% ylabel(labels{1})
% title(title_name{1})
% legend('Desired','Actual');

end

