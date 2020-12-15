# SEA_Humanoid_Control
PD Control of Humanoid Actuation Model (Series Elastic Actuator) in Matlab. Project at Carnegie Mellon University.

## Objectives
1. Develop a series elastic actuator of a humanoid robot. 48V Maxon EC90 Flat motor used in model.
2. Develop a cascaded controller to stabilize the humanoid to various reference heights: a) 0.7m, b) 0.8m, c) 0.9m. Humanoid begins at 1.0m.

## Description
The system begins with main.m with the desired y-position and trajectory tracking type (dir). The trajectory tracking type can be either a quadratic path from the initial y-position to the final y-position with a ramp velocity profile (dir = 1) or a single waypoint (final y-position) passed to the trajectory generator (dir = 2). The final y-position, dir, and the simulation time step (defined in main.m) are then passed to trajectory_generator.m where the desired output trajectory of states is generated, which includes y. trajectory_generator.m makes use of trajectory_planner.m by sending the determined waypoints to it in order to form the full trajectory. 

trajectory_generator.m also provides the total simulation time to main.m in order to define the full time-vector for simulation. With the desired y, the system flow then moves into behaviour_controller.m where the outer loop PD controller resides. This controller regulates the robot height based on the error between the desired y-position and the current y-position, and computes the desired leg force (spring force), and the desired motor position to be used in sea_controller.m. 

The system flow then proceeds into sea_controller.m where the inner loop PD controller resides. This controller regulates the motor position based on the error between the desired motor position (θm_des) and the current motor position (θm), and computes the desired motor torque. The desired leg force and desired motor torque then get passed to dynamics.m where the change in current state is computed, required for closed loop control in the following iteration. The state vector is defined by [y; y_dot; θm; θm_dot].
