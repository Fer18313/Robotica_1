% MATLAB controller for Webots
% File:          Laboratorio4.m
% Date:
% Description:
% Author:
% Modifications:

% uncomment the next two lines if you want to use
% MATLAB's desktop to interact with the controller:
%desktop;
%keyboard;

TIME_STEP = 64;

% get and enable devices, e.g.:
%  camera = wb_robot_get_device('camera');
%  wb_camera_enable(camera, TIME_STEP);
%  motor = wb_robot_get_device('motor');

elbow_joint = wb_robot_get_device('elbow<_joint');
shoulder_lift_joint = wb_robot_get_device('shoulder_lift_joint');
shoulder_pan_joint = wb_robot_get_device('shoulder_pan_joint');
wrist_1_joint = wb_robot_get_device('wrist_1_joint');
wrist_2_joint = wb_robot_get_device('wrist_2_joint');
wrist_3_joint = wb_robot_get_device('wrist_3_joint');
% main loop:
% perform simulation steps of TIME_STEP milliseconds
% and leave the loop when Webots signals the termination
%
while wb_robot_step(TIME_STEP) ~= -1

k = 0+(pi/2)*rand(6,1);

  % read the sensors, e.g.:
  %  rgb = wb_camera_get_image(camera);
  wb_motor_set_position(elbow_joint, q(1));
  wb_motor_set_position(shoulder_lift_joint, q(2));
  wb_motor_set_position(shoulder_pan_joint, q(3));
  wb_motor_set_position(wrist_1_joint, q(4));
  wb_motor_set_position(wrist_1_joint, q(5));
  wb_motor_set_position(wrist_3_joint, q(6));
  % Process here sensor data, images, etc.

  % send actuator commands, e.g.:
  %  wb_motor_set_postion(motor, 10.0);

  % if your code plots some graphics, it needs to flushed like this:
  %drawnow;
  pause(3);

end

% cleanup code goes here: write data to files, etc.
