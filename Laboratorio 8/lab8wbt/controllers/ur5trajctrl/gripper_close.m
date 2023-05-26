function gripper_close(gripper_handles)
    wb_motor_set_position(gripper_handles(1), pi/3);
    wb_motor_set_position(gripper_handles(2), pi/3);
    wb_motor_set_position(gripper_handles(3), pi/3);
end