function gripper_open(gripper_handles)
    wb_motor_set_position(gripper_handles(1), 0);
    wb_motor_set_position(gripper_handles(2), 0);
    wb_motor_set_position(gripper_handles(3), 0);
end