function robot_set_config(joint_handles, q)
    for i = 1:numel(q)
        wb_motor_set_position(joint_handles(i), q(i));
    end
end