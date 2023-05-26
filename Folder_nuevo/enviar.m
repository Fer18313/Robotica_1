function [N] = enviar(Robot,q0,qi,qii,qf,N)
% Envio configuracion para objeto
robotat_mycobot_send_angles(Robot, N, qi)
pause(2);
% Grip al objeto
robotat_mycobot_set_gripper_state_closed(Robot, N)
pause(2);
% Reset POS de brazo
robotat_mycobot_send_angles(Robot, N, q0) % tomamos q0 como pose int.
pause(2);
robotat_mycobot_send_angles(Robot,N,qf)
pause(2);
% Release object
robotat_mycobot_set_gripper_state_open(Robot, N)
pause(3);
% Reset al terminar la rutina
robotat_mycobot_send_angles(robotat,robot,q0)
robotat_mycobot_set_gripper_state_closed(Robot, N)
N = 1
end

