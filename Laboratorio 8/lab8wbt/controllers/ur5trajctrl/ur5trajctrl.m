% ============================================================
% NO MODIFICAR
% ============================================================
TIME_STEP = 64;
MAX_JOINT_SPEEDS = [pi; pi; pi; 2*pi; 2*pi; 2*pi];

% Handles para controlar los motores de las juntas del robot
joint_handles(1) = wb_robot_get_device('shoulder_pan_joint');
joint_handles(2) = wb_robot_get_device('shoulder_lift_joint');
joint_handles(3) = wb_robot_get_device('elbow_joint');
joint_handles(4) = wb_robot_get_device('wrist_1_joint');
joint_handles(5) = wb_robot_get_device('wrist_2_joint');
joint_handles(6) = wb_robot_get_device('wrist_3_joint');

% Handles para controlar los motores del gripper
gripper_handles(1) = wb_robot_get_device('finger_1_joint_1');
gripper_handles(2) = wb_robot_get_device('finger_2_joint_1');
gripper_handles(3) = wb_robot_get_device('finger_middle_joint_1');

% Velocidad de los motores de las juntas
% (se están empleando a la mitad de su velocidad máxima)
joint_speeds = 0.5 * MAX_JOINT_SPEEDS; 
for i = 1:6
  wb_motor_set_velocity(joint_handles(i), joint_speeds(i));
end

% Se coloca al robot en su configuración cero (HOME)
q0 = zeros(6, 1);
robot_set_config(joint_handles, q0);

i = 0; % tiempo actual de simulación (en iteraciones)
t = 0; % tiempo actual de simulación (en segundos)

% ============================================================

% ============================================================
%% FERNANDO SANDOVAL 18313
%% Configuracion Inicial

P0 = [-0.817 -0.354 -0.005];
P1 = [0.3 0.65 0.5];
P2 = [0.73 0.48 0.2];
P3 = [0.45 -0.75 0.65];
t_interval = 30;

T0 = mtraj(@tpoly, P0, P1, t_interval);
T1 = mtraj(@tpoly, P1, P2, t_interval);
T2 = mtraj(@tpoly, P2, P3, t_interval);

T = [T0; T1; T2];

ik = zeros(1,6);

for x=1:size(T, 1)
  pose = transl(T(x,:));
  ik(:,:,end + 1) = robot_ikine(pose, ik(:,:,end)', 'pos', 'transpose');
end

s = 0;

% ciclo principal de simulación:
while wb_robot_step(TIME_STEP) ~= -1 
  if (i<60)  
  s = s + 1;
  robot_set_config(joint_handles, ik(:,:,s)');
  end   
  if (i >= 60 && i <= 90)
  gripper_close(gripper_handles);
  end
  if (i > 90 && i < 120)
  robot_set_config(joint_handles, ik(:,:,s)');
  s = s + 1;
  end  
  if (i >= 120)
  gripper_open(gripper_handles);
  end
  
  % Se actualizan los tiempos actuales de simulación
  i = i + 1;
  t = t + TIME_STEP/1000;
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end