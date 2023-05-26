%% FERNANDO SANDOVAL 18313
%% Configuracion Inicial
% config inicial
% puntos de trayectoria

P0 = [-0.8173 -0.3543 -0.0052]';
P1 = [0.3 0.65 0.5]';
P2 = [0.73 0.48 0.21]';
P3 = [0.45 -0.75 0.65]';
t_interval = 50;

%% METODOLOGIA
WP = transpose([P1 P2 P3]);
% generacion de trayectorias
T = mstraj(WP, [], [0.1,0.1,0.1], P0', 0.02, 0);
plot3(T(:,1),T(:,2),T(:,3));
%T0 = mtraj(@tpoly,P0,P1,t_interval); %intermedia
%T1 = mtraj(@tpoly,P1,P2,t_interval); %hacia lata
%T2 = mtraj(@tpoly,P2,P3,t_interval); %hacia caja
%T = [T0; T1; T2]; % Concatenacion 
ik = zeros(size(T,1),6);

for x = 1:size(T,1)
    pose = transl(T(x,:)); 
    ik(x,:) = robot_ikine(pose,ik(x,:)','pos','transpose'); %cinematica inversa
end

%prueba de animacion
ur5.plot(ik)

%% SEGUNDA PARTE EXTRAIDA WEBOTS
s = 0;
% ciclo principal de simulación:
while wb_robot_step(TIME_STEP) ~= -1
  if  (i<100)
  s = s + 1;
  robot_set_config(joint_handles,ik(s,:));
  end 
  if ((i>=100) & (i<120))
  gripper_close(gripper_handles);
  end
  if ((i>=120) & (i<170))
  robot_set_config(joint_handles,ik(s,:));
  s = s + 1;
  end
  if ((i>=150) & (i<200))
  s = s + 1;
  robot_set_config(joint_handles,ik(s,:));
  end
  if ((i>=200) & (i<220))
  gripper_open(gripper_handles);
  end
  % Se actualizan los tiempos actuales de simulación
  i = i + 1;
  t = t + TIME_STEP/1000;
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end