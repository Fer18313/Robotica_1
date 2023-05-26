%% SETUP INICIAL
Robot = robotat_connect('192.168.50.200');
%/*
% offset de base
% offset: 4.5 cm radio 
% offset: 7 cm altura
%*/

% Seleccion de brazo
N = 1; 
% Asegurar robot en posicion natural 
q0 = zeros(6,1);
robotat_mycobot_send_angles(Robot, N, q0);
pause(2);
robotat_mycobot_set_gripper_state_open(Robot, N);
pause(2);
% Extraccion de datos Optitrack
objeto = robotat_get_pose(Robot, 13, 'eulzyx');
base = robotat_get_pose(Robot, 15, 'eulzyx');
% Obtenemos la pose de la caja
box = robotat_get_pose(Robot, 17, 'eulzyx');

%% RUTINA PRINCIPAL

% Variables globales
angles = objeto(4:6);
q_ii = [-70;-45;-5;-5;-5;-5]; % Simulacion .teach q intermedio, caja

% Seleccion de pose intermedia segun posicion en plano del objeto
if (objeto(2) > base(2)) % cuadrante inferior
    
    q_1 = [0;-60;5;-45;5;5]; % q intermedio, obtenido de q.teach
    
elseif (objeto(2) < base(2)) % cuadrante superior
    
    q_1 = [0;-45;5;-45;5;5]; % q intermedio, obtenido de q.teach
    
end

% Transformada de Objeto Homogenea
% El offset en z(3) se debe a que el marker se encuentra en el suelo y la esponja no.
T_obj = [eul2rotm(deg2rad(angles)),[objeto(1);objeto(2);objeto(3)+0.15];0,0,0,1];  
% Matriz de rotacion target
R_TH = [1 0 0 0;
    0 -1 0 0;
    0 0 -1 0;
    0 0 0 1];

% Transformada final
I_T = T_obj*R_TH;
% Conversión a grados para la interpretacion de myCobot
q_i = rad2deg(robot_ikine(I_T,q_1,'full','dampedls'));
% Restricciones (evitar que se golpee el brazo) y truncar a 160
q_i = restricciones(q_i);


% Transformada de caja homogenea
% Los offset son para lograr que se deposite al centro de la caja
T_box = [eye(3),[box(1)-0.12;box(2)-0.05;box(3)+0.2];0,0,0,1];
q_f = rad2deg(robot_ikine(T_box,q_ii,'pos','transpose')); % Conversión a grados para la interpretacion de myCobot

% Restricciones (evitar que se golpee el brazo)
q_f = res2(q_f);

%% Rutina para enviar datos
 
% Envio configuracion para objeto
robotat_mycobot_send_angles(Robot, N, q_i)
pause(2);
% Grip al objeto
robotat_mycobot_set_gripper_state_closed(Robot, N)
pause(2);
% Reset POS de brazo
robotat_mycobot_send_angles(Robot, N, q0) % tomamos q0 como pose int.
pause(2);
robotat_mycobot_send_angles(Robot,N,q_f)
pause(2);
% Release object
robotat_mycobot_set_gripper_state_open(Robot, N)
pause(3);
% Reset al terminar la rutina
robotat_mycobot_send_angles(robotat,robot,q0)
robotat_mycobot_set_gripper_state_closed(Robot, N)


