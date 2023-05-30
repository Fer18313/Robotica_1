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
    
    
    q_1 = [-20;-45;5;-45;5;2]; % q intermedio, obtenido de q.teach
    %robotat_mycobot_send_angles(Robot, N, q_1)
    
elseif (objeto(2) < base(2)) % cuadrante superior
    q_1 = [45;-45;5;-45;5;2]; % q intermedio, obtenido de q.teach
    %robotat_mycobot_send_angles(Robot, N, q_1)

end

% Transformada de Objeto Homogenea
% El offset en z(3) se debe a que el marker se encuentra en el suelo y la esponja no.
T_obj = [eul2rotm(deg2rad(angles)),[objeto(1);objeto(2);objeto(3)+0.15];0,0,0,1];  
% Matriz de rotacion target (rotar en 180 a x, para que el z del efector este al reves)
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
robotat_mycobot_set_gripper_state_open(Robot, N)

%% Simulacion del robot

% Dimensiones del robot (en m)
d1 = 131.22/1000; a1 = 0; alpha1 = pi/2; offset1 = 0;
d2 = 0; a2 = -110.4/1000; alpha2 = 0; offset2 = - pi/2;
d3 = 0; a3 = -96/1000; alpha3 = 0; offset3 = 0;
d4 = 63.4/1000; a4 = 0; alpha4 = pi/2; offset4 = - pi/2;
d5 = 75.05/1000; a5 = 0; alpha5 = -pi/2; offset5 = pi/2;
d6 = 45.6/10000; a6 = 0; alpha6 = 0; offset6 = 0;

% Definimos el robot como un objeto SerialLink
J1 = Revolute('d', d1, 'a', a1, 'alpha', alpha1, 'offset', offset1);
J2 = Revolute('d', d2, 'a', a2, 'alpha', alpha2, 'offset', offset2);
J3 = Revolute('d', d3, 'a', a3, 'alpha', alpha3, 'offset', offset3);
J4 = Revolute('d', d4, 'a', a4, 'alpha', alpha4, 'offset', offset4);
J5 = Revolute('d', d5, 'a', a5, 'alpha', alpha5, 'offset', offset5);
J6 = Revolute('d', d6, 'a', a6, 'alpha', alpha6, 'offset', offset6);
Tool = transl([0,0,150/1000])*trotz(-45);
myCobot = SerialLink([J1, J2, J3, J4, J5, J6], 'name', 'myCobot','tool',Tool)

k = 1;

[q_i,hist_pos] = robot_ikine(I_T,q_1,'full','dampedls');

while(k<size(hist_pos,2))
        test = res2(hist_pos(:,k))
        myCobot.plot(rad2deg(test'));
        k=k+1;
end

plot(hist_pos', 'LineWidth', 1);
