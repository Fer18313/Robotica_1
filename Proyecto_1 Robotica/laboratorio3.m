% =========================================================================
% MT3005 - LABORATORIO 3: Ángulos de Euler y transformaciones elementales
% -------------------------------------------------------------------------
% Ver las instrucciones en la guía adjunta

% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 13813
% =========================================================================
%% Inciso 2.
% Sub-inciso (a)
phi1 = 0; 
theta1 = rad2deg(-pi/2);
psi1 = 0;
AT_D = trans_hom(rotz(phi1)*roty(theta1)*rotz(psi1),[-1;1;0]);

% Sub-inciso (b)
phi2 = 0;
theta2 = rad2deg(pi/2);
psi2 = rad2deg(pi/2);
CT_D = trans_hom(rotz(phi2)*roty(theta2)*rotx(psi2),[0;0;2]);

% Sub-inciso (c)
phi3 = rad2deg(pi);
theta3 = 0;
psi3 = rad2deg(-pi/2);
AT_C = trans_hom(rotx(phi3)*roty(theta3)*rotz(psi3),[-1;1;2]);

% Sub-inciso (d)
BT_C = [1 0 0 4;
        0 1 0 0;
        0 0 1 0;
        0 0 0 1];

%AT_D * DT_C = AT_C * CT_B = AT_B
AT_B = AT_D*inv(CT_D)*inv(BT_C);

% Sub-inciso (e)
% Mostrar figura marcos de referencia

hold on
trplot(AT_D,'color','r')
trplot(CT_D,'color','m')
trplot(AT_C,'color','g')
trplot(BT_C,'color','b')
grid on;


%% Inciso 3. 
% Sub-inciso (a)
AT_Da = transl(3,3,6); 

% Sub-inciso (b)
AT_Db = transl(0,0,2)*transl(3,0,0)*transl(0,0,2)*transl(-3,0,0)*transl(0,3,0)*transl(0,0,2)*transl(3,0,0);

% Sub-inciso (c)
AT_Dc = transl(0,0,2)*(troty(90)*transl(0,0,3))*(troty(-90)*transl(0,0,2))*(troty(-90)*transl(0,0,3))*(trotx(-90)*transl(0,0,3))*(troty(90)*transl(0,0,2))*(trotx(-90)*transl(0,0,3))*trotx(90)*trotz(90);

% Sub-inciso (d)
AT_Dd = (transl(3,0,0)*trotx(180))*(transl(-3,0,0)*transl(3,-3,-2)*troty(-180)*transl(0,0,4)*trotz(180));

% Sub-inciso (e)
AT_De =0;