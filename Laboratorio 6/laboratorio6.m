% =========================================================================
% MT3005 - LABORATORIO 6: Cinem�tica diferencial num�rica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la gu�a adjunta
%
% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
%
% =========================================================================
%% Inciso 2.
% Verificaci�n de la matriz de par�metros DH del manipulador. 
% NO MODIFICAR, los valores se usan s�lo para la autocalificaci�n. 
qtest = (pi/6) * ones(6, 1);
DH = robot_def(qtest);
qtest_1 = [0;-pi/2;0;0;pi/2;0];
qtest_2 = [0;pi/2;0;0;-pi/2;0];
%% Inciso 3.
% Verificaci�n de la cinem�tica directa del manipulador. 
% NO MODIFICAR, los valores se usan s�lo para la autocalificaci�n. 
K = robot_fkine(qtest);


%% Inciso 4
% Dimensiones del robot (en mm)
d1 = 131.22; a1 = 0; alpha1 = pi/2; offset1 = 0;
d2 = 0; a2 = -110.4; alpha2 = 0; offset2 = - pi/2;
d3 = 0; a3 = -96; alpha3 = 0; offset3 = 0;
d4 = 63.4; a4 = 0; alpha4 = pi/2; offset4 = - pi/2;
d5 = 75.05; a5 = 0; alpha5 = -pi/2; offset5 = pi/2;
d6 = 45.6; a6 = 0; alpha6 = 0; offset6 = 0;

% Definimos el robot como un objeto SerialLink
J1 = Revolute('d', d1, 'a', a1, 'alpha', alpha1, 'offset', offset1);
J2 = Revolute('d', d2, 'a', a2, 'alpha', alpha2, 'offset', offset2);
J3 = Revolute('d', d3, 'a', a3, 'alpha', alpha3, 'offset', offset3);
J4 = Revolute('d', d4, 'a', a4, 'alpha', alpha4, 'offset', offset4);
J5 = Revolute('d', d5, 'a', a5, 'alpha', alpha5, 'offset', offset5);
J6 = Revolute('d', d6, 'a', a6, 'alpha', alpha6, 'offset', offset6);
myCobot = SerialLink([J1, J2, J3, J4, J5, J6], 'name', 'myCobot')

myCobot.fkine(qtest);

%% Inciso 5.
% Verificaci�n de la cinem�tica diferencial del manipulador. 
% NO MODIFICAR, los valores se usan s�lo para la autocalificaci�n.
J = robot_jacobian(qtest);
Jv = J(1:3, :); 
Jw = J(4:6, :);

%% Inciso 6.
% An�lisis de singularidades

robot_ellipsoid(qtest_1,'v')
robot_ellipsoid(qtest_2,'v')



