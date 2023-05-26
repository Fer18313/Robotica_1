% =========================================================================
% MT3005 - LABORATORIO 5: Cinemática Directa de Manipuladores Seriales
% -------------------------------------------------------------------------
% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
% 01/03/2023
% =========================================================================
%% Inciso 2.
% Dimensiones del robot
d1 = 317;
a1 = 81;
alpha1 = -pi/2;
offset1 = 0;

d2 = 192.5;
a2 = 0;
alpha2 = -pi/2;
offset2 = -pi/2;

d3 = 400;
a3 = 0;
alpha3 = -pi/2;
offset3 = 0;

d4 = 168.5;
a4 = 0;
alpha4 = -pi/2;
offset4 = pi;

d5 = 400;
a5 = 0;
alpha5 = -pi/2;
offset5 = 0;

d6 = 136.3;
a6 = 0;
alpha6 = -pi/2;
offset6 = pi;

d7 = 133.75;
a7 = 0;
alpha7 = 0;
offset7 = -pi/2;

% Definición del robot
J1 = Revolute('d', d1, 'a', a1, 'alpha', alpha1, 'offset', offset1);
J2 = Revolute('d', d2, 'a', a2, 'alpha', alpha2, 'offset', offset2);
J3 = Revolute('d', d3, 'a', a3, 'alpha', alpha3, 'offset', offset3);
J4 = Revolute('d', d4, 'a', a4, 'alpha', alpha4, 'offset', offset4);
J5 = Revolute('d', d5, 'a', a5, 'alpha', alpha5, 'offset', offset5);
J6 = Revolute('d', d6, 'a', a6, 'alpha', alpha6, 'offset', offset6);
J7 = Revolute('d', d7, 'a', a7, 'alpha', alpha7, 'offset', offset7);

sawyer = SerialLink([J1, J2, J3, J4, J5, J6, J7], 'name', 'SAWYER')

%% Inciso 3.
q0 = [0,0,0,0,0,0,0];
sawyer.plot(q0,'zoom',50)
%% Inciso 4. 
q1 = [pi/2,0,0,pi/2,-pi/2,pi,-pi/2];
figure(2)
sawyer.teach(q1, 'zoom', 50)

%% Inciso 5.
q2 = [pi/2,-pi/4,pi/4,pi/2,-pi/4,pi/2,pi/4];
Rt1 = sawyer.fkine(q2);
t1 = Rt1.t;
theta1 = tr2rpy(Rt1,'deg')';
