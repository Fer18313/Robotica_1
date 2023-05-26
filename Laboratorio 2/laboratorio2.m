% =========================================================================
% MT3005 - LABORATORIO 2: Cinemática de cuerpos rígidos en 2D
% -------------------------------------------------------------------------
% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
% 1/18/2023
% =========================================================================
%% Inciso 2. NO modificar, sólo se usa para la auto-calificación
fun_trans_rigida = trans_rigida(1, 2, 45); 

%% Inciso 3. NO modificar, sólo se usa para la auto-calificación
fun_cart2hom = cart2hom([-1; 2; 4]);
fun_hom2cart = hom2cart(fun_cart2hom);

%% Inciso 4. 
% {I}: marco de referencia inercial
% {B}: marco de referencia del robot
% {C}: marco de referencia del sensor 1
% {D}: marco de referencia del sensor 2

IT_B = trans_rigida(0,1,90);
BT_C = trans_rigida(0.5,-0.5,-45);
BT_D = trans_rigida(0.5,0.5,45);
Cp = [2;0];
Dp = [1;0];

IT_C = IT_B*BT_C;
IT_D = IT_B*BT_D;
Ip1 = hom2cart(IT_C*cart2hom(Cp));
Ip2 = hom2cart(IT_D*cart2hom(Dp));
sensor_malo = 2;


%% Inciso 5.
p1 = [0;0];
p2 = [1;0];
p3 = [1;1];
p4 = [0;1];

A = [0 1 1 0];
B = [0 0 1 1];
C = [2 2.8660 2.3660 1.5000];
D = [3 3.5000 4.3660 3.8660];

v1 = hom2cart(trans_rigida(2,3,30)*cart2hom(p1));
v2 = hom2cart(trans_rigida(2,3,30)*cart2hom(p2));
v3 = hom2cart(trans_rigida(2,3,30)*cart2hom(p3));
v4 = hom2cart(trans_rigida(2,3,30)*cart2hom(p4));
P = [p1 p2 p3 p4];
V = [v1 v2 v3 v4];

patch(A,B,'blue');
hold on;
patch(C,D,'red');
grid on;