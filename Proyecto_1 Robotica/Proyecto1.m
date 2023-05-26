% Alumno: Fernando Javier Sandoval Ruballos
% Carne: 18313
%robotat= robotat_connect('192.168.50.200');

% % Rutina para obtencion de pose para el robot en circulos
% xi2 = [0,0,0];
% for i= 1:200
%     xi2_ = robotat_get_pose(robotat,2,'eulzyx');
%     xi2(1,i) = xi2_(1,1);
%     xi2(2,i) = xi2_(1,2);
%     xi2(3,i) = xi2_(1,3);
%     pause(0.8)
% end
% coordenadas xyz con respecto a sistema de captura
% x1 = [0.9454,-0.8172,0.6796]; unicamente comprobacion
% x2 = [0,0,0]; 
% x3 = [-0.4986, 1.0273, 0.3959];
% x4 = [-0.6749,-0.7924,-0.0029];


load('pose1_quat.mat'); %UNICAMENTE COMPROBACION
load('pose3_quat.mat');
load('pose4_quat.mat');
load('pose2_25points_eulzyx.mat');
load('pose2_6points_eulzyx.mat');
load('Nd2.mat');

% CUERPO RIGIDO 2, EN MOVIMIENTO
phi2 = xi2(6);
theta2 = xi2(5);
psi2 = xi2(4);
AT_2 = trans_hom(rotz(phi2)*roty(theta2)*rotz(psi2),[xi2_(1);xi2_(2);xi2_(3)]);

% CUERPO RIGIDO 3 SOBRE ROVER
phi3 = xi3(6);
theta3 = xi3(5);
psi3 = xi3(4);
AT_3 = trans_hom(rotz(phi3)*roty(theta3)*rotz(psi3),[xi3(1);xi3(2);xi3(3)]); 

% CUERPO RIGIDO 4 EN SUELO
phi4 = xi4(6);
theta4 = xi4(5);
psi4 = xi4(4);
AT_4 = trans_hom(rotz(phi4)*roty(theta4)*rotz(psi4),[xi4(1);-0.70;xi4(3)]); 

% OBTENCION DE LA POSE DEL CUERPO RIGIDO 1
phi1 = 0;
theta1 = 0;
psi1 = 0;

xix = 0.912;
xiy = ; % esto se debe a que obtuvimos una 
xiz = -1.7 * xi3(3); % Sabemos que es una posicion intermedia con respecto al suelo y el cuerpo 1
AT_1 = trans_hom(rotz(phi1)*roty(theta1)*psi1(psi1),[DT2(3),xi4(2),-2*xi3(3)]);

hold on
trplot(AT_2,'color','r','frame','2')
plot(Vx,Vy);
trplot(AT_3,'color','m','frame','3')
trplot(AT_4,'color','g','frame','4')
trplot(T1_O,'color','b','frame','1')
grid on;



