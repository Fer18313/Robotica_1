%% FERNANDO SANDOVAL 18313
%% Configuracion Inicial> ROBOTAT
Robot = robotat_connect('192.168.50.200');
%% Datos Relevantes
%{ 
NUMERO DE MARKER> 
1: Efector final mycobot1
2: Efector final mycobot2
3: Objeto a manipular 1
4: Objeto a manipular 2
5: Aprox. base mycobot1
6: Aprox. base mycobot2
7: Marker fijo caja
8: Marker mediciones puntos intermedios
%}

%offset: 4.5 cm radio 
%offset: 7 cm altura

% Pose Efector Final
EF1 = robotat_mycobot_get_coords(Robot,2); 
% Pose Items a Manipular
item_1 = robotat_get_pose(Robot,4,'XYZ'); 
% Modificaciones Referencia
base_1 = robotat_get_pose(Robot,6,'XYZ');

% CONVERTIR MM - M
EF1_mm = EF1(1:3)/1000; % Efector Final Pose de mm a m.

% Transformacion base
oOB = [base_1(1)+0.045 base_1(2) base_1(3)+0.07]';
item_OTB = transl(-oOB)*[item_1(1:3)';1]; % OTB
item_OTB = item_OTB(1:3)';

item_1_DES = [item_1(1)+0.1;item_1(2)+0.1;item_1(3)-0.1]; %Desfase esponja

% Posicion Intermedia
item_OTB(3) = item_OTB(3);
item_OTB(2) = EF1_mm(2);
item_OTB(1) = item_OTB(1);

T1 = mtraj(@tpoly,EF1_mm',item_OTB,10); % GRIPPER TO INTER



T = [T1];
qrob = zeros(1,6);

for i = 1:size(T,1) %
    T_ik =[eye(3),T(i,:)';zeros(1,3),1];
    qrob(:,:,end+1) = robot_ikine(T_ik,qrob(:,:,end)','pos');
    
end

qrob(:,:,end+1) = [qrob(:,1,end),qrob(:,2,end),qrob(:,3,end),qrob(:,4,end),deg2rad(135),deg2rad(-155)]; %Rotación del efector final
qrob(:,:,end+1) = [qrob(:,1,end),qrob(:,2,end),qrob(:,3,end),qrob(:,4,end),deg2rad(135),deg2rad(-155)];

PosInt = robot_fkine(qrob(:,:,end)');
ObjPosInt = PosInt(1:3,4); %Int x,y,z

T2 = mtraj(@tpoly,ObjPosInt',item_1_DES',10); %INT TO SPONGE

T = [T2];

for i = 1:size(T,1)
    T_ik =[eye(3),T(i,:)';zeros(1,3),1];
    qrob(:,:,end+1) = robot_ikine(T_ik,qrob(:,:,end)','pos');
end


qrob = rad2deg(qrob); 
qrob = max(-160,qrob);
qrob = min(160,qrob);

%% Prueba de movimiento
k = 1;
robotat_mycobot_set_gripper_state_open(Robot,2); % Apertura del gripper
robotat_mycobot_send_angles(Robot,2,[0;0;0;0;0;0]); % Asegurar posición natural del brazo
% 
pause(1);
while(k<size(qrob,3))
    try
        robotat_mycobot_send_angles(robot,2,qrob(:,:,k)');
        k=k+1;
        pause(0.3);
    catch
        disp('Error envio de datos');
    end
end

robotat_mycobot_set_gripper_state_closed(robot,2);




