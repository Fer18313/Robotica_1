% robotat= robotat_connect('192.168.50.200');
% 
% x1=robotat_get_pose(robotat,1,'eulzyx');
% %x2=robotat_get_pose(robotat,2,'eulzyx');
% x3=robotat_get_pose(robotat,3,'eulzyx');
% x4=robotat_get_pose(robotat,4,'eulzyx');
% 
% 
% 
% 
% x2=[0,0,0];
% for i=1:50
%     x2_=robotat_get_pose(robotat,2,'eulzyx');
%     
%     x2(1,i)=x2_(1,1);
%     x2(2,i)=x2_(1,2);
%     x2(3,i)=x2_(1,3);
%     
%     pause (0.5)
% end
%%

%  x2=[0,0,0];
% 

% Vx=x2(1,:);
% 
% Vy=x2(2,:);
% 
% Vz=x2(3,:);
% n=50;
% plot(Vx,Vy)
%  
%  
%  
% r=(max(Vx)-min(Vx))/2;
%%
%manejo de datos para x2
Vx = xi2(1,:);
Vy = xi2(2,:);
Vz = xi2(3,:);

figure (2)
plot(Vx,Vy);

rx2 = (max(Vx)-min(Vx))/2;

%% Vectores de distancia

IA = [xi3(1); xi3(2); xi3(3)];
IB = [xi4(1); xi4(2); xi4(3)];
IC = [Vx; Vy; Vz];

AD = [0.87-xi3(1); -0.908-xi3(2); sqrt((xi3-xi.^2)-((0.87-xi3(1))^2+(-0.908-xi3(2))^2))];
BD = [0.87-xi4(1); -0.908-xi4(2); sqrt((xi4-xi.^2)-((0.87-xi4(1))^2+(-0.908-xi4(2))^2))];
CD = [0.87-Vx; -0.908-Vy; 0.8640-Vz];


%coordenada x1 con trasnformacion como intermedio robot x3 ITX1
ITDa= transl(IA')*transl(AD')

%coordenada x1 con trasnformacion como intermedio robot x4  ITX1
ITDb= transl(IB')*transl(BD')

%coordenada x1 con trasnformacion como intermedio robot x4  ITX1
Xn=max(CD(1,:));
[row, col] = find(CD(1,:) == Xn);

Xn_=max(IC(1,:));

[row2, col2] = find(IC(1,:) == Xn_);


ITDc=transl(IC(:,col2))*transl(CD(:,col))


X1=[x1(1,1);x1(1,2);x1(1,3)]


Xn_2=min(IC(1,:));

[row3, col3] = find(IC(1,:) == Xn_2);


%%

figure(1)
TO = [1,0,0,0;0,1,0,0;0,0,1,0;0,0,0,0];
trplot(TO, 'frame', 'A', 'color', 'b')
hold on
trplot(ITDa, 'frame', 'D', 'color', 'r')
trplot(transl(IA'), 'frame', 'B', 'color', 'y')

%%
figure(2)
trplot(TO, 'frame', 'referencia', 'color', 'b')
hold on
trplot(ITDb, 'frame', 'X1', 'color', 'g')
trplot(transl(IA'), 'frame', 'X3', 'color', 'y')
trplot(transl(IB'), 'frame', 'X4', 'color', 'r')
trplot(transl(IC(:,col2)), 'frame', 'X2', 'color', 'black')
trplot(transl(IC(:,col3)), 'frame', 'X22', 'color', 'black')
%trplot(ITDc, 'frame', 'B', 'color', 'y')

xlim([-3 3]);
ylim([-3 3]);
zlim([-3 3]);


%  ITDC= transl(IC)*transl(CD)