% ============================================================
% NO MODIFICAR
% ============================================================
TIME_STEP = 64;
MAX_WHEEL_VELOCITY = 12.3; % velocidad máxima ruedas (en rad/s)
WHEEL_RADIUS = (195/2); % radio de las ruedas (en mm)
DISTANCE_FROM_CENTER = (381/2); % distancia a ruedas (en mm)
% Velocidad lineal máxima (en mm/s)
MAX_SPEED = WHEEL_RADIUS * MAX_WHEEL_VELOCITY;

% Handles para controlar los motores de las ruedas del robot
left_wheel = wb_robot_get_device('left wheel');
right_wheel = wb_robot_get_device('right wheel');

% Handles de los sensores
gps = wb_robot_get_device('gps');
compass = wb_robot_get_device('compass');

% Set up para hacer control de velocidad en las ruedas
wb_motor_set_position(left_wheel, Inf);
wb_motor_set_position(right_wheel, Inf);
wb_motor_set_velocity(left_wheel, 0.0);
wb_motor_set_velocity(right_wheel, 0.0);

% Se habilitan los sensores
wb_gps_enable(gps, TIME_STEP);
wb_compass_enable(compass, TIME_STEP);

% Posiciones de meta
goal1 = [-4; 0];
goal2 = [3; -4];
goal3 = [1; 4];
goal4 = [3; 4];
goal_points = [goal1, goal2, goal3, goal4];

% Se carga el mapa (occupancy grid) y se muestra
load('lab9map.mat');
figure(1);
map = imrotate(map,180);
map = flip(map,2);
imshow(map);
map = double(map); % Requerido por la Toolbox

i = 0; % tiempo actual de simulación (en iteraciones)
t = 0; % tiempo actual de simulación (en segundos)
% ============================================================


% ============================================================
% MODIFICAR
% ============================================================
% Operaciones previas a la simulación

% SELECCION DE CONTROL Y TRAYECTORIA
select_case = 1;
select_DS_PRM = 2;


if select_case == 1
% PID orientación
kpO = 1*5;
kiO = 0.001; 
kdO = 0;
EO = 0;
eO_1 = 0;

% Acercamiento exponencial
v0 = 500;
alpha = 50;%0.5;50

xg = goal2(1);
yg = goal2(2);
thetag = 0;


elseif select_case == 2
xg = goal1(1);
yg = goal1(2);

ell = DISTANCE_FROM_CENTER; % en m

% Difeomorfismo para linealización por feedback
bearing  = 0;
finv = @(bearing,mu) [1,0; 0,1/ell] * [cos(bearing), -sin(bearing); sin(bearing), cos(bearing)]' * [mu(1); mu(2)];

% Matrices del sistema linealizado alrededor del punto de operación
A = zeros(2); 
B = eye(2); 
R = eye(2);


% LQR + Integradores (LQI)
Cr = eye(2);
Dr = zeros(2);
AA = [A, zeros(size(Cr')); Cr, zeros(size(Cr,1))];
BB = [B; Dr];
QQ = eye(size(A,1) + size(Cr,1)); QQ(3,3) = 10; QQ(4,4) = 10; 
Klqi = lqr(AA, BB, QQ, R);
ref = [xg; yg];
sigma = 0;

end



if select_DS_PRM == 1
% Planificacion de trayectorias
radius_in_cells = round(18); %Celdas 11.375
dstar = Dstar(map,'inflate', radius_in_cells);
c = dstar.costmap();

elseif select_DS_PRM == 2

radius_in_cells = round(18); %Celdas 11.375
map = robotics.BinaryOccupancyGrid(map);
inflate(map, radius_in_cells);
map = map.occupancyMatrix;

map = double(map); % Requerido por la Toolbox

prm = PRM(map);
prm.plan('npoints',150);
prm.plot();

end

bandera = 1;
contador = 1;


% ciclo principal de simulación:
while wb_robot_step(TIME_STEP) ~= -1
  gps_position = wb_gps_get_values(gps);

  
  if bandera == 1
  bandera = 0;
  gps_position = wb_gps_get_values(gps);
  
  start_x = gps_position(1);
  start_y = gps_position(2);

  goal_x = goal2(1);
  goal_y = goal2(2);

  start_x =  (map_value(start_x,-5,5,1,500));
  start_y = (map_value(start_y,-5,5,1,500));

  goal_x = (map_value(goal_x,-5,5,1,500));
  goal_y = (map_value(goal_y,-5,5,1,500));


  start_points = round([start_x,start_y]);
  goal = round([goal_x,goal_y]);
  
  if select_DS_PRM == 1 
  dstar.plan(goal,start_points);
  figure(2);
  path = dstar.query(start_points,'animate');
  
  elseif select_DS_PRM == 2
  path = prm.query(goal,start_points);
  prm.plot();
  path = flip(path,1);
  end
  
  for p = 1:length(path)
   path(p,1) = map_value(path(p,1),1,500,-5,5);
   path(p,2) = map_value(path(p,2),1,500,-5,5);
  end
  
 
  end
  
  if select_case == 1
  
   gps_position = wb_gps_get_values(gps);

   compass_values = wb_compass_get_values(compass);
   bearing = atan2(compass_values(1), compass_values(2));
   
   %bearing = bearing * 180 / pi;
   if contador < length(path)
    xg = path(contador,1);
    yg = path(contador,2);
    coords = [xg yg];
    disp(coords)
   end
   
    x = gps_position(1); y = gps_position(2); theta = bearing;
    e = [xg - x; yg - y];
    thetag = atan2(e(2), e(1));
            
    eP = norm(e);
    eO = thetag - theta;
    eO = atan2(sin(eO), cos(eO));
            
    % Control de velocidad lineal
    kP = v0 * (1-exp(-alpha*eP^2)) / eP;
    v = kP*eP;
            
    % Control de velocidad angular
    eO_D = eO - eO_1;
    EO = EO + eO;
    w = kpO*eO + kiO*EO + kdO*eO_D;
    eO_1 = eO;
    
         
    
   giroder = (v + w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   giroiz = (v - w*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   
   
   wb_motor_set_velocity(left_wheel, giroiz);
   wb_motor_set_velocity(right_wheel, giroder);
   
   if eP <= 0.1
   contador = contador + 1;
   end
   
   elseif select_case == 2
   
   
  % bearing = atan2(compass_values(1), compass_values(2));
 %  xi = [gps_position(1);gps_position(2);bearing];
   xi = zeros(2,1);
   compass_values = wb_compass_get_values(compass);
   bearing = atan2(compass_values(1), compass_values(2));
   gps_position = wb_gps_get_values(gps);
   x = [gps_position(1);gps_position(2)];
   
   sigma = sigma + (Cr*x - ref)*0.8;
   mu = -Klqi*[x; sigma];
   u = finv(bearing, mu);
   disp(gps_position)
   
   giroder = (u(1) + u(2)*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   giroiz = (u(1) - u(2)*DISTANCE_FROM_CENTER)/WHEEL_RADIUS;
   
   
  if (gps_position(1) > goal1(1) - 0.1 & gps_position(1) < goal1(1) + 0.1) & (gps_position(2) > goal1(2) - 0.1 & gps_position(2) < goal1(2) + 0.1)
   wb_motor_set_velocity(left_wheel, 0);
   wb_motor_set_velocity(right_wheel, 0);
   else
   wb_motor_set_velocity(left_wheel, giroiz);
   wb_motor_set_velocity(right_wheel, giroder);
   end
   
   
   end
   
  % Se actualizan los tiempos actuales de simulación
  i = i + 1;
  t = t + TIME_STEP/1000;
  
  % if your code plots some graphics, it needs to flushed like this:
  drawnow;
end

% cleanup code goes here: write data to files, etc.