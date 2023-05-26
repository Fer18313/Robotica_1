% =========================================================================
% MT3005 - LABORATORIO 7: Cinemática inversa numérica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la guía adjunta
% =========================================================================
%% Inciso 3.
% Verificación de TODAS las variantes de cinemática inversa. 
% NO MODIFICAR, los valores se usan sólo para la autocalificación. 
qtest = (pi/6) * ones(6, 1);
qtest2 = (pi/4) * ones(6,1);
qtest3 = [pi/2; 0.45; pi/3; pi/4; pi/6; pi/2];
q0 = zeros(6, 1);
q1 = [2*pi/3; pi/4; -3*pi/4; pi/4; 0; pi/4];
q2 = [0; pi; -pi/2; 0; 0; 0];


Tdtest = robot_fkine(qtest);
Tdtest2 = robot_fkine(qtest2);
Tdtest3 = robot_fkine(qtest3);

[qpos_pinv,qpos_pinv_hist] = robot_ikine(Tdtest, q0, 'pos', 'pinv');
pos_pinv = robot_fkine(qpos_pinv); pos_pinv = pos_pinv(1:3, 4);
[qpos_dampedls,qpos_dampedls_hist] = robot_ikine(Tdtest, q0, 'pos', 'dampedls');
pos_dampedls = robot_fkine(qpos_dampedls); pos_dampedls = pos_dampedls(1:3, 4);
[qpos_transpose,qpos_transpose_hist] = robot_ikine(Tdtest, q0, 'pos', 'transpose', 100);
pos_transpose = robot_fkine(qpos_transpose); pos_transpose = pos_transpose(1:3, 4);

[qrot_pinv,qrot_pinv_hist] = robot_ikine(Tdtest, q0, 'rot', 'pinv');
rot_pinv = robot_fkine(qrot_pinv); rot_pinv = rot_pinv(1:3, 1:3);
[qrot_dampedls,qrot_dampedls_hist] = robot_ikine(Tdtest, q0, 'rot', 'dampedls');
rot_dampedls = robot_fkine(qrot_dampedls); rot_dampedls = rot_dampedls(1:3, 1:3);
[qrot_transpose,qrot_transpose_hist] = robot_ikine(Tdtest, q0, 'rot', 'transpose', 100);
rot_transpose = robot_fkine(qrot_transpose); rot_transpose = rot_transpose(1:3, 1:3);

[qfull_pinv,qfull_pinv_hist] = robot_ikine(Tdtest, q0, 'full', 'pinv');
full_pinv = robot_fkine(qfull_pinv);
[qfull_dampedls,qfull_dampedls_hist] = robot_ikine(Tdtest, q0, 'full', 'dampedls');
full_dampedls = robot_fkine(qfull_dampedls);
[qfull_transpose,qfull_transpose_hist] = robot_ikine(Tdtest, q0, 'full', 'transpose', 100);
full_transpose = robot_fkine(qfull_transpose);

%% Inciso 4.
% Respuesta a las preguntas. Coloque el valor según las opciones que se le
% presentan para cada caso.

% Método con convergencia más rápida, opciones:
% 1 - pseudo-inversa
% 2 - levenberg-marquadt
% 3 - transpuesta

metodo_con_convergencia_mas_rapida = 1;

% Método con convercia más suave, opciones:
% 1 - pseudo-inversa
% 2 - levenberg-marquadt
% 3 - transpuesta

metodo_con_convergencia_mas_suave = 3;

% Combinación que NO funcionó como se esperaba:
% 1 - IK de posición con pseudo-inversa
% 2 - IK de posición con levenberg-marquadt
% 3 - IK de posición con transpuesta
% 4 - IK de orientación con pseudo-inversa
% 5 - IK de orientación con levenberg-marquadt
% 6 - IK de orientación con transpuesta
% 7 - IK completa con pseudo-inversa
% 8 - IK completa con levenberg-marquadt
% 9 - IK completa con transpuesta

combinacion_que_no_funciono = 9;

mdl_puma560;

% Configuraciones de prueba

qtest = (pi/6) * ones(6, 1);
Tdtest = robot_fkine(qtest);

% disp('Resultado del algoritmo de cinemática inversa:');
% disp(qfull_dampedls_hist)
% disp('Verificación con la pose de efector final deseada:')
% disp(full_dampedls)

% Histórico de la configuración
figure('WindowState', 'maximized');
subplot(1,2,1);
% plot(qpos_pinv_hist', 'LineWidth', 1);
% plot(qpos_transpose_hist', 'LineWidth', 1);
plot(qfull_pinv_hist', 'LineWidth', 1);

ylabel('$\mathbf{q}_k$', 'Interpreter', 'latex', 'FontSize', 18);
xlabel('$k$', 'Interpreter', 'latex', 'FontSize', 18);
grid minor;

% Animación del histórico
subplot(1,2,2);
p560.plot3d(q0');
waitforbuttonpress;
% p560.plot3d(qpos_pinv_hist');
% p560.plot3d(qpos_transpose_hist');
p560.plot3d(qfull_pinv_hist');


