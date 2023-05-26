% =========================================================================
% MT3005 - LABORATORIO 6: Cinemática diferencial numérica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la guía adjunta
% =========================================================================
function K = robot_fkine(q)
%ROBOT_FKINE 
% Obtiene la cinemática directa del robot descrito (mediante sus parámetros DH) 
% en la función robot_def. 
    if(size(q, 2) ~= 1)
        error('robot_fkine expects the configuration as a column vector.');
    end

    % Transformaciones homogéneas elementales que definen las
    % transformaciones junta a junta empleadas en la convención DH
    Rz = @(theta) [ cos(theta), -sin(theta), 0, 0; 
                    sin(theta),  cos(theta), 0, 0; 
                             0,           0, 1, 0; 
                             0,           0, 0, 1 ];

    Tz = @(d) [ 1, 0, 0, 0; 
                0, 1, 0, 0; 
                0, 0, 1, d; 
                0, 0, 0, 1 ];

    Tx = @(a) [ 1, 0, 0, a; 
                0, 1, 0, 0; 
                0, 0, 1, 0; 
                0, 0, 0, 1 ];

    Rx = @(alpha) [ 1,          0,           0, 0; 
                    0, cos(alpha), -sin(alpha), 0; 
                    0, sin(alpha),  cos(alpha), 0; 
                    0,          0,           0, 1 ];
    
    % Transformación junta a junta completa
    Aj = @(theta, d, a, alpha) Rz(theta) * Tz(d) * Tx(a) * Rx(alpha);   

    % Se obtienen los parámetros DH y las transformaciones de base y
    % herramienta desde la definición del robot
    [DH, IT_B, ET_F] = robot_def(q);
    
    % Se calcula la cinemática directa del robot empleando las filas de la
    % matriz de parámetros DH
    K = IT_B;
    for j = 1:length(q)
        K = K * Aj(DH(j, 1), DH(j, 2), DH(j, 3), DH(j, 4));
    end
    K = K * ET_F;
end