% =========================================================================
% MT3005 - LABORATORIO 7: Cinemática inversa numérica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Para la implementación de los algoritmos numéricos ver la guía y seguir
% las instrucciones.
% 
% INSTRUCCIONES DE USO:
% -------------------------------------------------------------------------
% La función robot_ikine puede tomar un número variable de argumentos,
% siendo el mínimo 2 (Td y q0) y el máximo 7. Los parámetros que no se
% especifican toman su valor default.
%
% - Td      pose de efector final deseada (en forma de una matriz de
%           transformación homogénea de 4x4.
%
% - q0      configuración inicial (inicialización del algoritmo) en forma
%           de vector columna.
%
% - type    tipo de cinemática inversa, posibles valores:
%           -> 'pos'  para sólo cinemática inversa de posición (default).
%           -> 'rot'  para sólo cinemática inversa de orientación.
%           -> 'full' para cinemática inversa completa.
%
% - method  tipo de algoritmo numérico, posibles valores:
%           -> 'pinv'      pseudo-inversa
%           -> 'dampedls'  algoritmo de Levenberg-Marquadt
%           -> 'transpose' transpuesta
%
% - maxiter número máximo de iteraciones para el algoritmo (default = 50).
%
% - tolp    tolerancia para el error de posición (default = 1e-06).
%
% - tolo    tolerancia para el error de orientación (default = 1e-05).
%
% Finalmente, la función retorna (el segundo valor de retorno puede 
% obviarse):
% 
% - q       solución al problema de cinemática inversa, en forma de vector
%           columna.
%
% - q_hist  histórico de la evolución iterativa de la solución a la
%           cinemática inversa, corresponde a un array de tamaño n x k.
% =========================================================================
function [q, q_hist] = robot_ikine(Td, q0, type, method, maxiter, tolp, tolo)
%ROBOT_IK 
% Obtiene la cinemática inversa del robot descrito (mediante sus parámetros DH) 
% en la función robot_def. 
    if((nargin < 2) || (nargin > 7))
        error('Invalid number of arguments.');
    end

    switch nargin
        case 2
            type = 'pos';
            method = 'pinv';
            maxiter = 50;
            tolp = 1e-06;
            tolo = 1e-05;
        case 3
            method = 'pinv';
            maxiter = 50;
            tolp = 1e-06;
            tolo = 1e-05;
        case 4
            maxiter = 50;
            tolp = 1e-06;
            tolo = 1e-05;
        case 5
            tolp = 1e-06;
            tolo = 1e-05;
        case 6
            tolo = 1e-05;
    end

    if(size(q0, 2) ~= 1)
        error('robot_ikine expects the initial configuration as a column vector.');
    end

% Inicialización
    q = q0;
    k = 0;
    q_hist = zeros(numel(q0), maxiter); % histórico de la configuración
    % COMPLETAR 
    % |   |   |
    % v   v   v
    T = robot_fkine(q);
    
    ep = [Td(1:3,4)-T(1:3,4)];
    eo = [Td(1:3,4)-T(1:3,4)];

    
    
    % Ciclo responsable de implementar el método iterativo, según el 
    % algoritmo descrito en las notas de clase
    while( ((norm(ep) > tolp) || (norm(eo) > tolo)) && (k < maxiter))
    % COMPLETAR 
    % |   |   |
    % v   v   v 
       
    Dq = q;
    j = robot_jacobian(Dq);
    tr = robot_fkine(Dq);
    Q2 = rot2cuat(Td(1:3,1:3));
    q2 = rot2cuat(tr(1:3,1:3));
    Pq_e = multcuat(Q2,invcuat(q2));
    
        switch type
            case 'pos'
                eo = zeros(3, 1);
                e  = [Td(1:3,4)-tr(1:3,4)];
                I  = eye(3);
                J  = j(1:3,:);
            case 'rot'
                ep = zeros(3, 1);
                e  = Pq_e(2:4);
                I  = eye(3);
                J  = j(4:6,:);
            case 'full'
                ep = [Td(1:3,4)-tr(1:3,4)];
                eo = Pq_e(2:4);
                e  = [ep; eo];
                J = j;
                I  = eye(size(j,1));

            otherwise
                error('Invalid ikine type.');
        end
        switch method
            case 'pinv'
                J_t=pinv(J);
            case 'dampedls'
                J_t=J'/(J*J'+0.1*I);
            case 'transpose'
                alpha=(e'*J*J'*e)/(e'*J*J'*J*J'*e);
                J_t=alpha*J';
            otherwise
                error('Invalid ikine method.');
        end

        q = Dq+J_t*e;
        k = k + 1;
        q_hist(:, k) = q; % se almacena la configuración en el histórico 
    end

    disp(['ikine algorithm ended after ', num2str(k), ' iterations']);
    q_hist = q_hist(:, 1:k);
end