% =========================================================================
% MT3005 - LABORATORIO 6: Cinem�tica diferencial num�rica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la gu�a adjunta
% =========================================================================
function J = robot_jacobian(q, delta)    
    if(size(q, 2) ~= 1)
        error('robot_jacobian expects the configuration as a column vector.');
    end

    if(nargin == 1)
        delta = 1e-6;
    end

    % Dimensi�n de la configuraci�n (DOFs)
    n = length(q); 
    
    % COMPLETAR 
    % |   |   |
    % v   v   v
    
    % Ciclo responsable de calcular la j-�sima columna del jacobiano, con 
    % base en el algoritmo descrito en la gu�a del laboratorio
    for j = 1:n
        K = robot_fkine(q);
        R = tform2rotm(K); 
        q_n = q;
        q_n(j) = q(j) + delta;
        K_n = robot_fkine(q_n);
        dKj = (K_n - K) / delta;
        dO = dKj(1:3,4);
        dR = tform2rotm(dKj);
        Sw = dR * (R');
        w_q = [Sw(3,2);Sw(1,3);Sw(2,1)]; 
        J(:, j) = [dO;w_q];
    end
end 