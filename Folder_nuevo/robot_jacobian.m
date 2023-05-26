% =========================================================================
% MT3005 - LABORATORIO 6: Cinem�tica diferencial num�rica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la gu�a adjunta
% =========================================================================
function J = robot_jacobian(q, delta)    
%ROBOT_JACOBIAN 
% Obtiene la cinem�tica diferencial del robot descrito (mediante sus par�metros DH) 
% en la funci�n robot_def.     
    if(size(q, 2) ~= 1)
        error('robot_jacobian expects the configuration as a column vector.');
    end

    if(nargin == 1)
        delta = 1e-6;
    end

    % Dimensi�n de la configuraci�n (DOFs)
    n = length(q); 
    
    % Se calcula la cinem�tica directa del robot
    T = robot_fkine(q);
    
    % Se extrae la matriz de rotaci�n R
    R = T(1:3, 1:3);

    % Se inicializa el jacobiano
    J = zeros(6, n);
    
    % Ciclo responsable de calcular la j-�sima columna del jacobiano, con 
    % base en el algoritmo descrito en la gu�a del laboratorio
    for j = 1:n
        e = zeros(6, 1);
        e(j) = delta;
        
        dKdqj = (robot_fkine(q + e) - T) / delta;
        dtdqj = dKdqj(1:3,end);
        dRdqj = dKdqj(1:3,1:3);
        S = dRdqj*R.';
        dthetadqj = vex(S);
        
        J(:, j) = [dtdqj; dthetadqj];
    end
end 