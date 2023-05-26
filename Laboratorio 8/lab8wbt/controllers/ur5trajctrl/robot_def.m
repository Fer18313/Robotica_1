% =========================================================================
% MT3005 - LABORATORIO 8: control cinemático de manipuladores seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la guía adjunta
% =========================================================================
function [DH, IT_B, ET_F] = robot_def(q)
%ROBOT_DEF 
% Función en donde deben definirse los parámetros cinemáticos del manipulador 
% serial de interés.

    % Matriz de parámetros DH
    DH = [ q(1), 0.089459,        0,  pi/2;
           q(2),        0,   -0.425,     0;
           q(3),        0, -0.39225,     0;
           q(4),  0.10915,        0,  pi/2;
           q(5),  0.09465,        0, -pi/2;
           q(6),   0.0823,        0,     0 ];
    
    % Transformaciones de base y herramienta
    IT_B = eye(4);
    ET_F = [ 1, 0, 0,             0; 
             0, 1, 0,             0; 
             0, 0, 1, (13+150)/1000;
             0, 0, 0,             1 ];
end