% =========================================================================
% MT3005 - LABORATORIO 6: Cinemática diferencial numérica de manipuladores 
%                         seriales
% -------------------------------------------------------------------------
% Ver las instrucciones en la guía adjunta
% =========================================================================
function [DH, IT_B, ET_F] = robot_def(q)
%ROBOT_DEF 
% Función en donde deben definirse los parámetros cinemáticos del manipulador 
% serial de interés.
    % Matriz de parámetros DH <-- COMPLETAR
    DH = [q(1,1) 131.22 0 pi/2;
        q(2,1)-pi/2 0 -110.4 0;
        q(3,1) 0 -96 0;
        q(4,1)-pi/2 63.4 0 pi/2;
        q(5,1)+pi/2 75.05 0 -pi/2;
        q(6,1) 45.6 0 0];
    % Transformaciones de base y herramienta
    IT_B = eye(4);
    ET_F = eye(4);
end