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
    DH = [ q(1), 0.13122,        0,  pi/2;
           q(2)-pi/2,        0,   -0.1104,     0;
           q(3),        0, -0.096,     0;
           q(4)-pi/2,  0.0634,        0,  pi/2;
           q(5)+pi/2,  0.07505,        0, -pi/2;
           q(6),   0.0456,        0,     0 ];
    
    
    IT_B = [ -1, 0, 0,             -0.5162 + 0.06; 
             0, -1, 0,              -0.0282 + 0.03; 
             0, 0, 1,               0.061;
             0, 0, 0,               1 ];
         
    ET_F = [  0.7071, 0.7071, 0,        0; 
              -0.7071, 0.7071, 0,        0; 
              0, 0, 1,        0 + 0.09;
              0, 0, 0,         1 ];
end