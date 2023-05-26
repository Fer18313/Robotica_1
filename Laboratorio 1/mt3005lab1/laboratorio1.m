% =========================================================================
% MT3005 - LABORATORIO 1: Introducción a MATLAB
% -------------------------------------------------------------------------
% Nombre: Fernando Javier Sandoval Ruballos
% Carne: 18313
% 1/18/2023

% =========================================================================
%% Inciso 2.
v = [4 
    5 
    -1];

w = [1 3 5].'; %tranpose(w)

syms hx hy hz;

h = [hx 
    hy 
    hz];

A = [1 0 0
    0 2 0
    0 0 3];

B = [1 2 3
    -5 0 1
    4 5 6
    2 -1 -3];

b = [-1 2 0 1].';

%% Inciso 3.

% a) calcule la matriz C, utilizando unicamente el vector w.
C = w.*w.';
% b) la expresion p
p = sum(A*h.^2);
% c) Bx = b
x = inv(B.'*B)*(B.'*b);


%% Inciso 4.

% a) vector unitario u que es perpendicular a v y w.
numk = cross(w,v); 
u_hat = numk / norm(numk);

% b) los vectores r, s, t.
r = v + w;
s = v - w;
t = w - u_hat;

% c) los angulos en radianes.
alpha = subspace(v,r); 
beta = subspace(w,t);

% d) el producto interno entre s y u.
k = sum(s.*u_hat);

%% Inciso 5.
f = trans_lineal(1,2,3);
%% Inciso 6.
T = [trans_lineal(1,0,0) trans_lineal(0,1,0) trans_lineal(0,0,1)];
%% Inciso 7.
q = T\[-2 1 0].';

