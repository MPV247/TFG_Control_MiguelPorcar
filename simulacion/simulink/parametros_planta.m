%% MODELADO DEL PÉNDULO INVERTIDO SOBRE CARRO. 
%Parámetros proporcionados por el fabricante.
mc = 1.12;   %Kg , masa del carro.
mp = 0.11;   %Kg , masa del péndulo.
M = mp + mc; %Kg, masa total del sistema.
J = 0.0136;  %Kgm², momento de inercia del sistema.
cp = 0.0005 ;   %Nm.rad/s, coeficiente de rozamiento del péndulo.
cc = 0.05;   %Ns/m , coeficiente de rozamiento del carro.
g = 9.81;    % m/s², aceleración debida a la gravedad
lcm = 0.17; %m, Distancia entre el centro de masas y el eje de rotación.  

%% Obtención de la representación en SS a partir de la dinámica

%Matrices M, C, G, B (Definen la dinámica del sistema). 

M_bar = [M,          +mp*lcm;
         +mp*lcm,      J]; 

C_bar = [cc,           0;
          0,          cp       ]; 

G_bar= [  0              0; 
          0,         -mp*g*lcm]; 

%A partir de estas matrices linealizadas en q = pi, se obtienen las matrices
%de estado A y B. 
%A: 4x4, se tienen cuatros estados. 
A= [zeros(2),        eye(2);
    -M_bar\G_bar ,-M_bar\C_bar];

%B: 4x1, una acción de control para cuatro estados.
B = [0;0;[M_bar\[1;0]]]

%% Controlador 
% 1. Definición de pesos
Q = eye(n);             % <--- Define el consumo (uso del actuador)

r = 0.01;                % <--- Define como de agresivo es.

R_lqr = r * eye(m);       

% 2. Resolver la optimización
[K_lqr, S, E] = lqr(A, B, Q, R_lqr);

%Uso de place para ubicar los polos.
[K_pl] = place(A, B, [-1, -2, -3, -4]);

