%% 1. PARÁMETROS EXPERIMENTALES DEL SISTEMA
clear; clc; close all;

mc = 1.08;     % Masa del carro (Kg)
mp = 0.12;     % Masa del péndulo (Kg)
M  = mp + mc;  % Masa total del sistema (Kg)
J  = 0.0139;   % Momento de inercia del sistema (Kgm²)
cp = 0.0002;   % Coeficiente de rozamiento del péndulo (Nm·rad/s)
cc = 4.08;     % Coeficiente de rozamiento del carro (Ns/m)
g  = 9.81;     % Aceleración de la gravedad (m/s²)
lcm = 0.3;     % Distancia del centro de masas al eje de rotación (m)

%% 2. OBTENCIÓN DE LA REPRESENTACIÓN EN ESPACIO DE ESTADOS (SS)
% Matrices inercial (M_bar), amortiguamiento (C_bar) y gravitatoria (G_bar)
M_bar = [M,       -mp*lcm;
        -mp*lcm,   J]; 
    
C_bar = [cc,       0;
         0,        cp]; 
     
G_bar = [0,        0; 
         0,       -mp*g*lcm]; 

% Construcción de la matriz dinámica A (4x4) y matriz de entrada B (4x1)
A = [zeros(2),      eye(2);
     -M_bar\G_bar, -M_bar\C_bar];

B = [0; 0; (M_bar \ [1; 0])];

% Conversión de entrada: Relación estática lineal (de Voltios a Newtons)
B_volt = 0.64 * B; 

C = eye(4); % Matriz de salida: Monitorización completa del vector de estados
D = 0;      % Matriz de paso directo

%% 3. SINTONÍA DE CONTROLADORES (LQR & POLE PLACEMENT)
n = 4; % Número de estados
m = 1; % Número de entradas

% --- Opción A: Optimizador Lineal Cuadrático (LQR) ---
Q = diag([1, 1, 1, 1]); 
r = 0.1;                
R = r * eye(m);       
[K_lqr, S, E] = lqr(A, B_volt, Q, R);

% --- Opción B: Asignación de Polos (PLC) ---
p = [-2.5, -3, -3.5, -4]; 
K_plc = place(A, B_volt, p);


