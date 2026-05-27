%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (simulacion)
%==========================================================================
% SCRIPT:      control_swingup.m
% CARPETA:     /simulacion/simulaciones_codigo/control_Kx_integrador_cartpole
% DESCRIPCIÓN: Define los parámetros físicos del péndulo invertido y
%              obtiene la respuesta del control SwingUp con PFL
%              
% ENTRADAS:    - Parámetros dinámicos teóricos/identificados del sistema.
% SALIDAS:     - control_swingup_T_XXms.pdf (Validación temporal del controlador)
%==========================================================================
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

%% 2. GANANCIAS CONTROLADOR
% Inyección de energía
Ke = 25.0;  

%Control de posición
Kp = 8.0; 
Kd = 4.0; 

%% 4. SIMULACIÓN TEMPORAL (MÉTODO RECURSIVO NO LINEAL)
N = 600;          % Número de iteraciones de la simulación
T = 10e-3;        % Periodo de muestreo/integración (s)

% Inicialización de matrices de estado y control
x = zeros(4, N);
x(2, 1) = 0.0;            % Condición inicial del ángulo (0 rad), posición estable

u = zeros(1, N);
t = zeros(1, N);

u_max = 4.68;            % Saturación del actuador (V)
E_ref = mp*g*lcm;        % Energía de referencia (Péndulo vertical hacia arriba)

for k = 1:N-1
   % Ley de control por realimentación del estado (usando K_plc):
   u(k) = -K_plc * (x(:,k) - ref'); 
   
   % Saturación del actuador (Seguridad de la planta)
   if (u(k) > u_max)
       u(k) = u_max;
   elseif (u(k) < -u_max)
       u(k) = -u_max;
   end
  
   % --- Dinámica No Lineal ("Realidad Física") ---
   M_t = [M,                     mp*lcm*cos(x(2,k));
          mp*lcm*cos(x(2,k)),    J];
      
   C_t = [cc,   -mp*lcm*x(4,k)*sin(x(2,k));
          0,     cp];
      
   G_t = [0; mp*g*lcm*sin(x(2,k))];
   
   % Vector de derivadas de los estados (Euler)
   f1 = x(3, k);
   f2 = x(4, k);
   f34 = M_t \ ([u(k); 0] - C_t*x(3:4, k) - G_t);
   
   % Integración numérica hacia la siguiente muestra
   x(:, k+1) = x(:, k) + T * [f1; f2; f34];
   t(k+1) = t(k) + T;
end

% Desglose de estados y centrado del ángulo en cero para la gráfica
dist      = x(1, :);
theta     = x(2, :) - pi; 
vel       = x(3, :);
theta_dot = x(4, :);