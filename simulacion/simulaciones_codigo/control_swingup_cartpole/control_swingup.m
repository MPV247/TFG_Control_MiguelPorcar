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
Ke = 30.0;  

%Control de posición (ts = 1.5s, xi = 0.7)
Kp = 16.0; 
Kd = 4.16; 

%% 4. SIMULACIÓN TEMPORAL (MÉTODO RECURSIVO NO LINEAL)
N = 500;          % Número de iteraciones de la simulación
T = 10e-3;        % Periodo de muestreo/integración (s)

% Inicialización de matrices de estado y control
x = zeros(4, N);
x(2, 1) = 0.0;            % Condición inicial del ángulo (0 rad), posición estable
x(4, 1) = 0.05;           % Velocidad angular inicial (rad/s)

u = zeros(1, N);
t = zeros(1, N);

u_max = 3.0;            % Saturación del actuador (N)
E_ref = mp*g*lcm;        % Energía de referencia (Péndulo vertical hacia arriba)

for k = 1:N-1
   % --- Energía mecánica del sistema en instante K ---
   E_c = 0.5* J*x(4, k)^2;
   E_p = - mp*g*lcm*cos(x(2, k));
   E_m =  E_c + E_p; 

   % --- LEY DE CONTROL ---
   x_ddot_r = Ke * x(4, k) * cos(x(2, k)) * (E_m - E_ref) - Kp * x(1, k) - Kd * x(3, k);

   % --- PFL ---
   theta_ddot = (1 / J) * (-mp * g * lcm * sin(x(2, k)) - mp * lcm * cos(x(2, k)) * x_ddot_r - cp * x(4, k));

   u(k) = (mc + mp) * x_ddot_r + mp * lcm * cos(x(2, k)) * theta_ddot - mp * lcm * sin(x(2, k)) * x(4, k)^2 + cc * x(3, k);
   
   % --- Saturación del actuador (Seguridad de la planta) ---
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
theta     = x(2, :); 
vel       = x(3, :);
theta_dot = x(4, :);

%% Figuras
hold on
plot(t, theta)
plot(t, dist)
plot(t, u);

legend("theta", "x", "u");