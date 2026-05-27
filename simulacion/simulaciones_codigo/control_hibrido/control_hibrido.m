%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (Simulación Unificada)
%==========================================================================
% SCRIPT:      control_hibrido.m
% DESCRIPCIÓN: Implementa la simulación no lineal completa del Cart-Pole.
%              Inicia en la posición estable inferior (theta = 0) y ejecuta
%              un Swing-Up por energía (PFL) para balancear el péndulo. Al
%              alcanzar la vecindad de la vertical superior (theta = pi),
%              conmuta automáticamente a un control LQR aumentado con
%              acción integral para estabilizar la posición y el ángulo.
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

%% 2. DISEÑO DEL CONTROLADOR DE ESTABILIZACIÓN (LQR + INTEGRAL)
% Modelo linealizado continuo en la vertical superior (Upright position)
M_bar = [M,          -mp*lcm;
        -mp*lcm,      J]; 
C_bar = [cc,           0;
          0,          cp]; 
G_bar = [0,            0; 
         0,    -mp*g*lcm]; 

A = [zeros(2),        eye(2);
     -M_bar\G_bar, -M_bar\C_bar];
B = [0; 0; (M_bar\[1; 0])];

% Conversión del modelo a entrada en Voltios (1V -> 0.64 N)
B_volt = 0.64 * B; 

% Aumento del espacio de estados para añadir acción integral en el carro (X)
C_x = [1, 0, 0, 0]; 
Ae = [A,         zeros(4,1); 
      C_x,       0];
Be = [B_volt;    0];

% Sintonía del optimizador LQR
Q = diag([12, 1.5, 0.8, 0.1, 4.5]); % Pesos ajustados para priorizar captura
R = 0.001;        
[K_lqr, ~, ~] = lqr(Ae, Be, Q, R);

%% 3. CONFIGURACIÓN DE LA SIMULACIÓN TEMPORAL
T = 10e-3;            % Periodo de muestreo/integración (10 ms)
t_final = 8.0;        % Tiempo total de simulación (s)
N = round(t_final/T); % Número de iteraciones

% Inicialización de matrices de estado y control
x = zeros(4, N);
x(2, 1) = 0.0;        % Condición inicial: Péndulo totalmente ABAJO (0 rad)
x(4, 1) = 0.01;       % Pequeña perturbación inicial para romper el equilibrio estable

u = zeros(1, N);
t = zeros(1, N);
modo_activo = zeros(1, N); % Vector para registrar el controlador activo (1=SwingUp, 2=LQR)

% Parámetros de diseño del Swing-Up
Ke = 30.0;            % Ganancia de energía
Kp_su = 8.0;          % Kp amortiguación del carro en Swing-Up
Kd_su = 4.16;         % Kd amortiguación del carro en Swing-Up
E_ref = mp*g*lcm;     % Energía de referencia en la vertical superior

% Límites del Actuador (Estandarizado en Voltios)
u_max = 4.68;         
Ie = 0.0;             % Acumulador integral del LQR

% Estado inicial de la máquina de conmutación
controlador = "SwingUp";

%% 4. BUCLE DE SIMULACIÓN EN TIEMPO REAL (MÉTODO RECURSIVO NO LINEAL)
for k = 1:N-1
   % --- 4.1. Cálculo del error angular acotado y envuelto en [-pi, pi] ---
  
   error_theta = atan2(sin(pi - x(2,k)), cos(pi - x(2,k)));
   
   % --- 4.2. Máquina de Estados de Conmutación (Switching Logic) ---
   if controlador == "SwingUp"
       modo_activo(k) = 1; 
       
       if abs(error_theta) < 0.8 && abs(x(4,k)) < 2.0
           controlador = "LQR";
           Ie = 0.0; % Reseteo limpio del integrador
       end
       
   elseif controlador == "LQR"
       modo_activo(k) = 2;
       
       % CONDICIÓN DE CAÍDA / EMERGENCIA: Si una perturbación saca al péndulo de la zona lineal
       if abs(error_theta) > 0.60
           controlador = "SwingUp";
       end
   end
   
   % --- 4.3. Cálculo de las Leyes de Control ---
   if controlador == "SwingUp"
       % -- ALGORITMO 1: SWING-UP POR ENERGÍA + PFL --
       E_c = 0.5 * J * x(4, k)^2;
       E_p = - mp * g * lcm * cos(x(2, k));
       E_m =  E_c + E_p; 
       
       % Aceleración virtual del carro
       x_ddot_r = Ke * x(4, k) * cos(x(2, k)) * (E_m - E_ref) - Kp_su * x(1, k) - Kd_su * x(3, k);
       % Dinámica no lineal de la PFL
       theta_ddot_su = (1 / J) * (-mp * g * lcm * sin(x(2, k)) - mp * lcm * cos(x(2, k)) * x_ddot_r - cp * x(4, k));
       F_fuerza = (mc + mp) * x_ddot_r + mp * lcm * cos(x(2, k)) * theta_ddot_su - mp * lcm * sin(x(2, k)) * x(4, k)^2 + cc * x(3, k);
       
       % Conversión de Fuerza (N) a Voltios (V) para el actuador
       u_calc = F_fuerza / 0.64;
       
   elseif controlador == "LQR"
       % -- ALGORITMO 2: CONTROL LQR + ACCIÓN INTEGRAL --
       % Acumulación del error del carro (Referencia X = 0.0 m)
       Ie = Ie + (0.0 - x(1,k)) * T;
       
       % Construcción del vector de error aumentado (5x1)
       x_aug = [0.0 - x(1,k); 
                error_theta;  
                0.0 - x(3,k); 
                0.0 - x(4,k); 
                Ie];
            
       u_calc = K_lqr * x_aug;
       
       % Esquema Anti-windup para el integrador
       if (u_calc > u_max || u_calc < -u_max)
            Ie = Ie - (0.0 - x(1, k)) * T; % Deshacer integración en saturación
       end
   end
   
   % --- 4.4. Saturación física del puente en H (Voltios) ---
   if (u_calc > u_max)
       u(k) = u_max;
   elseif (u_calc < -u_max)
       u(k) = -u_max;
   else
       u(k) = u_calc;
   end
  
   % --- 4.5. Dinámica No Lineal del Mundo Real (Simulación de la Planta) ---
   M_t = [M,                     mp*lcm*cos(x(2,k));
          mp*lcm*cos(x(2,k)),    J];
      
   C_t = [cc,   -mp*lcm*x(4,k)*sin(x(2,k));
          0,     cp];
      
   G_t = [0; mp*g*lcm*sin(x(2,k))];
   
   % Conversión de la tensión de salida (V) a Fuerza física aplicada (N)
   Fuerza_Entrada = u(k) * 0.64;
   
   % Integración por método de Euler
   f1 = x(3, k);
   f2 = x(4, k);
   f34 = M_t \ ([Fuerza_Entrada; 0] - C_t*x(3:4, k) - G_t);
   
   x(:, k+1) = x(:, k) + T * [f1; f2; f34];
   t(k+1) = t(k) + T;
end
modo_activo(N) = modo_activo(N-1);

% Desglose de estados y centrado del ángulo en cero para la gráfica
dist      = x(1, :);
theta     = x(2, :); 
vel       = x(3, :);
theta_dot = x(4, :);

%% 5. GENERACIÓN DE GRÁFICAS Y EXPORTACIÓN VECTORIAL
figure('Units', 'centimeters', 'Position', [5, 5, 15, 11]);
hold on; grid on;

% Trazado de variables (Transpuestas a vectores columna para el plot)
plot(t, dist',  'LineWidth', 1.5, 'Color', [0 0.4470 0.7410]);
plot(t, theta', 'LineWidth', 1.5, 'Color', [0.8500 0.3250 0.0980]);
plot(t, u',     '--', 'LineWidth', 1.2, 'Color', [0.4660 0.6740 0.1880]);

% Configuración de etiquetas con formato LaTeX
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
ylabel('Amplitud de las Variables', 'Interpreter', 'latex');
title('\textbf{Validaci\''on de la Simulaci\''on Discreta ($T = 10$ ms)}', 'Interpreter', 'latex');

% Ubicación de la leyenda configurada en la esquina superior derecha (northeast)
legend({'Posici\''on carro $x$ (m)', '$\acute{A}$ngulo p\''endulo $\theta-\pi$ (rad)', 'Acci\''on de control $u$ (V)'}, ...
       'Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 9);
   
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);
hold off;

exportgraphics(gcf, 'TFG_Simulacion_Conmutada.pdf', 'ContentType', 'vector');