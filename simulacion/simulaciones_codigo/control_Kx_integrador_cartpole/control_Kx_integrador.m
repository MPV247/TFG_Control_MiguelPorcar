%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (simulacion)
%==========================================================================
% SCRIPT:      control_Kx_integrador.m
% CARPETA:     /simulacion/simulaciones_codigo/control_Kx_integrador_cartpole
% DESCRIPCIÓN: Define los parámetros físicos del péndulo invertido, obtiene
%              el modelo linealizado en espacio de estados (SS) con el vector
%              de estados expandido para tener acción integral, sintoniza
%              los controladores (LQR) y valida la
%              respuesta ante condiciones iniciales mediante un bucle de
%              simulación discreto no lineal con saturación de actuador.
%              
% ENTRADAS:    - Parámetros dinámicos teóricos/identificados del sistema.
% SALIDAS:     - control_Kx_integrador_T_XXms.pdf (Validación temporal del controlador)
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

%% 2. OBTENCIÓN DE LA REPRESENTACIÓN EN ESPACIO DE ESTADOS (SS)
% Matrices inercial (M_bar), amortiguamiento (C_bar) y gravitatoria (G_bar)

M_bar = [M,          -mp*lcm;
        -mp*lcm,      J]; 

C_bar = [cc,           0;
          0,          cp       ]; 

G_bar= [  0              0; 
          0,         -mp*g*lcm]; 

% Construcción de la matriz dinámica A (4x4) y matriz de entrada B (4x1)
A= [zeros(2),        eye(2);
    -M_bar\G_bar ,-M_bar\C_bar];

B = [0;0;[M_bar\[1;0]]];

% Conversión de entrada: Relación estática lineal (de Voltios a Newtons)
B_volt = 0.64*B; 


C = eye(4); % Matriz de salida: Monitorización completa del vector de estados
D = 0;      % Matriz de paso directo

%% 3. EXPANDIR EL VECTOR DE ESTADOS PARA AÑADIR ACCIÓN INTEGRAL
% Seleccionamos solo la primera fila de C (que corresponde a la posición x)
C_x = [1 0 0 0]; 

Ae = [A,         zeros(4,1); 
     C_x,       0];

Be = [B_volt; 0];


%% 4. SINTONÍA DE CONTROLADORES (LQR)
%--- Optimizador Lineal Cuadrático (LQR) ---
Q = diag([5, 1, 0.7, 0.1, 3]); 
r = 0.001;                
R = r * eye(1);       
[K_lqr, S, E] = lqr(Ae, Be, Q, R);


%% 5. SIMULACIÓN TEMPORAL (MÉTODO RECURSIVO NO LINEAL)

N=600;            % Número de iteraciones de la simulación
T = 10e-3;        % Periodo de muestreo/integración (s)

% Inicialización de matrices de estado y control
x=zeros(4,N);
x(2,1)=3.05;           % Condición inicial del ángulo (cercano a pi rad)

u=zeros(1,N);
t=zeros(1,N);

u_max = 4.68;           % Saturacion de la acción de control (V)
ref = [0, 3.14, 0, 0];  % Punto de operación inestable (Péndulo vertical hacia arriba)
Ie = 0.0;               % Acumulación del error en el eje X

for k=1:N-1
   % Acumulación del error integral (Exclusivo para la posición del carro 'x')
   Ie = Ie + (ref(1) - x(1,k)) * T;
   
   % Construcción del vector de estado aumentado (Dimensión 5x1)
   x_aug = [ref' - (x(:,k)); Ie];
   
   % Calculo de la acción de control con las 5 ganancias de K_lqr
   u_calc = K_lqr * x_aug;

   % Gestión de la Saturación y Estrategia Anti-windup
   if (u_calc > u_max || u_calc <-u_max)
        %Anti-windup: Deshacer la integración para evitar la saturación del término integral
        Ie = Ie - (ref(1) - x(1, k)) * T;
        x_aug = [(ref' - x(:, k)); Ie];    % Reajuste del estado aumentado

        % Aplicación de la saturación
        if (u_calc > u_max)
            u(k) = u_max;
        elseif (u_calc < -u_max)
            u(k) = -u_max; 
        end
   else
        u(k) = u_calc;
   end
  
   % Dinámica No Lineal ("Realidad Física del Sistema")

   % Matriz de inercia
   M_t = [M,          +mp*lcm*cos(x(2,k));
          +mp*lcm*cos(x(2,k)),      J];

   % Matriz de fuerzas de rozamiento, centrífugas y de Coriolis
   C_t = [cc,   -mp*lcm*x(4,k)*sin(x(2,k))
           0,          cp        ];

   % Vector de pares y fuerzas gravitatorias
   G_t = [0; +mp*g*lcm*sin(x(2,k))];

   % Integración Numérica del Vector de Estados
   f1=x(3, k);   % Velocidad lineal del carro (x_dot)
   f2 = x(4, k); % Velocidad angular del péndulo (theta_dot)
   f34=M_t\([u(k);0]-C_t*x(3:4,k)-G_t); %Aceleraciones

   %Cálculo del valor de los estados en la siguiente iteración.
   x(:,k+1)=x(:,k)+T*[f1;f2;f34];
   t(k+1)=t(k)+T;
end

% Desglose de estados y centrado del ángulo en cero para la gráfica
dist      = x(1, :);
theta     = x(2, :) - pi; 
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

%% Exportación directa a PDF vectorial para la memoria
exportgraphics(gcf, 'control_Kx_integrador_T_10ms.pdf', 'ContentType', 'vector');
