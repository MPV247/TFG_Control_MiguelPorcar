%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      identificacion_BC_carro.m
% CARPETA:     /firmware_C2000/experimentos/ident_carro_lineal/
% DESCRIPCIÓN: Procesa los datos del carro en bucle cerrado (BC) bajo un
%              control proporcional (KP=80). Aísla el transitorio útil y
%              reconstruye de forma analítica la función de transferencia
%              de la planta en bucle abierto (BA).
%              
% ENTRADAS:    - Ident_carro_KP80.txt (Telemetría de la prueba en BC)
% SALIDAS:     - respuesta_BC_KP80.pdf (Gráfica de validación temporal)
%==========================================================================

%% 1. INICIALIZACIÓN Y CARGA DE DATOS
clear; clc; close all;

% Cargar los datos experimentales brutos
raw_data = readmatrix('Ident_carro_KP80.txt');

% Parámetros de muestreo del microcontrolador (C2000)
Ts = 0.01; % Periodo de muestreo (s)
t = (0:size(raw_data,1)-1)' * Ts;

% Extracción de variables de telemetría
ref   = raw_data(:,1); % Referencia de posición (m)
x     = raw_data(:,2); % Posición real del carro (m)
x_dot = raw_data(:,3); % Velocidad lineal del carro (m/s)
u     = raw_data(:,4); % Acción de control lineal calculada (V)
u_zm  = raw_data(:,5); % Acción de control final con zona muerta (V)

%% 2. VENTANEO TEMPORAL DE LOS VECTORES
% Ajuste de índices para aislar la ventana temporal bajo análisis
k_ini = round(9 / 0.01);
k_fin = round(15 / 0.01);

% Recorte de señales restando el valor inicial para eliminar offsets
dt     = t(k_ini:k_fin) - t(k_ini);
dr     = ref(k_ini:k_fin) - ref(k_ini);
du     = u(k_ini:k_fin) - u(k_ini);
dx     = x(k_ini:k_fin) - x(k_ini);
dx_dot = x_dot(k_ini:k_fin) - x_dot(k_ini);

%% 3. GENERACIÓN DE GRÁFICAS Y EXPORTACIÓN
figure('Units', 'centimeters', 'Position', [5, 5, 15, 10]); % Tamaño idóneo A4
hold on; 

% Plot de señales de referencia y salida real
plot(dt, dr, '--k', 'LineWidth', 1.0);
plot(dt, dx, 'LineWidth', 1.3, 'Color', [0 0.4470 0.7410]); 

% Configuración estética de la gráfica (Estilo LaTeX)
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
ylabel('Posici\''on, $x$ (m)', 'Interpreter', 'latex');
legend({'Ref. $r_x$ (m)', 'Carro $x$ (m)'}, 'Location', 'northeast', 'Interpreter', 'latex');
grid on;
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

% Exportación directa a PDF vectorial para la memoria
exportgraphics(gcf, 'respuesta_BC_KP80.pdf', 'ContentType', 'vector');

%% 4. OBTENCIÓN DE LA PLANTA EN BUCLE ABIERTO (BA)
% Definición de la variable compleja de Laplace
s = tf('s');

% Parámetros identificados del modelo complementario de sensibilidad M(s)
xi = 0.4; 
K = 1.0; 
wn = 1 / 0.14; 

% Modelo del sistema en Bucle Cerrado (BC)
M = (K * wn^2) / (s^2 + 2 * xi * wn * s + wn^2);  

% Ganancia Proporcional utilizada en el ensayo de hardware
KP = 80;

% Despeje algebraico para aislar la planta en Bucle Abierto: G = M / (KP * (1 - M))
% Se utiliza minreal para cancelar polos y ceros redundantes en el origen
G_planta = minreal(M / KP / (1 - M));

% Mostrar el modelo resultante en formato de Ceros, Polos y Ganancia (ZPK)
disp('--- Modelo de la planta en Bucle Abierto G(s) ---');
zpk(G_planta)