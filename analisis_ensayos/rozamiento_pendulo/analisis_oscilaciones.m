%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar 
%==========================================================================
% SCRIPT:      analisis_oscilaciones.m
% CARPETA:     /firmware_C2000/experimentos/ident_roz_pendulo/
% DESCRIPCIÓN: Procesa los datos de oscilaciones libres del péndulo para
%              aislar la ventana temporal útil y preparar la identificación
%              de los coeficientes de rozamiento y amortiguamiento.
%              
% ENTRADAS:    - oscilaciones.txt (Vectores brutos de posición y referencia)
% SALIDAS:     - ident_oscilaciones.pdf (Curva experimental para la memoria)
%==========================================================================

%% 1. INICIALIZACIÓN Y CARGA DE DATOS
clear; clc; close all;

% Cargar los datos experimentales brutos
raw_data = readmatrix('oscilaciones.txt');

% Parámetros de muestreo del microcontrolador (C2000)
Ts = 0.01; % Periodo de muestreo (s)
t = (0:size(raw_data,1)-1)' * Ts;

% Extracción de variables de telemetría
pulsos = raw_data(:,1); % Referencia (unidad digital / pulsos encoder)
theta  = raw_data(:,2); % Ángulo de salida real (rad)

%% 2. VENTANEO TEMPORAL E IDENTIFICACIÓN
% Ajuste de índices para aislar la respuesta transitoria útil
k_ini = round(17.12 / 0.01);
k_fin = round(142.21 / 0.01);

% Vectores recortados con el transitorio bajo análisis
dt     = t(k_ini:k_fin) - t(k_ini);
dtheta = theta(k_ini:k_fin);

%% 3. GENERACIÓN DE GRÁFICAS Y EXPORTACIÓN
figure('Units', 'centimeters', 'Position', [5, 5, 15, 10]); % Tamaño idóneo para hoja A4
hold on; 

% Plot de la respuesta experimental
plot(dt, dtheta, 'LineWidth', 1.2, 'Color', [0 0.4470 0.7410]); 
yline(0, '--k', 'Alpha', 0.5); 

% Configuración de etiquetas y formato (Estilo LaTeX)
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
ylabel('$\acute{A}$ngulo, $\theta$ (rad)', 'Interpreter', 'latex');
grid on;
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

% Exportación directa a PDF vectorial para la memoria
exportgraphics(gcf, 'ident_oscilaciones.pdf', 'ContentType', 'vector');