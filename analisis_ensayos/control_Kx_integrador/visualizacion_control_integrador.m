%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      visualizacion_control_integrador.m
% CARPETA:     /firmware_C2000/experimentos/control_Kx_integrador_cartpole/
% DESCRIPCIÓN: Analiza y visualiza la respuesta temporal del controlador
%              definitivo Kz aumentativo (con acción integral). Evalúa la
%              capacidad del bucle cerrado para eliminar el error en régimen
%              permanente ante cambios de referencia en la posición del carro.
%              
% ENTRADAS:    - control_con_integrador.txt (Datos de telemetría)
% SALIDAS:     - control_con_integrador.pdf (Gráfica de doble eje)
%==========================================================================

%% 1. INICIALIZACIÓN Y CARGA DE DATOS
clear; clc; close all;

% Cargar los datos experimentales brutos
raw_data = readmatrix('control_con_integrador.txt');

% Parámetros del experimento
Ts = 0.01; % Periodo de muestreo (s)
t = (0:size(raw_data,1)-1)' * Ts;

% Extracción de columnas según formato CSV del firmware:
% [x, ref_x, theta, ref_theta, x_dot, theta_dot, u]
x         = raw_data(:,1); % Posición real del carro (m)
ref_x     = raw_data(:,2); % Referencia de posición del carro (m)
theta     = raw_data(:,3); % Ángulo real del péndulo (rad)
ref_theta = raw_data(:,4); % Referencia del ángulo (rad)
x_dot     = raw_data(:,5); % Velocidad del carro (m/s)
theta_dot = raw_data(:,6); % Velocidad angular (rad/s)
u         = raw_data(:,7); % Acción de control enviada (V)

%% 2. GENERACIÓN DE GRÁFICA CON DOBLE EJE Y (MÉTRICA PROFESIONAL)
figure('Units', 'centimeters', 'Position', [5, 5, 16, 11]);
hold on;

% --- EJE IZQUIERDO: Posición del carro (Metros) ---
yyaxis left
plot(t, ref_x, '--', 'Color', [0.5 0.5 0.5], 'LineWidth', 1.2, 'DisplayName', 'Ref. Carro $r_x$');
plot(t, x, '-', 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5, 'DisplayName', 'Posici\''on $x$');
ylabel('Posici\''on del Carro (m)', 'Interpreter', 'latex');
ax = gca;
ax.YColor = [0 0.4470 0.7410]; % Sintoniza el color del eje con la variable real

% --- EJE DERECHO: Ángulo del péndulo (Radianes) ---
yyaxis right
plot(t, ref_theta, ':', 'Color', [0.2 0.2 0.2], 'LineWidth', 1.2, 'DisplayName', 'Ref. P\''endulo $r_{\theta}$');
plot(t, theta, '-', 'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.3, 'DisplayName', '$\acute{A}$ngulo $\theta$');
ylabel('$\acute{A}$ngulo del P\''endulo (rad)', 'Interpreter', 'latex');
ax.YColor = [0.8500 0.3250 0.0980]; % Sintoniza el color del eje con la variable real

%% 3. CONFIGURACIÓN ESTÉTICA Y FORMATO LATEX
grid on;
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Respuesta del Controlador con Integrador}', 'Interpreter', 'latex');

% Ajuste de límites temporales de visualización
xlim([10, 40]);

% Forzar a que los números de ambos ejes usen la tipografía tipográfica de LaTeX
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

% Añadir leyenda interactiva configurada con intérprete LaTeX
legend('Location', 'northeast', 'Interpreter', 'latex', 'FontSize', 9);

hold off;

%% 4. EXPORTACIÓN EN FORMATO VECTORIAL PARA LA MEMORIA
exportgraphics(gcf, 'control_con_integrador.pdf', 'ContentType', 'vector');