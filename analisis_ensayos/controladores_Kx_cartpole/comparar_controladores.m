%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      comparar_controladores.m
% CARPETA:     /firmware_C2000/experimentos/comparativa_controles/
% DESCRIPCIÓN: Carga y superpone los datos experimentales de 6 sintonías
%              diferentes del controlador (variando matrices de peso Q y R).
%              Evalúa la respuesta transitoria tanto en la posición del 
%              carro como en la estabilización angular del péndulo.
%              
% ENTRADAS:    - 6x Archivos .txt con telemetría de cambios de referencia.
% SALIDAS:     - Comparativa_Controladores.pdf (Gráfica compuesta para la memoria)
%==========================================================================

%% 1. CONFIGURACIÓN DE ARCHIVOS Y LEYENDAS
clear; clc; close all;

% Listado de archivos de datos experimentales a contrastar
files = {
    'control1_Q_Ident_R_0_1_cambio_ref.txt', ...
    'control1_Q_Ident_R_0_01_cambio_ref.txt', ...
    'control1_Q_Ident_R_0_001_cambio_ref.txt', ...
    'control1_Q_2111_R_0_001_cambio_ref.txt', ...
    'control1_Q_10111_R_0_001_cambio_ref.txt', ...
    'control1_Q_510701_R_0_001_cambio_ref.txt'
};

labels = {'C1', 'C2', 'C3', ...
          'C4', 'C5', 'C6'};

% Parámetros temporales generales
Ts = 0.01; 
t_inicio = 10;
t_fin = 40;

%% 2. INICIALIZACIÓN DE LA FIGURA MULTIPLOT
figure('Name', 'Comparativa de Controladores', ...
       'NumberTitle', 'off', ...
       'Units', 'centimeters', ...
       'Position', [3, 3, 17, 15]); % Proporción optimizada para subplots apilados

% Inicialización y reserva de los ejes para evitar llamadas redundantes
ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;

%% 3. BUCLE DE PROCESAMIENTO Y VISUALIZACIÓN
for i = 1:length(files)
    % Carga de datos locales
    data = readmatrix(files{i});
    t = (0:size(data,1)-1)' * Ts;
    
    % Extracción de variables indexadas
    x_pos  = data(:,1);
    ref_x  = data(:,2); 
    th     = data(:,3);
    ref_th = data(:,4);
    
    % --- Gráfica de Posición del Carro (Subplot Superior) ---
    subplot(ax1);
    if i == 1 
        plot(t, ref_x, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_x$');
    end
    plot(t, x_pos, 'LineWidth', 1.2, 'DisplayName', labels{i});
    
    % --- Gráfica de Ángulo del Péndulo (Subplot Inferior) ---
    subplot(ax2);
    if i == 1 
        plot(t, ref_th, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_{\theta}$');
    end
    plot(t, th, 'LineWidth', 1.2, 'DisplayName', labels{i});
end

%% 4. AJUSTES ESTÉTICOS Y FORMATEADO LATEX

% Formateado del Subplot 1 (Posición)
subplot(ax1);
ylabel('Posici\''on, $x$ (m)', 'Interpreter', 'latex');
title('\textbf{Comparativa Temporal: Posici\''on del Carro}', 'Interpreter', 'latex');
legend('Location', 'northeastoutside', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax1, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

% Formateado del Subplot 2 (Ángulo)
subplot(ax2);
ylabel('$\acute{A}$ngulo, $\theta$ (rad)', 'Interpreter', 'latex');
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Comparativa Temporal: $\acute{A}$ngulo del P\''endulo}', 'Interpreter', 'latex');
legend('Location', 'northeastoutside', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax2, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

%% 5. EXPORTACIÓN EN FORMATO VECTORIAL
exportgraphics(gcf, 'Comparativa_Controladores.pdf', 'ContentType', 'vector');