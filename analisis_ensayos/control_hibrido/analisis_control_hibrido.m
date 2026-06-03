%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      analisis_control_hibrido.m
% DESCRIPCIÓN: Carga y visualiza los datos experimentales del control
%              híbrido. Muestra la evolución temporal de la posición y el 
%              ángulo medido (con inversión de signo) e identifica las 
%              fases de control mediante sombreados de fondo.
%              
% ENTRADAS:    - control_hibrido.txt
% SALIDAS:     - Hibrido_Energia.pdf
%              - Hibrido_Secuencia_Swingup.pdf
%==========================================================================

%% 1. INICIALIZACIÓN Y CARGA DE DATOS
clear; clc; close all;

% Archivo de datos
filename = 'control_hibrido.txt';
data = readmatrix(filename);

% Parámetros temporales
Ts = 0.01; 
t = (0:size(data,1)-1)' * Ts;

% Extracción de variables según la estructura del archivo
x_pos       = -data(:,1);
ref_x       = data(:,2); 
th          = -data(:,3); % Inversión de signo validada para consistencia física
ref_th      = data(:,4);
E_m         = data(:,8);
E_ref       = data(:,9);
tipo_ctrl   = data(:,10); % 0 = Swing-up, 1 = Estabilización (LQR)

% Límites temporales solicitados
t_inicio = 10;
t_fin = t(end); 

% Detección dinámica del instante de conmutación (primer '1' tras t_inicio)
idx_conm = find(tipo_ctrl == 1 & t >= t_inicio, 1, 'first');
if ~isempty(idx_conm)
    t_conmutacion = t(idx_conm);
else
    t_conmutacion = t_fin; 
end

%% 2. GRÁFICA 1: ENERGÍA DE REFERENCIA VS MEDIDA (Solo durante Swing-up)
fig1 = figure('Name', 'Energía durante Swing-up', ...
              'NumberTitle', 'off', ...
              'Units', 'centimeters', ...
              'Position', [3, 12, 17, 9]);
hold on; grid on;

idx_fase_energia = (t >= t_inicio) & (t <= t_conmutacion);
plot(t(idx_fase_energia), E_ref(idx_fase_energia), 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $E_{ref}$');
plot(t(idx_fase_energia), E_m(idx_fase_energia), 'b-', 'LineWidth', 1.2, 'DisplayName', 'Medida $E_m$');

ylabel('Energ\''ia (J)', 'Interpreter', 'latex');
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Seguimiento de Energ\''ia durante la Fase de Swing-up}', 'Interpreter', 'latex');
legend('Location', 'northeast', 'Interpreter', 'latex');
xlim([t_inicio, t_conmutacion]); 
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

%% 3. GRÁFICA 2: SECUENCIA COMPLETA (Con Sombreado de Control de Fondo)
fig2 = figure('Name', 'Secuencia de Swing-up y Conmutación', ...
              'NumberTitle', 'off', ...
              'Units', 'centimeters', ...
              'Position', [3, 2, 17, 15]); % Proporción optimizada de la plantilla

% --- Subplot 1: Posición del Carro ---
ax1 = subplot(2,1,1); hold on; grid on;
plot(t, ref_x, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_x$');
plot(t, x_pos, 'r-', 'LineWidth', 1.2, 'DisplayName', '$x$ medido');
ylabel('Posici\''on, $x$ (m)', 'Interpreter', 'latex');
title('\textbf{Secuencia Temporal: Posici\''on del Carro}', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax1, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

% --- Subplot 2: Ángulo del Péndulo ---
ax2 = subplot(2,1,2); hold on; grid on;
plot(t, ref_th, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_{\theta}$');
plot(t, th, 'b-', 'LineWidth', 1.2, 'DisplayName', '$\theta$ medido');
ylabel('$\acute{A}$ngulo, $\theta$ (rad)', 'Interpreter', 'latex');
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Secuencia Temporal: $\acute{A}$ngulo del P\''endulo}', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax2, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

%% 4. APLICACIÓN AUTOMÁTICA DEL FONDO DE CONMUTACIÓN
% Recorremos ambos subplots para inyectar los parches de color en el fondo
ejes = [ax1, ax2];
for j = 1:length(ejes)
    ax = ejes(j);
    subplot(ax);
    
    % Recuperamos de forma exacta los límites del eje Y para cubrir todo el fondo
    ylims = ylim(ax);
    
    % Fase 1: Swing-up -> Fondo Azul Tenue
    p1 = patch([t_inicio, t_conmutacion, t_conmutacion, t_inicio], ...
               [ylims(1), ylims(1), ylims(2), ylims(2)], ...
               [0.92, 0.95, 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
               'DisplayName', 'Fase: Swing-up');
           
    % Fase 2: Estabilización -> Fondo Verde Tenue
    p2 = patch([t_conmutacion, t_fin, t_fin, t_conmutacion], ...
               [ylims(1), ylims(1), ylims(2), ylims(2)], ...
               [0.92, 0.97, 0.92], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
               'DisplayName', 'Fase: Estabilizaci\''on');
    
    % Línea vertical muy fina que delimita el instante de cambio
    xline(t_conmutacion, 'g-.', 'LineWidth', 1.2, 'HandleVisibility', 'off');
    
    % IMPORTANTE: Enviamos los parches al fondo absolute para que las curvas 
    % de datos y las líneas de la cuadrícula (grid) queden por encima.
    uistack(p1, 'bottom');
    uistack(p2, 'bottom');
    
    % Renderizar la leyenda con el formateador LaTeX de la plantilla
    legend('Location', 'northeastoutside', 'Interpreter', 'latex');
end

% Enlazar ejes X para que el zoom interactivo funcione sincronizado
linkaxes([ax1, ax2], 'x');

%% 5. EXPORTACIÓN EN FORMATO VECTORIAL
exportgraphics(fig1, 'Hibrido_Energia.pdf', 'ContentType', 'vector');
exportgraphics(fig2, 'Hibrido_Secuencia_Swingup.pdf', 'ContentType', 'vector');

fprintf('Procesamiento finalizado. Conmutación detectada en t = %.2f s.\n', t_conmutacion);