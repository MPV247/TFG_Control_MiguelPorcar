%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      analisis_control_hibrido.m
% DESCRIPCIÓN: Carga y visualiza los datos experimentales del control
%              híbrido. Optimizado para mostrar el crecimiento continuo 
%              del ángulo hacia +pi y diferenciar fases mediante fondos
%              sombreados en lugar de subplots adicionales.
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
x_pos       = data(:,1);
ref_x       = data(:,2); 
th          = data(:,3);
ref_th      = data(:,4);
E_m         = data(:,8);
E_ref       = data(:,9);
tipo_ctrl   = data(:,10); % 0 = Swing-up, 1 = Estabilización

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

%% 3. GRÁFICA 2: SECUENCIA COMPLETA (Diseño Compacto y Visual)
fig2 = figure('Name', 'Secuencia de Swing-up y Conmutación', ...
              'NumberTitle', 'off', ...
              'Units', 'centimeters', ...
              'Position', [3, 2, 17, 15]); % Proporción idéntica a tu plantilla original

% --- TRATAMIENTO DEL ÁNGULO PARA CRECIMIENTO CONTINUO HACIA +PI ---
% Corregimos el desfase sumando 2*pi a los valores que oscilan por el lado negativo
% para que rodeen a +pi limpiamente y se estabilicen junto a la referencia.
th_ajustado = th;
th_ajustado(th_ajustado < -pi/2) = th_ajustado(th_ajustado < -pi/2) + 2*pi;

% --- Subplot 1: Posición del Carro ---
ax1 = subplot(2,1,1); hold on; grid on;
plot(t, ref_x, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_x$');
plot(t, x_pos, 'r-', 'LineWidth', 1.2, 'DisplayName', '$x$ actual');
ylabel('Posici\''on, $x$ (m)', 'Interpreter', 'latex');
title('\textbf{Secuencia H\''ibrida: Posici\''on del Carro}', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax1, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

% --- Subplot 2: Ángulo del Péndulo ---
ax2 = subplot(2,1,2); hold on; grid on;
plot(t, ref_th, 'k--', 'LineWidth', 1.5, 'DisplayName', 'Ref. $r_{\theta}$');
plot(t, th_ajustado, 'b-', 'LineWidth', 1.2, 'DisplayName', '$\theta$ ajustado');
ylabel('$\acute{A}$ngulo, $\theta$ (rad)', 'Interpreter', 'latex');
xlabel('Tiempo, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Secuencia H\''ibrida: $\acute{A}$ngulo del P\''endulo}', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax2, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

%% 4. APLICACIÓN DE SOMBREADO DE FASES (SWING-UP VS ESTABILIZACIÓN)
% Aplicamos el formateado de fondos de manera síncrona en ambos ejes
ejes = [ax1, ax2];
for j = 1:length(ejes)
    ax = ejes(j);
    subplot(ax);
    
    % Capturamos los límites del eje Y actual para ajustar el tamaño del fondo
    ylims = ylim(ax);
    
    % Sombreado Zona 1: Swing-up (Azul muy tenue)
    p1 = patch([t_inicio, t_conmutacion, t_conmutacion, t_inicio], ...
               [ylims(1), ylims(1), ylims(2), ylims(2)], ...
               [0.90, 0.94, 0.98], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
               'DisplayName', 'Fase: Swing-up');
           
    % Sombreado Zona 2: Estabilización LQR (Verde muy tenue)
    p2 = patch([t_conmutacion, t_fin, t_fin, t_conmutacion], ...
               [ylims(1), ylims(1), ylims(2), ylims(2)], ...
               [0.91, 0.97, 0.91], 'EdgeColor', 'none', 'FaceAlpha', 0.6, ...
               'DisplayName', 'Fase: Estabilizaci\''on');
    
    % Línea vertical discontinua que marca el hito de la conmutación
    xline(t_conmutacion, 'g-.', 'LineWidth', 1.5, 'HandleVisibility', 'off');
    
    % CRUCIAL: Enviar los parches al fondo para que no oculten las líneas de datos
    uistack(p1, 'bottom');
    uistack(p2, 'bottom');
    
    % Forzar la actualización de la leyenda incluyendo las etiquetas de las fases
    legend('Location', 'northeastoutside', 'Interpreter', 'latex');
end

% Enlazar ejes X para navegación interactiva en MATLAB
linkaxes([ax1, ax2], 'x');

%% 5. EXPORTACIÓN EN FORMATO VECTORIAL
exportgraphics(fig1, 'Hibrido_Energia.pdf', 'ContentType', 'vector');
exportgraphics(fig2, 'Hibrido_Secuencia_Swingup.pdf', 'ContentType', 'vector');

fprintf('Procesamiento finalizado. Conmutación detectada en t = %.2f s.\n', t_conmutacion);