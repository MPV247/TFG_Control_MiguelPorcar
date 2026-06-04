%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      comparar_swingup_ke_final.m
% CARPETA:     /firmware_C2000/experimentos/comparativa_controles/
% DESCRIPCIÓN: Compara el efecto de la ganancia Ke (20 a 100) en el Swing-Up.
%              Gráfica desde t = 10s hasta el punto de conmutación.
%==========================================================================
%% 1. CONFIGURACIÓN DE ARCHIVOS Y LEYENDAS
clear; clc; close all;

% Archivos seleccionados 
files = {
    'swp_ke_20.txt', ...
    'swp_ke_40.txt', ...
    'swp_ke_60.txt', ...
    'swp_ke_80.txt', ...
    'swp_ke_100.txt'
};

labels = {
    '$K_e = 20$ (C1)', ...
    '$K_e = 40$ (C2)', ...
    '$K_e = 60$ (C3)', ...
    '$K_e = 80$ (C4)', ...
    '$K_e = 100$ (C5)'
};

% Paleta de colores para la comparativa
colors = {
    [0.40 0.65 0.85], ... % Azul claro
    [0.15 0.45 0.75], ... % Azul oscuro
    [0.95 0.75 0.40], ... % Ámbar
    [0.85 0.45 0.15], ... % Naranja
    [0.70 0.10 0.10]      % Rojo
};

Ts = 0.01; 
t_inicio = 10;        % Inicio de visualización
t_conmutacion_max = 0; % Para ajuste de ejes

%% 2. INICIALIZACIÓN DE LA FIGURA
figure('Name', 'Análisis Swing-Up: Variación de Inyección Energética', ...
       'NumberTitle', 'off', ...
       'Units', 'centimeters', ...
       'Position', [2, 2, 17, 15]); 

ax1 = subplot(2,1,1); hold on; grid on;
ax2 = subplot(2,1,2); hold on; grid on;

%% 3. PROCESAMIENTO DE DATOS
for i = 1:length(files)
    if ~exist(files{i}, 'file')
        continue;
    end
    
    data = readmatrix(files{i});
    t_raw = (0:size(data,1)-1)' * Ts;
    
    x_pos   = data(:,1);
    th_raw  = data(:,3);
    flag_sp = data(:,10); % Columna 10: 0=Swing-up, 1=Control vertical
    
    % Encontrar índices relevantes
    idx_start = find(t_raw >= t_inicio, 1, 'first');
    idx_conmutacion = find(flag_sp == 1, 1, 'first');
    
    if isempty(idx_conmutacion), idx_conmutacion = length(t_raw); end
    if isempty(idx_start), idx_start = 1; end
    
    t_conm_actual = t_raw(idx_conmutacion);
    if t_conm_actual > t_conmutacion_max, t_conmutacion_max = t_conm_actual; end
    
    % Rango de datos
    rango = idx_start:idx_conmutacion;
    t_plot = t_raw(rango);
    th_unwrap = unwrap(th_raw(rango)); % Evita saltos de 2pi
    
    % --- Subplot 1: Posición ---
    subplot(ax1);
    plot(t_plot, x_pos(rango), 'LineWidth', 1.3, 'Color', colors{i}, 'DisplayName', labels{i});
    plot(t_plot(end), x_pos(idx_conmutacion), 'o', 'MarkerFaceColor', colors{i}, 'Color', colors{i}, 'HandleVisibility', 'off');
    
    % --- Subplot 2: Ángulo ---
    subplot(ax2);
    plot(t_plot, th_unwrap, 'LineWidth', 1.3, 'Color', colors{i}, 'DisplayName', labels{i});
    plot(t_plot(end), th_unwrap(end), 'o', 'MarkerFaceColor', colors{i}, 'Color', colors{i}, 'HandleVisibility', 'off');
end

%% 4. ESTÉTICA Y EXPORTACIÓN
t_fin = t_conmutacion_max + 0.2;

% Formateo Eje Superior
subplot(ax1);
ylabel('Posici\''on, $x$ (m)', 'Interpreter', 'latex');
title('\textbf{Desplazamiento del Carro durante la Inyecci\''on de Energ\''ia}', 'Interpreter', 'latex');
legend('Location', 'northeastoutside', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax1, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

% Formateo Eje Inferior
subplot(ax2);
ylabel('$\acute{A}$ngulo desenvuelto, $\theta$ (rad)', 'Interpreter', 'latex');
xlabel('Tiempo absoluto, $t$ (s)', 'Interpreter', 'latex');
title('\textbf{Evoluci\''on del Balanceo hasta Conmutaci\''on}', 'Interpreter', 'latex');
legend('Location', 'northeastoutside', 'Interpreter', 'latex');
xlim([t_inicio, t_fin]);
set(ax2, 'TickLabelInterpreter', 'latex', 'FontSize', 10);

% Guardar para la memoria
exportgraphics(gcf, 'Comparativa_SwingUp_Ke_Final.pdf', 'ContentType', 'vector');
disp('Gráfica generada: Comparativa_SwingUp_Ke_Final.pdf');