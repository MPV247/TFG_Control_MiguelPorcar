%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      comparativa_frecuencias_pwm.m
% CARPETA:     /firmware_C2000/experimentos/zona_muerta_motor/
% DESCRIPCIÓN: Analiza el impacto de la frecuencia de conmutación de la PWM
%              (desde 200 Hz hasta 10 kHz) sobre la zona muerta del motor.
%              Superpone las curvas estáticas de velocidad vs. acción de 
%              control para evaluar pérdidas por conmutación y linealidad.
%              
% ENTRADAS:    - 5x Archivos .txt con ensayos en rampa a diferentes frecuencias.
% SALIDAS:     - comparativa_frecuencias_pwm.pdf (Gráfica comparativa vectorial)
%==========================================================================

%% 1. CONFIGURACIÓN DE ARCHIVOS, ETIQUETAS Y PALETA DE COLORES
clc, clear; close all;

% Listado de archivos de telemetría experimental
archivos = {'zona_muerta_200Hz.txt', ...
            'zona_muerta_1KHz.txt', ...
            'zona_muerta_2_5KHz.txt', ...
            'zona_muerta_5KHz.txt', ...
            'zona_muerta_10KHz.txt'};

etiquetas = {'200 Hz', '1 kHz', '2.5 kHz', '5 kHz', '10 kHz'};

% Paleta académica de colores desaturados (Estilo MATLAB professional)
colores = [0.0000  0.4470  0.7410;   % Azul
           0.8500  0.3250  0.0980;   % Naranja
           0.4660  0.6740  0.1880;   % Verde
           0.4940  0.1840  0.5560;   % Morado
           0.9290  0.6940  0.1250];  % Dorado/Amarillo

%% 2. INICIALIZACIÓN DE LA FIGURA
figure('Color', 'w', ...
       'Name', 'Comparativa de Frecuencias PWM', ...
       'Units', 'centimeters', ...
       'Position', [5, 5, 15, 11]); % Tamaño idóneo para incorporar en documento A4
hold on; 

%% 3. BUCLE DE PROCESAMIENTO Y FILTRADO DE DATOS
for i = 1:length(archivos)
    % Cargar matriz de datos experimentales
    data = readmatrix(archivos{i});
    
    % Filtrar el efecto del frenado dinámico (PWM == 0 con velocidad residual alta)
    data = data(~(data(:, 1) == 0 & abs(data(:, 2)) > 1.0), :);
    
    % Extraer vectores de entrada y salida
    pwm = data(:, 1); 
    w   = data(:, 2);
    
    % Ordenación de vectores según la acción de control para evitar artefactos en el trazado
    [pwm_sort, idx] = sort(pwm); 
    w_sort = w(idx); 
    
    % Graficado de la curva de cada frecuencia
    plot(pwm_sort, w_sort, '-', 'LineWidth', 1.3, ...
         'Color', colores(i,:), 'DisplayName', etiquetas{i});
end

%% 4. FORMATEADO ESTÉTICO Y EJES DE REFERENCIA (ESTILO LaTeX)
grid on;

% Etiquetas de los ejes e intérprete matemático
xlabel('Acci\''on de Control (PWM)', 'Interpreter', 'latex');
ylabel('Velocidad Angular, $\omega$ (rad/s)', 'Interpreter', 'latex');
title('\textbf{Efecto de la Frecuencia PWM en la Zona Muerta}', 'Interpreter', 'latex');

% Líneas de referencia de los ejes cartesianos centrales
xline(0, '--k', 'LineWidth', 0.8, 'Alpha', 0.4, 'HandleVisibility', 'off'); 
yline(0, '--k', 'LineWidth', 0.8, 'Alpha', 0.4, 'HandleVisibility', 'off'); 

% Configuración del formato de los números de los ejes (Ticks)
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

% Configuración de la leyenda en la zona libre de datos
legend('Location', 'best', 'Interpreter', 'latex', 'FontSize', 10);
hold off;

%% 5. EXPORTACIÓN EN FORMATO VECTORIAL PARA LA MEMORIA
exportgraphics(gcf, 'comparativa_frecuencias_pwm.pdf', 'ContentType', 'vector');