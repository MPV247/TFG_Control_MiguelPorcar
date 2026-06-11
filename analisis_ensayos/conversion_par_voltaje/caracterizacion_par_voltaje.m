%==========================================================================
% UNIVERSITAT JAUME I (UJI) - GRADO EN INTELIGENCIA ROBÓTICA
% Alumno: Miguel Porcar Vicent
% Proyecto: TFG_Control_MiguelPorcar (firmware_C2000)
%==========================================================================
% SCRIPT:      caracterizacion_par_voltaje.m
% CARPETA:     /firmware_C2000/experimentos/caracterizacion_motor/
% DESCRIPCIÓN: Determina la relación estática entre el voltaje aplicado y el
%              par generado por el motor mediante ensayos de carga con masas.
%              Compara los resultados de dos estrategias de control (P y PI)
%              frente al modelo lineal teórico derivado de Kt y R.
%              
% ENTRADAS:    - Datos medidos de masa, voltaje y desfase angular (vectores).
% SALIDAS:     - experimento_PI.pdf (Ajuste con control PI)
%              - experimento_P.pdf  (Ajuste con control P)
%              - relacion_par_voltios.pdf (Comparativa frente a modelo teórico)
%==========================================================================

%% 1. INICIALIZACIÓN Y CONFIGURACIÓN GENERAL
clear; clc; close all;

% Vector de voltajes teórico para evaluar los modelos ajustados
u_teoria = 0:0.1:15; 
g = 9.81; % Aceleración de la gravedad (m/s^2)

%% 2. ENSAYO EN BUCLE CERRADO: CONTROL DE POSICIÓN PI
l = 0.114; % Longitud de la palanca (m)

% Datos experimentales (Masas en Kg y Voltajes en V)
m_PI = [10, 20, 30, 50, 60, 100, 120, 140, 150] * 1e-3; 
u_PI = [4.1, 4.6, 6.08, 8.02, 9.00, 10.3, 12.00, 13.6, 14.3];

% Cálculo del par generado: tau = m * g * l (Nm)
tau_PI = m_PI * g * l; 

% Ajuste por mínimos cuadrados (Polinomio de grado 1: Recta)
pol_PI = polyfit(u_PI, tau_PI, 1);
tau_pred_PI = polyval(pol_PI, u_teoria);
tau_pred_PI(tau_pred_PI < 0) = 0;

% --- Gráfica Ensayo PI ---
figure('Name', 'Caracterización PI', 'Units', 'centimeters', 'Position', [2, 2, 14, 10]);
hold on; grid on;
plot(u_PI, tau_PI, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'DisplayName', 'Datos reales (PI)'); 
plot(u_teoria, tau_pred_PI, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ajuste lineal PI'); 

xlabel('Voltaje, $u$ (V)', 'Interpreter', 'latex');
ylabel('Par de fuerza, $\tau$ (Nm)', 'Interpreter', 'latex');
title('\textbf{Caracterizaci\''on Est\''atica con Control PI}', 'Interpreter', 'latex');
legend('Location', 'northwest', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

exportgraphics(gcf, 'experimento_PI.pdf', 'ContentType', 'vector');

%% 3. ENSAYO EN BUCLE CERRADO: CONTROL DE POSICIÓN P
% Datos experimentales con control P (incluye deflexión angular por error en régimen permanente)
m_P = [10, 20, 30, 50, 60, 80, 100] * 1e-3; 
u_P = [3.93, 4.79, 5.4, 7.49, 7.86, 8.78, 9.33];
theta = [-0.07, -0.11, -0.15, -0.25, -0.27, -0.31, -0.34]; % Ángulo medido (rad)

% Cálculo del par corregido por el ángulo de la palanca: tau = m * cos(theta) * g * l
tau_P = (m_P .* cos(theta)) * g * l; 

% Ajuste por mínimos cuadrados
pol_P = polyfit(u_P, tau_P, 1);
tau_pred_P = polyval(pol_P, u_teoria);
tau_pred_P(tau_pred_P < 0) = 0;

% --- Gráfica Ensayo P ---
figure('Name', 'Caracterización P', 'Units', 'centimeters', 'Position', [4, 4, 14, 10]);
hold on; grid on;
plot(u_P, tau_P, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6, 'DisplayName', 'Datos reales (P)'); 
plot(u_teoria, tau_pred_P, 'b-', 'LineWidth', 1.5, 'DisplayName', 'Ajuste lineal P'); 

xlabel('Voltaje, $u$ (V)', 'Interpreter', 'latex');
ylabel('Par de fuerza, $\tau$ (Nm)', 'Interpreter', 'latex');
title('\textbf{Caracterizaci\''on Est\''atica con Control P}', 'Interpreter', 'latex');
legend('Location', 'northwest', 'Interpreter', 'latex');
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

exportgraphics(gcf, 'experimento_P.pdf', 'ContentType', 'vector');

%% 4. COMPARATIVA: MODELOS EXPERIMENTALES VS. MODELO TEÓRICO ELEC/MEC
% Parámetros nominales del motor DC de la planta
Kt = 0.068; % Constante de par (Nm/A)
R  = 5.5;   % Resistencia eléctrica del devanado del rotor (Ohmios)

% Pendiente teórica ideal en régimen permanente (bloqueado): tau = (Kt/R) * V
slope_teorica = Kt / R;  
tau_teorico = slope_teorica * u_teoria; 

% --- Gráfica Comparativa Final ---
figure('Name', 'Comparativa Modelos', 'Units', 'centimeters', 'Position', [6, 6, 15, 11]);
hold on; grid on;

plot(u_teoria, tau_pred_PI, 'Color', [0 0.4470 0.7410], 'LineWidth', 1.5); 
plot(u_teoria, tau_pred_P,  'Color', [0.8500 0.3250 0.0980], 'LineWidth', 1.5); 
plot(u_teoria, tau_teorico, 'k--', 'LineWidth', 1.5); 

xlabel('Acci\''on de control, $u$ (V)', 'Interpreter', 'latex'); 
ylabel('Par de fuerza, $\tau$ (Nm)', 'Interpreter', 'latex'); 
title('\textbf{Contraste de Modelos: Experimental vs. Te\''orico}', 'Interpreter', 'latex'); 

legend({'Ajuste con datos PI', 'Ajuste con datos P', 'Modelo te\''orico ($\frac{K_t}{R} \cdot u$)'}, ...
       'Location', 'northwest', 'Interpreter', 'latex'); 
set(gca, 'TickLabelInterpreter', 'latex', 'FontSize', 11);

exportgraphics(gcf, 'relacion_par_voltios.pdf', 'ContentType', 'vector');