//#############################################################################
//
// FILE:        control.c
//
// TITLE:       TFG - Control Swing-Up por Linealización por Realimentación Parcial
//
// AUTHOR:      Miguel Porcar Vicent
// DATE:        Mayo 2026
// TARGET:      TI C2000 (TMS320F28004x)
//
// DESCRIPCIÓN:
// Este módulo ejecuta un algoritmo de control no lineal basado en el bombeo de
// energía acoplado a una Ley de Linealización por Realimentación Parcial (PFL)
// de tipo colocada para balancear el péndulo desde su posición estable inferior
// (theta = 0) hacia la vertical inestable (theta = pi). 
// Funciona en tiempo real a una frecuencia de muestreo de 100 Hz (T = 10 ms).
//
// ARQUITECTURA DEL SISTEMA DE CONTROL:
// 1. Control de Energía (Swing-Up): Modula la aceleración virtual del carro 
//    (x_ddot_r) basándose en el error entre la energía mecánica instantánea (E_m) 
//    y la energía de la vertical inestable (E_ref = m*g*Lcm).
// 2. Desacoplamiento No Lineal (PFL Colocada): Cancela de forma exacta las 
//    dinámicas secundarias de la planta (gravedad, fuerzas centrífugas de 
//    Coriolis y fricciones) inyectando la fuerza física necesaria "u" al motor.
//
// SECUENCIA EXPERIMENTAL DE REFERENCIAS (Perfil de Ensayos Automático):
// - 0 a 10s:   Calibración / Calma (Motor deshabilitado, u = 0V).
// - > 10s:     Activación del Swing-Up.
//
// TELEMETRÍA (Salida Serial CSV):
// [ x(m), theta(rad), x_dot(m/s), theta_dot(rad/s), u(V) ]
//
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"   // HAL - C2000Ware
#include "perifericos.h"    // Funciones de inicialización de hardware
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>

//
// Defines
//
#define ENCODER1_CPR 6597      // Relación pulsos-riel (Carro)
#define ENCODER2_CPR 2000      // Relación pulsos-radián (Péndulo)
#define T 10000                // Periodo de la ISR en microsegundos (10 ms = 100 Hz)
#define T_sec 0.01f            // Periodo de muestreo en segundos
#define MAX_PWM 5000           // Resolución digital máxima del módulo ePWM
#define VCC 15.0f              // Alimentación de la etapa de potencia (V)
#define UMAX 4.68f             // Saturación de diseño de la ley de control (V)
#define ZM_FWD 5.0f            // Voltaje de compensación de zona muerta directa (V)
#define ZM_BWD 5.0f            // Voltaje de compensación de zona muerta inversa (V)

// Parámetros Físicos del Proceso (Cart-Pole)
#define Ke 30.0f               // Ganancia del bombeo de energía del Swing-Up
#define Kp 16.0f               // Ganancia proporcional para el control del carro
#define Kd 4.16f               // Ganancia derivativa para el control del carro
#define J 0.0139f              // Momento de inercia del péndulo (Kg·m²)
#define M 1.08f                // Masa del carro (Kg)
#define m 0.12f                // Masa del péndulo (Kg)
#define Cx 4.08f               // Coeficiente de fricción viscosa del carro (N·s/m)
#define Cth 0.0002f            // Coeficiente de fricción viscosa del péndulo (N·m·s/rad)
#define Lcm 0.305f             // Distancia al centro de masas del péndulo (m)
#define g 9.81f                // Aceleración de la gravedad (m/s²)


//
// Globals
//
long pulsos1, pulsos2;
float x, x_ant, x_dot, theta, theta_ant, theta_dot;     // Variables de estado del proceso
float u, u_zm, E_ref, E_m;                               // Variables del algoritmo de control
int16_t u_dig;                                          // Salida digitalizada para el PWM

char txBuffer[100];                                     // Buffer de transmisión UART
volatile bool send_data = false;                        // Flag de sincronización de telemetría
uint32_t timer_ref = 0;                                 // Contador de muestras para perfil temporal (1 muestra = 10ms)

// Estructura para el desempaquetado de floats en UART sin printf pesado
typedef struct {
    char signo; 
    int entero;
    int decimal; 
} float_parts;

//
// Function Prototypes
//
void condiciones_iniciales(void);
void puerto_serie(void); 
void mide_encoder(void);
void calcula_accion_control(void); 
float corrige_ZM(float u);
void aplica_u(float u);
float_parts desglosar_float(float val);
__interrupt void control(void); 

//
// Main
//
void main(void)
{
    // --- INICIALIZACIÓN DEL HW - PERIFÉRICOS ---
    init_System(); 
    
    // --- CONFIGURACIÓN DE LA INTERRUPCIÓN PERIÓDICA (TIMER 0) ---
    setup_TimeInterrupt(&control, T); 
    
    // --- CONDICIONES INICIALES DEL SISTEMA ---
    condiciones_iniciales(); 
    
    // --- BUCLE PRINCIPAL (BACKGROUND LOOP) ---
    while (1)
    {
        if (send_data)
        {
            puerto_serie(); // Envío diferido fuera de la ISR para proteger el tiempo real
            send_data = false; 
        }
    }
}

// Inicialización de variables de estado y actuadores
void condiciones_iniciales(void)
{
    pulsos1 = 0; 
    pulsos2 = 0; 
    x = 0.0f; 
    x_ant = 0.0f; 
    x_dot = 0.0f; 
    theta = 0.0f; 
    theta_ant = 0.0f; 
    theta_dot = 0.0f; 

    // Forzar el Puente en H a estado de parada (Safe Mode)
    EPwm1Regs.CMPA.bit.CMPA = 0;          // Duty cycle al 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;  // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;  // IN2 -> LOW

    u = 0.0f; 
    u_zm = 0.0f; 
    u_dig = 0; 

    // Parámetro energético objetivo (Péndulo en reposo invertido)
    E_ref = m * g * Lcm; 
    E_m = 0.0f; 
    timer_ref = 0;
}

// Desglose de flotantes
float_parts desglosar_float(float val) 
{
    float_parts p;
    p.signo = (val < 0.0f) ? '-' : ' '; 
    
    float val_abs = fabsf(val);
    p.entero = (int)val_abs;
    p.decimal = (int)((val_abs - (float)p.entero) * 10000.0f); 
    
    return p;
}

// Transmisión UART en formato CSV
void puerto_serie(void)
{
    float_parts p_x         = desglosar_float(x);
    float_parts p_theta     = desglosar_float(theta);
    float_parts p_x_dot     = desglosar_float(x_dot);
    float_parts p_theta_dot = desglosar_float(theta_dot);
    float_parts p_u         = desglosar_float(u); 

    sprintf(txBuffer, "%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d\r\n",
            p_x.signo, p_x.entero, p_x.decimal,
            p_theta.signo, p_theta.entero, p_theta.decimal,
            p_x_dot.signo, p_x_dot.entero, p_x_dot.decimal,
            p_theta_dot.signo, p_theta_dot.entero, p_theta_dot.decimal,
            p_u.signo, p_u.entero, p_u.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

// Captura de contadores QEP y derivación con Filtro Paso Bajo (LPF)
void mide_encoder(void)
{
    // ---- MEDICIONES CARRO ----
    pulsos1 = EQep1Regs.QPOSCNT;                                                                         
    x = ((float)pulsos1 / ENCODER1_CPR) * 0.5f;    
    float x_dot_raw = (x - x_ant) * 100.0f;                                 
    x_dot = 0.2f * x_dot + 0.8f * x_dot_raw;            // Filtro paso bajo (Alfa = 0.8)                                                                         
    x_ant = x; 

    // ---- MEDICIONES PÉNDULO ----
    pulsos2 = EQep2Regs.QPOSCNT;
    theta = ((float)pulsos2 / ENCODER2_CPR) * 2.0f * M_PI;
    float theta_dot_raw = (theta - theta_ant) * 100.0f;
    theta_dot = 0.2f * theta_dot + 0.8f * theta_dot_raw;  // Filtro paso bajo (Alfa = 0.8)
    theta_ant = theta; 
}

// Algoritmo Swing-Up basado en Energía + PFL Colocada
void calcula_accion_control(void)
{
    // 1. Energía mecánica instantánea (Kin + Pot)
    E_m = 0.5f * J * theta_dot * theta_dot - m * g * Lcm * cosf(theta);
    
    // 2. Ley de bombeo de energía (Aceleración virtual deseada del carro)
    float x_ddot_r = Ke * theta_dot * cosf(theta) * (E_m - E_ref) - Kp * x - Kd * x_dot;

    // 3. PFL - Aceleración angular inducida en el péndulo (lambda)
    float theta_ddot = (1.0f / J) * (-m * g * Lcm * sinf(theta) - m * Lcm * cosf(theta) * x_ddot_r - Cth * theta_dot);

    // 4. PFL - Cálculo de la Fuerza física "u" (N) para desacoplar no linealidades
    u = (M + m) * x_ddot_r + m * Lcm * cosf(theta) * theta_ddot - m * Lcm * sinf(theta) * (theta_dot * theta_dot) + Cx * x_dot;

    u = u / 0.64f; //Conversión a voltios
    
    // 5. Saturación del actuador
    if (u < -UMAX) {
        u = -UMAX; 
    }
    else if (u > UMAX) {
        u = UMAX; 
    }
}

// Inyección estática de voltaje para vencer fricción de Coulomb
float corrige_ZM(float u)
{
    float uzm = 0.0f; 
    if (u < 0.0f) {
        uzm = u - ZM_BWD;
        if (uzm < -VCC) { uzm = -VCC; } 
    }
    else if (u > 0.0f) {
        uzm = u + ZM_FWD; 
        if (uzm > VCC) { uzm = VCC; } 
    }
    return uzm;
}

// Modulación de registros ePWM y GPIOs de dirección del puente H
void aplica_u(float u)
{  
    u_zm = corrige_ZM(u); 

    // Mapeo lineal de tensión (V) a cuentas de CMPA digitales
    u_dig = (int16_t)((u_zm / VCC) * MAX_PWM);

    if (u_dig < 0) {
        // --- Marcha Atrás (Backward) ---
        EPwm1Regs.CMPA.bit.CMPA = -u_dig;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPASET.bit.GPIO6 = 1;   // IN2 -> HIGH
    }
    else if (u_dig > 0) {
         // --- Marcha Adelante (Forward) --- 
        EPwm1Regs.CMPA.bit.CMPA = u_dig;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;   // IN1 -> HIGH
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }
    else {
        // --- Parada de Emergencia / Neutro ---
        EPwm1Regs.CMPA.bit.CMPA = 0;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; 
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; 
    }
}

// Rutina de Servicio de Interrupción (ISR) del Temporizador de la CPU (100 Hz)
__interrupt void control(void)
{
    CpuTimer0.InterruptCount++; 
    timer_ref++; // Incremento del contador temporal hardware

    if (timer_ref < 1000) 
    {
        // 0 a 10 segundos: Ventana de calma para calibración manual. Motor desconectado.
        mide_encoder();
        u = 0.0f;
        aplica_u(0.0f);
    }
    else 
    {
        // --- BUCLE DE CONTROL ---
        mide_encoder();
        calcula_accion_control();
        aplica_u(u); 
    }
    
    send_data = true; // Flag para procesar telemetría UART
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1; // Limpieza del flag de interrupción física
}

//
// End of file
//
