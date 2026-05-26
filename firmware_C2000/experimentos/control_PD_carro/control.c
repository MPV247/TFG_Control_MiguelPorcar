//#############################################################################
//
// FILE:        control.c
//
// TITLE:       TFG - Control de Posición PD del Carro con Dither y Zona Muerta
//
// AUTHOR:      Miguel Porcar
// DATE:        Mayo 2026
// TARGET:      TI C2000 (TMS320F28004x)
//
// DESCRIPCIÓN:
// Éste módulo ejecuta un lazo de control Proporcional-Derivativo (PD) discreto 
// a una frecuencia de 100 Hz (T = 10ms) para regular la posición lineal (x) 
// del carro transportador sobre el riel guía.
//
// ESTRATEGIAS AVANZADAS DE CONTROL E INSTRUMENTACIÓN:
// 1. Algoritmo PD sin "Derivative Kick": La acción derivativa se calcula usando 
//    directamente la velocidad medida de la planta ($-K_D \cdot \dot{x}$) en lugar de 
//    la derivada del error. Esto previene picos violentos de tensión en el motor 
//    cuando la referencia cambia instantáneamente de forma escalonada.
// 2. Inyección de Dither Activo: Cuando el motor está activo ($u \neq 0$), el script 
//    superpone una señal cuadrada alterna de alta frecuencia (50 Hz) y amplitud 
//    AMP_DITHER = 1.4V. Esta microvibración reduce drásticamente el rozamiento 
//    estático (stiction) en los cojinetes del carro, linealizando el comportamiento.
// 3. Compensación Lineal de Zona Muerta: Introduce un offset simétrico de 4.5V 
//    (ZM_FWD/ZM_BWD) para saltar de inmediato la banda de no-respuesta del puente H.
// 4. Supresión de Ciclos Límite: Si el error de posición cae por debajo de la 
//    resolución física crítica (0.00009m), el control se apaga de forma estricta 
//    para evitar oscilaciones innecesarias en estado estacionario.
// 5. Perfil Secuencial de Referencias Automático: Implementa una máquina de 
//    estados periódica que cambia la consigna de posición cada 10 segundos en la 
//    secuencia: [ 0.0m -> 0.3m -> -0.3m -> 0.3m -> 0.0m ].
//
// TELEMETRÍA (Salida Serial CSV):
// Formato: [ ref(m) , x_real(m) , x_dot(m/s) , u_ideal(V) , u_final_con_ZM_y_Dither(V) ]
//
// CONFIGURACIÓN DE PERIFÉRICOS ASOCIADOS:
// - eQEP1: Decodificación de cuadratura del encoder del carro (ENCODER1_CPR = 6597).
// - ePWM1 / GPIO1 / GPIO6: Señales de potencia y sentido de giro del motor.
// - CpuTimer0: Temporizador maestro de la ISR (10 ms).
//
//#############################################################################

//
// Included Files
//
#include "f28x_project.h"   //HAL - C2000 Ware
#include "perifericos.h"    //Funciones de inicializacion
#include <stdio.h> 
#include <stdlib.h>
#include <math.h>

//
// Defines
//
#define ENCODER1_CPR 6597      //Relacion pulsos-riel
#define T 10000                //Periodo de ISR (10000 mcs = 10 ms)
#define MAX_PWM 5000           //Saturación acción de control (valor digital)
#define VCC 15.0f              //Alimentación del motor (V)
#define UMAX 10.0f             //Saturación del control
#define ZM_FWD 4.5f            //Zona muerta delante (V)
#define ZM_BWD 4.5f            //Zona muerta atrás (V)
#define AMP_DITHER 1.4f        //Amplitud del Dihter (V)
#define KP 15.21f              //Control P
#define KD 1.05f               //Control D
#define CUENTAS_10S 1000

//
// Globals
//
long pulsos;
float x, x_delta, x_ant, x_dot, u, u_zm, ref, e; 
int16_t u_dig; 
char txBuffer[100];
bool send_data = false;  //Flag para envio por puerto serie
bool dither_estado = false; //Dither en el actuador (micromovimientos)

// --- CAMBIO PERIÓDICO DE LA REFERENCIA
uint32_t timer_ref = 0;    // Contador para los 10 segundos
int estado_ref = 0;        // Estado actual de la secuencia (0 a 4)

//Estructura de datos, envio por puerto serie. 
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
void saturacion(void); 
float corrige_ZM(float u);
void aplica_u(float u);
__interrupt void control(void); 

//
// Main
//
void main(void)
{

   // --- INICIALIZACIÓN DEL HW - PERIFERICOS ---
    init_System(); 
    
    // --- RUITNA INTERRUPCIÓN PERIÓDICA ---
    setup_TimeInterrupt(&control, T); 
    
    // --- CONDICIONES INICIALES SISTEMA ---
    condiciones_iniciales(); 
    
    // --- LOOP --
    while (1){
        if (send_data){
            //Envio de datos por el puerto serie
            puerto_serie(); 
            send_data = false; 
        }

    }
}

//Inicializacion de variables
void condiciones_iniciales(void)
{
    //Contador - Encoder
    pulsos = 0;                            // (unidades digitales)
    x = 0.0f;                              // (m)
    x_ant = x;                             // (m)
    x_dot = 0.0f;                          // (m/s)

    //Actuador (Motor parado)
    EPwm1Regs.CMPA.bit.CMPA = 0;          // EN -> 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;  // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;  // IN2 -> LOW

    //Acción de control
    u = 0.0f;                             // (v)
    u_zm = 0.0f;                          // (V)
    u_dig = 0;                            // Valores digitales

    //Referencia y error de medida: 
    ref = 0.0f;                          // Posición en el riel (m)
    e = 0.0f; 
}

//Envio datos
float_parts desglosar_float(float val) {
    float_parts p;
    p.signo = (val < 0.0f) ? '-' : ' '; 
    
    float val_abs = fabsf(val);
    p.entero = (int)val_abs;
    p.decimal = (int)((val_abs - (float)p.entero) * 10000.0f); 
    
    return p;
}

void puerto_serie(void)
{
    float_parts p_r = desglosar_float(ref);
    float_parts p_x  = desglosar_float(x);
    float_parts p_x_dot  = desglosar_float(x_dot);
    float_parts p_u = desglosar_float(u); 
    float_parts p_uzm = desglosar_float(u_zm);

    //Formato CSV: x,x_dot,u
    sprintf(txBuffer, "%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d\r\n",
            p_r.signo, p_r.entero, p_r.decimal,
            p_x.signo,p_x.entero, p_x.decimal,
            p_x_dot.signo, p_x_dot.entero, p_x_dot.decimal,
            p_u.signo,p_u.entero, p_u.decimal,
            p_uzm.signo, p_uzm.entero, p_uzm.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Encoders
void mide_encoder(void)
{
    pulsos = EQep1Regs.QPOSCNT;                                                         
    x = ((float)pulsos / ENCODER1_CPR) * 0.5f;                                //posicion riel (m)
    x_dot = (x - x_ant) * 100.0f;                                            //velocidad riel (m/s)
    x_ant = x; 
}

//Calculo de la acción de control
void calcula_accion_control(void)
{
    e = ref - x; 
    u = KP*e -KD*x_dot; 

    if (fabs(e) < 0.00009){ //Ciclo límite (resolucion)
        u = 0; 
    }
}

//Saturacion de la acción de control teórica
void saturacion(void){
    if (u < -UMAX){
        u = -UMAX; 
    }
    else if (u > UMAX) {
        u = UMAX; 
    }
}

//Zona muerta
float corrige_ZM(float u)
{
    float uzm; 
    if (u < 0){
        uzm = u - ZM_BWD;
        if (uzm < -VCC){u = -VCC; } //Saturar
    }
    else if (u > 0){
        uzm = u  + ZM_FWD; 
        if (uzm > VCC){u = VCC;  } //Saturar
    }

    return uzm;
}
//Driver
void aplica_u(float u)
{
    
    u_zm = corrige_ZM(u); 

    //--- DITHER ---
    if (u != 0.0f) {
        dither_estado = !dither_estado; // Cambia de true a false cada 10ms
        
        if (dither_estado) {
            u_zm += AMP_DITHER; // Semiciclo positivo
        } else {
            u_zm -= AMP_DITHER; // Semiciclo negativo
        }
    }

    if (u_zm > VCC) {
        u_zm = VCC;
    } else if (u_zm < -VCC) {
        u_zm = -VCC;
    }
    //Conversion a valores digitales
    u_dig = (int16_t)((u_zm/VCC)*MAX_PWM);

    if (u_dig < 0){
        // --- Backward ---
        EPwm1Regs.CMPA.bit.CMPA = -u_dig;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPASET.bit.GPIO6 = 1;   // IN2 -> HIGH
    }
    else if (u_dig > 0){
         // --- Forward --- 
        EPwm1Regs.CMPA.bit.CMPA = u_dig;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;   // IN1 -> HIGH
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }
    else{
        // --- Stop ---
        EPwm1Regs.CMPA.bit.CMPA = u_dig;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }
}

//Rutina de interrupcion periodica
__interrupt void control(void)
{
    CpuTimer0.InterruptCount++; 

    // --- CAMBIO DE LA REFERENCIA CADA 10 s
    timer_ref++;
    float t = timer_ref*T; 

    if (timer_ref >= CUENTAS_10S)
    {
        timer_ref = 0;       // Reiniciar cronómetro
        estado_ref++;        // Siguiente posición

        if (estado_ref > 4) estado_ref = 0; // Reiniciar ciclo tras t5

        // Asignación de la referencia según el estado
        switch (estado_ref)
        {
            case 0: ref = 0.0f;   break; // t1
            case 1: ref = 0.3f;   break; // t2
            case 2: ref = -0.3f;   break; // t3
            case 3: ref = 0.3f;  break; // t4
            case 4: ref = 0.0f;   break; // t5
        }
    }

    // --- BUCLE DE CONTROL ---
    mide_encoder(); 
    calcula_accion_control();
    saturacion(); 
    aplica_u(u); 
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//
