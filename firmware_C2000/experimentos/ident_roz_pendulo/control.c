//#############################################################################
//
// FILE:        control.c
//
// TITLE:       TFG - Ensayo de Decaimiento del Péndulo (Estimación de Rozamiento)
//
// AUTHOR:      Miguel Porcar
// DATE:        Mayo 2026
// TARGET:      TI C2000 (TMS320F28004x)
//
// DESCRIPCIÓN:
// Éste módulo está diseñado para capturar la oscilación libre y amortiguada 
// del péndulo a partir de una posición inicial no nula. No aplica ninguna
// acción de control sobre el motor (u = 0 implicitamente al no activarse).
//
// OBJETIVO DEL ENSAYO:
// Registrar la atenuación de la amplitud angular (theta) en el tiempo tras 
// perturbar el péndulo. Los datos obtenidos en el CSV se utilizarán 
// posteriormente en MATLAB/Python para identificar el modelo de pérdidas por 
// fricción (rozamiento viscoso y culómbico) en la articulación.
//
// TELEMETRÍA (Salida Serial CSV):
// Envía muestras de forma periódica estricta cada 10ms a través del puerto SCI-A:
// Formato: [ pulsos_crudos(long) , angulo_theta(rad) ]
//
// CONFIGURACIÓN DE PERIFÉRICOS ASOCIADOS:
// - eQEP2: Periférico asignado para la lectura del encoder del péndulo.
// - CpuTimer0: Configurado con un periodo de muestreo rápido (T = 10ms) para 
//   capturar con suficiente resolución la dinámica transitoria de la oscilación.
// - SCI-A: Volcado de la trama serial hacia el sistema de adquisición de datos.
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
#define ENCODER1_CPR 2000      //Relacion pulsos-riel
#define T 10000                //Periodo de ISR (10000 mcs = 10 ms)
#define MAX_PWM 5000           //Saturación acción de control (valor digital)

//
// Globals
//
long pulsos;
float theta; 
int16_t u_dig; 
char txBuffer[100];
bool send_data = false;  //Flag para envio por puerto serie

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
    pulsos = 0;                                // (unidades digitales)
    theta = 0.0f;                              // (rad)
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

    float_parts p_theta  = desglosar_float(theta);
    //Formato CSV: pulsos, theta
    sprintf(txBuffer, "%ld,%c%d.%04d\r\n",
            pulsos,
            p_theta.signo,p_theta.entero, p_theta.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Encoders
void mide_encoder(void)
{
    pulsos = EQep2Regs.QPOSCNT;                                                         
    theta = ((float)pulsos / ENCODER1_CPR) * 2* M_PI;                               //angulo pendulo
}


//Rutina de interrupcion periodica
__interrupt void control(void)
{
    CpuTimer0.InterruptCount++; 
    mide_encoder(); 
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//
