//#############################################################################
//
// FILE:   control.c
//
// TITLE:  CONTROL.
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
#define ZM_FWD 0.0f            //Zona muerta delante (V)
#define ZM_BWD 0.0f            //Zona muerta atrás (V)

//
// Globals
//
long pulsos;
float x, x_delta, x_ant, x_dot, u, u_zm; 
int16_t u_dig; 
char txBuffer[100];
bool send_data = false;  //Flag para envio por puerto serie

//Experimento ZM
uint32_t timer_pasos = 0;
float u_test = 0.0f;
bool test_activo = true; 
long pulsos_iniciales = 0;
int n = 1; //Indica el numero de experimento
bool en_espera = false;    // Indica si estamos en la pausa de 3 segundos
uint32_t timer_espera = 0; // Contador para la pausa

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
    pulsos_iniciales = EQep1Regs.QPOSCNT;

    //Actuador (Motor parado)
    EPwm1Regs.CMPA.bit.CMPA = 0;          // EN -> 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;  // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;  // IN2 -> LOW

    //Acción de control
    u = 0.0f;                             // (v)
    u_zm = 0.0f;                          // (V)
    u_dig = 0;                            // Valores digitales
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
    float_parts p_x  = desglosar_float(x);
    float_parts p_x_dot  = desglosar_float(x_dot);
    float_parts p_u = desglosar_float(u); 

    //Formato CSV: num_exp,x,x_dot,u
    sprintf(txBuffer, "%d,%c%d.%04d,%c%d.%04d,%c%d.%04d\r\n",
            n, 
            p_x.signo,p_x.entero, p_x.decimal,
            p_x_dot.signo, p_x_dot.entero, p_x_dot.decimal,
            p_u.signo,p_u.entero, p_u.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Encoders
void mide_encoder(void)
{
    pulsos = EQep1Regs.QPOSCNT;
    long pulsos_relativos = pulsos - pulsos_iniciales;                                                            
    x = ((float)pulsos_relativos / ENCODER1_CPR) * 0.5f;                                //posicion riel (m)
    x_dot = (x - x_ant) * 100.0f;                                            //velocidad riel (m/s)
    x_ant = x; 
}

//Zona muerta
float corrige_ZM(float u)
{
    if (u < 0){
        u = u - ZM_BWD;
        if (u < -VCC){u = -VCC; } //Saturar
    }
    else if (u > 0){
        u = u  + ZM_FWD; 
        if (u > VCC){u = VCC;  } //Saturar
    }

    return u;
}
//Driver
void aplica_u(float u)
{
    u_zm = corrige_ZM(u); 
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
    mide_encoder(); 

    if (test_activo) {
        
        if (en_espera) {
            // --- LÓGICA DE ESPERA (3 SEGUNDOS) ---
            u = 0.0f; // Motor parado

            x = 0.0f; //Forzamos todo a 0 para captura limpia de datos
            x_ant = 0.0f;
            x_dot = 0.0f;
            timer_espera++;

            if (timer_espera >= 300) { // 300 * 10ms = 3 segundos
                en_espera = false;
                timer_espera = 0;
                
                // Justo al terminar la espera, calibramos el cero real
                // Esto absorbe cualquier movimiento por inercia previo
                pulsos_iniciales = EQep1Regs.QPOSCNT; 
                u_test = 0.0f;
                timer_pasos = 0;
            }
        } 
        else {
            // --- LÓGICA DE EXPERIMENTO (RAMPA) ---
            timer_pasos++;
            
            if (timer_pasos >= 100) {
                timer_pasos = 0;
                u_test += 0.05f; 
            }

            // Límite de distancia (5 cm)
            if (abs(pulsos - pulsos_iniciales) > 660) {
                u = 0.0f; 
                n++; 
                
                if (n > 5) {
                    test_activo = false; // Fin de la batería de tests
                } else {
                    // En lugar de resetear aquí, activamos la espera
                    en_espera = true; 
                    timer_espera = 0;
                }
            } else {
                u = u_test;
            }
        }
    } 
    else {
        u = 0.0f; // Sistema apagado
    }

    aplica_u(u); 
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//

