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
#include <math.h>

//
// Defines
//
#define ENCODER_CPR 2048.0f    //Relacion pulsos-rev
#define T 1000000              //Periodo de muestreo (microsec)
#define MAX_PWM 5000           //Saturacion de la accion de control
#define ZM_FWD 1130            //Valor zona muerta adelante
#define ZM_BWD 1200            //Valor zonas muertas atras

//
// Globals
//
uint16_t loopCounter = 0;
long pulsos, pulsos_ant;
float pos, w; 
char txBuffer[100];
bool send_data = false;  //Flag para envio por puerto serie
int estado_test; 
int16_t u; 

//
// Function Prototypes
//
void condiciones_iniciales(void);
void puerto_serie(void); 
void mide_encoder(void);
void ataca_motor(int16_t u); 
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
    
    // --- CONDICIONES INICIALES ---
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
    pulsos = 0; 
    pulsos_ant = pulsos; 
    pos = 0.0f; 
    w = 0.0f; 

    //Incializacion PTE-H (Motor parado)
    EPwm1Regs.CMPA.bit.CMPA = 0;         // EN  -> 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW

    //Accion de control
    u = 0; 

    //Estado test zona muerta
    estado_test = 0; 

}

//Envio datos
void puerto_serie(void)
{
    //Conversion flotante-entero (Mayor eficiencia)
    int w_int = (int)w; 
    int w_dec = (int)((w - w_int)*100); 
    
    //Corregimos números negativos en la parte decimal
    if (w_dec < 0) {w_dec = -w_dec; }
            
    // Formatear cadena formato CSV
    if (w < 0.0f && w_int == 0) {
        // Añadimos el signo menos manualmente si w_int es 0 pero w es negativo
        sprintf(txBuffer, "%d,-%d.%02d\r\n", u, w_int, w_dec);
    } else {
        sprintf(txBuffer, "%d,%d.%02d\r\n", u, w_int, w_dec);
    }
                
    // Transmitir msg
    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Medida
void mide_encoder(void)
{
    pulsos = EQep1Regs.QPOSCNT;                  //Lectura del registro
    pos = (pulsos / ENCODER_CPR) * 2.0f * M_PI;  //Posicion angular (rad)
    w = ((float)(pulsos - pulsos_ant) / ENCODER_CPR) * 2.0f * M_PI;  //Para 1 s.              //Velocidad angular (rad/s)

    pulsos_ant = pulsos; //Actualizar 
}

//Accion de control
void ataca_motor(int16_t u) 
{
    /* * Función para controlar la potencia y dirección del puente H.
    * Recibe un valor 'accion_control' entre -5000 (Atrás al 100%) y 5000 (Adelante al 100%)
     */
     
    if (u < 0){
        u = u - ZM_BWD; 
        if (u < -MAX_PWM) { //Saturacion
            u = -MAX_PWM;
        }

        // --- Backward ---
        EPwm1Regs.CMPA.bit.CMPA = -u;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPASET.bit.GPIO6 = 1;   // IN2 -> HIGH
    }
    else if (u > 0) {
        u = u + ZM_FWD; 
        if (u > MAX_PWM) {  //Saturacion
            u = MAX_PWM;
        }
         // --- Forward --- 
        EPwm1Regs.CMPA.bit.CMPA = u;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;   // IN1 -> HIGH
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }
    else {
        // --- Stop ---
        EPwm1Regs.CMPA.bit.CMPA = u;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }

}

//Rutina de interrupcion periodica
__interrupt void control(void)
{
    CpuTimer0.InterruptCount++; 
    
    // 1. Lectura encoder
    mide_encoder(); 

    //2. Accion de control (Detectar zona muerta)
    switch(estado_test) 
    {
        case 0:  //Adelante
            u = u + 1; 
            if (u > 60) { //50% del ciclo de trabajo
                u = 0; 
                estado_test = 1; // Pasamos a frenar
            }
            break;

        case 1:  //Transitorio1
            u = 0; // Motor apagado
            if (w < 1.0f && w > -1.0f) { //Casi detenido
                estado_test = 2;
            }
            break;

        case 2:  //Atras
            u = u - 1; 
            if (u < -60) { //50% del ciclo de trabajo
                u = 0; 
                estado_test = 3; 
            }
            break;

        case 3:  //Transitorio2
            u = 0; // Motor apagado
            if (w < 1.0f && w > -1.0f) { 
                estado_test = 0; // Reinicia el ciclo continuo
            }
            break;
    }

    ataca_motor(u); 
    
    //3. Flag para el envio de datos por serie
    send_data = true; 

    // 4. Limpias la bandera de la interrupción general de la CPU
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//

