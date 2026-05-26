//#############################################################################
//
// FILE:        control.c
//
// TITLE:       TFG - Control de Posición P en Vacío (Sin Carro Acoplado)
//
// AUTHOR:      Miguel Porcar
// DATE:        Mayo 2026
// TARGET:      TI C2000 (TMS320F28004x)
//
// DESCRIPCIÓN:
// Éste módulo implementa un lazo de control puramente Proporcional (P) para 
// regular la posición angular (rad) del motor sin carga mecánica acoplada.
//
// OBJETIVOS Y ESTRATEGIAS DEL ENSAYO:
// 1. Control Proporcional Puro: Se utiliza únicamente la ganancia KP = 6.0f. 
//    Al prescindir de la acción integral, este script permite evaluar de forma 
//    directa el error en régimen permanente (offset) inherente al sistema.
// 2. Frecuencia de Ejecución: El bucle corre estrictamente a 100 Hz (T = 10ms)
//    dentro de la rutina de interrupción del CpuTimer0.
// 3. Banda de Tolerancia Estacionaria: Mantiene el filtro de umbral mínimo 
//    (u_v < 0.05V) para evitar oscilaciones de baja amplitud en el motor cuando 
//    el error es prácticamente nulo.
// 4. Linealización por Zona Muerta: El driver aplica los mismos offsets directos 
//    (ZM_FWD_V/ZM_BWD_V = 2.55V) para neutralizar la fricción estática inicial 
//    del motor de corriente continua.
//
// TELEMETRÍA (Salida Serial CSV):
// Envía los datos desglosados en coma flotante tras cada ciclo de control:
// Formato: [ tension_P(V) , tension_con_ZM(V) , posicion_real(rad) , referencia(rad) ]
//
// CONFIGURACIÓN DE PERIFÉRICOS ASOCIADOS:
// - eQEP1: Registro de conteo de pulsos del encoder del motor.
// - ePWM1 / GPIO1 / GPIO6: Modulación de ancho de pulso y lógica de dirección.
// - SCI-A: Transmisión de tramas de depuración y registro hacia el PC.
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
#define ENCODER_CPR 2048.0f    //Relacion pulsos-rev
#define T 10000                //Periodo de ISR (10000 mcs = 10 ms)
#define MAX_PWM 5000           //Saturacion de la accion de control (valores digitales)
#define VCC 15.0f              //Alimentación del motor (V)
#define ZM_FWD 850             //Valor zona muerta adelante (valores digitales)
#define ZM_BWD 850             //Valor zonas muertas atras  (valores digitales)
#define ZM_FWD_V 2.55f          //Valor zona muerta adelante (V)
#define ZM_BWD_V 2.55f          //Valor zonas muertas atras  (V)
#define KP 6.0f                 //Ganancia proporcional 


//
// Globals
//
long pulsos, pulsos_ant;
float pos, w, ref, u_v, u_vzm, e, e_ant, e_dot; 
char txBuffer[100];
bool send_data = false;  //Flag para envio por puerto serie
int counter_ref, cambios; 

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
void ataca_motor(float u); 
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
    pulsos = 0; 
    pulsos_ant = pulsos; 
    pos = 0.0f;                          // rad
    w = 0.0f;                            // rad/s
    ref = 0.0f;                          // rad

    //Incializacion PTE-H (Motor parado)
    EPwm1Regs.CMPA.bit.CMPA = 0;         // EN  -> 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW

    //Accion de control
    u_v = 0.0f;                          // Sin ZM (V)
    u_vzm = 0.0f;                        // Con ZM (V)
    
    //Error de medida
    e = 0.0f; 
    e_ant = 0.0f; 
    e_dot = 0.0f; 

    //Contadores (cambiar ref)
    counter_ref = 0;                     //Contador para bajar la frecuencia
    cambios = 0;                        //Nº veces que se ha cambiado la ref
}

//Envio datos
float_parts desglosar_float(float val) {
    float_parts p;
    p.signo = (val < 0.0f) ? '-' : ' '; 
    
    float val_abs = fabsf(val);
    p.entero = (int)val_abs;
    p.decimal = (int)((val_abs - (float)p.entero) * 100.0f); 
    
    return p;
}

void puerto_serie(void)
{
    float_parts p_u    = desglosar_float(u_v);
    float_parts p_uzm  = desglosar_float(u_vzm);
    float_parts p_pos  = desglosar_float(pos);
    float_parts p_ref  = desglosar_float(ref);

    sprintf(txBuffer, "%c%d.%02d,%c%d.%02d,%c%d.%02d,%c%d.%02d\r\n",
            p_u.signo,   p_u.entero,   p_u.decimal,
            p_uzm.signo, p_uzm.entero, p_uzm.decimal,
            p_pos.signo, p_pos.entero, p_pos.decimal,
            p_ref.signo, p_ref.entero, p_ref.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Encoders
void mide_encoder(void)
{
    pulsos = EQep1Regs.QPOSCNT;                                               //Lectura del registro
    pos = (pulsos / ENCODER_CPR) * 2.0f * M_PI;                               //Posicion angular (rad)
    w = ((float)(pulsos - pulsos_ant) / ENCODER_CPR) * 2.0f * M_PI * 100.0f;  //Para 10 ms. (rad/s)

    pulsos_ant = pulsos;                                                      //Actualizar contador
}

//Calculo de la accion de control
void calcula_accion_control(void)
{
    e = ref - pos;
    e_dot = (e - e_ant)/0.01f; 
    u_v = KP * e ; 


    if (fabsf(u_v) < 0.05f) { //Evitar vibraciones
        u_v = 0.0f; 
    } 

    e_ant = e; //Actualiza el error
}

//Motores
void ataca_motor(float u) 
{
    /* * Función para controlar la potencia y dirección del puente H.
    * Recibe un valor 'accion_control' entre -5000 (Atrás al 100%) y 5000 (Adelante al 100%)
     */

    //Conversión voltios a undidades_digitales
    int16_t u_dig = (int16_t)((u/VCC)*MAX_PWM);
     
    if (u_dig < 0){
        u_vzm = u - ZM_BWD_V;  //Por experimentar y comprobar. 
        u_dig= u_dig- ZM_BWD; 
        if (u_dig< -MAX_PWM) { //Saturacion
            u_dig= -MAX_PWM;
            u_vzm = -15.0f; 
        }

        // --- Backward ---
        EPwm1Regs.CMPA.bit.CMPA = -u_dig;
        GpioDataRegs.GPACLEAR.bit.GPIO1 = 1; // IN1 -> LOW
        GpioDataRegs.GPASET.bit.GPIO6 = 1;   // IN2 -> HIGH
    }
    else if (u_dig> 0) {
        u_vzm = u + ZM_FWD_V;  //Por experimentar y comprobar. 
        u_dig= u_dig+ ZM_FWD;  
        if (u_dig> MAX_PWM) {  //Saturacion
            u_dig= MAX_PWM;
            u_vzm = 15.0f; 
        }
         // --- Forward --- 
        EPwm1Regs.CMPA.bit.CMPA = u_dig;
        GpioDataRegs.GPASET.bit.GPIO1 = 1;   // IN1 -> HIGH
        GpioDataRegs.GPACLEAR.bit.GPIO6 = 1; // IN2 -> LOW
    }
    else {
        u_vzm = u; 
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
        
    //1. Control (100 Hz)
    mide_encoder(); 
    calcula_accion_control();
    ataca_motor(u_v); 
 
    // 2. Limpias la bandera de la interrupción general de la CPU y bandera de envio de datos
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//

