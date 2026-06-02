//#############################################################################
// DESCRIPCIÓN 
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
#define ENCODER1_CPR 6597       //Relacion pulsos-riel
#define ENCODER2_CPR 2000       //Relacion pulsos-pendulo
#define T 10000                 //Periodo de ISR (10000 mcs = 10 ms)
#define T_sec 0.01              //Periodo de muestreo en segundos
#define MAX_PWM 5000            //Saturación acción de control (valor digital)
#define VCC 15.0f               //Alimentación del motor (V)
#define UMAX 4.68f              //Saturación del control (V)
#define ZM_FWD 5.0f             //Zona muerta delante (V)
#define ZM_BWD 5.0f             //Zona muerta atrás (V)
#define K1 -115.71f               // Ganancia para posición (x)
#define K2 260.31f               // Ganancia para ángulo (theta)
#define K3 -82.96f               // Ganancia para velocidad (x_dot)
#define K4 51.87f               // Ganancia para velocidad angular (theta_dot) 
#define KI -54.77f              // Ganancia para el error integral en la posicion (x)

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
#define g 9.81f                // Aceleración de la gravedad (m/s^2)

//
// Globals
//
long pulsos1, pulsos2;
float x, x_ant, x_dot, theta, theta_ant, theta_dot, Ie;            //Variables de estado

float u, u_zm, ref_theta, E_ref, E_m;                              //Variables de control
int16_t u_dig; 

char txBuffer[100];                                                //Variables envio puerto serie
bool send_data = false;                                           

uint32_t timer_ref = 0;                                            // Contador para los 10 segundos iniciales


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
    pulsos1 = 0;                           // Encoder motor (u dig)
    pulsos2 = 0;                           // Encoder pendulo (u dig)
    x = 0.0f;                              // posicion riel (m)
    x_ant = x;                            
    x_dot = 0.0f;                          // velocidad lineal carro (m/s)
    theta = 0.0f;                          // angulo del pendulo (rad)
    theta_ant = theta;                    
    theta_dot = 0.0f;                      // velocidad angular pendulo (rad/s)

    //Actuador (Motor parado)
    EPwm1Regs.CMPA.bit.CMPA = 0;          // EN -> 0%
    GpioDataRegs.GPACLEAR.bit.GPIO1 = 1;  // IN1 -> LOW
    GpioDataRegs.GPACLEAR.bit.GPIO6 = 1;  // IN2 -> LOW

    //Acción de control
    u = 0.0f;                             // (v)
    u_zm = 0.0f;                          // (V)
    u_dig = 0;                            // Unidades digitales (u dig)

    //Referencia y error de medida:                           
    ref_theta = 0.0f;                      // Referencia angulo del pendulo
    Ie = 0.0f;                             //Error integral
    E_ref = m * g * Lcm;                   //Energía de referencia
    E_m = 0.0f;                            //Energía mecánica del sistema 
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
    float_parts p_theta = desglosar_float(theta);
    float_parts p_x_dot  = desglosar_float(x_dot);
    float_parts p_theta_dot = desglosar_float(theta_dot);
    float_parts p_r = desglosar_float(ref_theta);
    float_parts p_rx = desglosar_float(ref_x);
    float_parts p_u = desglosar_float(u); 

    //Formato CSV: x, ref_x, theta, ref_theta, x_dot,theta_dot,u
    sprintf(txBuffer, "%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d\r\n",
            p_x.signo, p_x.entero, p_x.decimal,
            p_rx.signo, p_rx.entero, p_rx.decimal,
            p_theta.signo,p_theta.entero, p_theta.decimal,
            p_r.signo, p_r.entero, p_r.decimal,
            p_x_dot.signo, p_x_dot.entero, p_x_dot.decimal,
            p_theta_dot.signo, p_theta_dot.entero, p_theta_dot.decimal,
            p_u.signo,p_u.entero, p_u.decimal);

    transmitSCIAMessage((unsigned char *)txBuffer);
}

//Encoders
void mide_encoder(void)
{
    // ---- MEDICIONES CARRO ----
    pulsos1 = EQep1Regs.QPOSCNT;                                                         
    x = ((float)pulsos1 / ENCODER1_CPR) * 0.5f;    
    float x_dot_raw = (x - x_ant) * 100.0f;                         
    x_dot = 0.2f * x_dot + 0.8f * x_dot_raw ;            //Filtro Paso Bajo                                         
    x_ant = x; 

    // ---- MEDICIONES PENDULO ----
    pulsos2 = EQep2Regs.QPOSCNT;
    theta = ((float)pulsos2 / ENCODER2_CPR) * 2 * M_PI;
    float theta_dot_raw = (theta - theta_ant) * 100.0f;
    theta_dot = 0.2f * theta_dot + 0.8f * theta_dot_raw;  //Filtro Paso Bajo
    theta_ant = theta; 
}

//Calculo de la acción de control
void calcula_accion_control(void)
{
    float e1 = ref_x - x;
    float e2 =  ref_theta - theta; 
    e2 = atan2f(sinf(e2), cosf(e2));    //Normalizar error
    float e3 = 0.0f - x_dot;
    float e4 = 0.0f - theta_dot;
    Ie = Ie + T_sec * (ref_x - x);
 
    if (fabs(e2) < 0.2){ //Kx

        // Ley de control por realimentacion del estado: u = K * e
        u = K1 * e1 + K2 * e2 + K3 * e3 + K4 * e4 + KI * Ie;
        //Ciclo límite (evitar desgaste excesivo del actuador)
        if (fabs(e1) < 0.001 && fabs(e2) < 0.003){
            u = 0; 
        }
        //Saturación y anti-windup aquí: 
        if (u < -UMAX){
            u = -UMAX; 
            Ie = Ie - T_sec * (ref_x - x);
        }
        else if (u > UMAX) {
            u = UMAX; 
            Ie = Ie - T_sec * (ref_x - x);
        }
    }
    else{ //SWP + PFL

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
}

//Zona muerta
float corrige_ZM(float u)
{
    float uzm; 
    if (u < 0){
        uzm = u - ZM_BWD;
        if (uzm < -VCC){uzm = -VCC; } //Saturar
    }
    else if (u > 0){
        uzm = u  + ZM_FWD; 
        if (uzm > VCC){uzm = VCC;  } //Saturar
    }
    return uzm;
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
    // --- BUCLE DE CONTROL ---
    mide_encoder();
    calcula_accion_control();
    aplica_u(u); 

    
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//