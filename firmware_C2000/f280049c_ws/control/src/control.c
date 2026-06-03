//#############################################################################
// DESCRIPCIÓN:
//
// Este módulo implementa un algoritmo de CONTROL HÍBRIDO CONMUTADO para un
// sistema de péndulo invertido sobre un carro (Cart-Pole). 
//
// Atendiendo a un umbral estricto en el error angular del péndulo, el sistema
// conmuta automáticamente entre dos regímenes de operación:
//   1. Modo No Lineal: Control de energía (Swing-Up) para elevar el péndulo.
//   2. Modo Lineal: Realimentación de estados para estabilizarlo en la vertical.
//
//
// ARQUITECTURA DEL SISTEMA DE CONTROL:
//
//
// --- MODALIDAD A: CONTROL NO LINEAL (Swing-Up + PFL Colocada) ---
// Fuera del umbral de captura, el sistema busca ganar energía:
//
// 1. Control de Energía: Modula la aceleración virtual del carro (x_ddot_r) 
//    basándose en el error entre la energía mecánica instantánea (E_m) 
//    y la energía de la vertical inestable de referencia (E_ref = m * g * Lcm).
//
// 2. Desacoplamiento No Lineal (PFL): Cancela de forma exacta las dinámicas 
//    secundarias de la planta (gravedad, fuerzas centrífugas de Coriolis y 
//    fricciones) inyectando la fuerza física "u" calculada al motor.
//
// --- MODALIDAD B: CONTROL LINEAL (Realimentación de Estados) ---
// Dentro del umbral de captura, el sistema se comporta como un regulador local:
//
// 1. Vector de Estados: Se define mediante los estados nativos
//    x_vec = [x, theta, x_dot, theta_dot]^T.
//
// 2. Ley de Control Implementada:
//    u = K1*(ref_x - x) + K2*(ref_theta - theta) + K3*(0 - x_dot) + K4*(0 - theta_dot)
//
//
// ACONDICIONAMIENTO DE SEÑALES Y SEGURIDAD:
//
//
// 1. Envoltura Trigonométrica (Acondicionamiento de Ángulo):
//    Se utiliza la función `atan2f(sinf(e), cosf(e))` para normalizar el error 
//    angular en el intervalo simétrico [-pi, pi]. Esto evita discontinuidades 
//    catastróficas (saltos de 2*pi) en la acción de control cuando el péndulo 
//    cruza velozmente la vertical superior.
//
// 2. Saturación:
//    Cuando la acción de control calculada supera la saturación física de 
//    seguridad (UMAX = 4.68V).
//
//
// SECUENCIA EXPERIMENTAL DE REFERENCIAS (Perfil de Ensayos):
//
// - 0 a 10s:   Motor deshabilitado, u = 0V.
// - > 10s:     Activación del Control híbrido.
//
// TELEMETRÍA (Salida Serial CSV):
//
// [ x(m), ref_x(m), theta(rad), ref_theta(rad), x_dot(m/s), theta_dot(rad/s), u(V),
// E_m (J), E_ref (J), tipo_control (bool)]
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
#define ENCODER1_CPR 6597       //Relacion pulsos-riel
#define ENCODER2_CPR 2000       //Relacion pulsos-pendulo
#define T 10000                 //Periodo de ISR (10000 mcs = 10 ms)
#define T_sec 0.01              //Periodo de muestreo en segundos
#define MAX_PWM 5000            //Saturación acción de control (valor digital)
#define VCC 15.0f               //Alimentación del motor (V)
#define UMAX 4.68f              //Saturación del control (V)
#define ZM_FWD 5.0f             //Zona muerta delante (V)
#define ZM_BWD 5.0f             //Zona muerta atrás (V)
#define K1 -31.6228f            // Ganancia para posición (x)
#define K2 246.8238f            // Ganancia para ángulo (theta)
#define K3 -56.5082f            // Ganancia para velocidad (x_dot)
#define K4 57.6621f             // Ganancia para velocidad angular (theta_dot) 
#define KI 0.0f                 //Sin acción integral
#define Ke 100.0f               // Ganancia del bombeo de energía del Swing-Up
#define Kp 3.62f               // Ganancia proporcional para el control del carro
#define Kd 2.66f               // Ganancia derivativa para el control del carro

// Parámetros Físicos del Proceso (Cart-Pole)
#define J 0.028f               // Momento de inercia del péndulo (Kg·m²)
#define M 1.08f                // Masa del carro (Kg)
#define m 0.24f                // Masa del péndulo (Kg)
#define Cx 4.08f               // Coeficiente de fricción viscosa del carro (N·s/m)
#define Cth 0.0002f            // Coeficiente de fricción viscosa del péndulo (N·m·s/rad)
#define Lcm 0.305f             // Distancia al centro de masas del péndulo (m)
#define g 9.81f                // Aceleración de la gravedad (m/s^2)

//
// Globals
//
long pulsos1, pulsos2;
float x, x_ant, x_dot, theta, theta_ant, theta_dot,theta_dot_ant, theta_ddot_m, Ie;            //Variables de estado
float theta_dot_raw, theta_ddot_raw, theta_ddot;
float u, u_zm, ref_x, ref_theta, E_ref, E_m;                              //Variables de control
float u_swp, u_kx; 
int16_t u_dig; 

char txBuffer[100];                                                //Variables envio puerto serie
bool send_data = false;
bool region_lineal = false;                                            

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
    ref_x = 0.0f;                          //Referencia en el eje X
    ref_theta = M_PI;                      //Referencia posición péndulo                     
    Ie = 0.0f;                             //Error integral
    E_ref = - m * g * Lcm;                   //Energía de referencia
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
    float_parts p_Em = desglosar_float(E_m);
    float_parts p_Er = desglosar_float(E_ref);

    //Formato CSV: x, ref_x, theta, ref_theta, x_dot,theta_dot,u,E_m, E_ref, control
    sprintf(txBuffer, "%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%c%d.%04d,%d\r\n",
            p_x.signo, p_x.entero, p_x.decimal,
            p_rx.signo, p_rx.entero, p_rx.decimal,
            p_theta.signo,p_theta.entero, p_theta.decimal,
            p_r.signo, p_r.entero, p_r.decimal,
            p_x_dot.signo, p_x_dot.entero, p_x_dot.decimal,
            p_theta_dot.signo, p_theta_dot.entero, p_theta_dot.decimal,
            p_u.signo,p_u.entero, p_u.decimal,
            p_Em.signo, p_Em.entero, p_Em.decimal,
            p_Er.signo, p_Er.entero, p_Er.decimal,
            region_lineal);

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
    theta_dot_raw = (theta - theta_ant) * 100.0f;
    theta_dot = 0.2f * theta_dot + 0.8f * theta_dot_raw;  //Filtro Paso Bajo
    theta_ant = theta; 

    theta_ddot_raw = (theta_dot - theta_dot_ant) * 100.0f;
    theta_ddot_m = 0.2f * theta_ddot_m + 0.8f * theta_ddot_raw;  //Filtro Paso Bajo
    theta_dot_ant = theta_dot;
}

//Calculo de la acción de control
void calcula_accion_control(void)
{
    float e1 = ref_x - x;
    float e2 = ref_theta - theta; 
    float e3 = 0.0f - x_dot;
    float e4 = 0.0f - theta_dot;
    
    // --- UMBRAL DE CONMUTACIÓN ---
    e2 = atan2f(sinf(e2), cosf(e2));    // Normalizar error a [-pi, pi]
    // Bandera para saber en qué región de control estamos
    region_lineal = (fabs(e2) < 0.1f);

    if (region_lineal) { // Control Kx (Realimentación de estado)

        // 1. Ley de control
        u = K1 * e1 + K2 * e2 + K3 * e3 + K4 * e4;
        
        // 2. Ciclo límite (evitar desgaste del actuador)
        if (fabs(e1) < 0.001f && fabs(e2) < 0.003f) {
            u = 0.0f; 
        }

    } else { // Control SWP + PFL (Swing-up)

        // 1. Rampa de energía de referencia
        if (2*m * g * Lcm > E_ref + 0.001*m * g * Lcm){
            E_ref = E_ref + 0.001*m * g * Lcm;
        }
        
        // 2. Energía mecánica 
        E_m = 0.5f * J * theta_dot * theta_dot - m * g * Lcm * cosf(theta);
        
        // 2. Ley de bombeo de energía (Aceleración virtual deseada del carro)
        float x_ddot_r = Ke * theta_dot * cosf(theta) * (E_m - E_ref) - Kp *( x-ref_x) - Kd * x_dot;

        // 3. PFL - Aceleración angular en el péndulo
        theta_ddot = theta_ddot_m;
        // 4. PFL - Cálculo de la Fuerza física "u" (N) para desacoplar no linealidades
       u = (M + m) * x_ddot_r + m * Lcm * cosf(theta) * theta_ddot - m * Lcm * sinf(theta) * (theta_dot * theta_dot) + Cx * x_dot;
       u = u / 0.64f; //Conversión a voltios
    }

    // --- BLOQUE DE SATURACIÓN ---
    if (u < -UMAX) {
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
    
    send_data = true; 
    PieCtrlRegs.PIEACK.all = PIEACK_GROUP1;
}
//
// End of file
//
