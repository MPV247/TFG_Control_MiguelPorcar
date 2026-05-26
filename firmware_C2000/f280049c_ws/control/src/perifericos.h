#ifndef PERIFERICOS_H_
#define PERIFERICOS_H_

#include "f28x_project.h"

// --- Prototipos de Inicialización ---
void init_GPIOs(void);
void init_SCI(void);
void init_SCIA_FIFO(void);
void init_EQEP1(void);
void init_EQEP2(void);
void init_EPWM(void);
void init_System(void);

// --- Funciones de soporte para comunicación ---
void transmitSCIAChar(uint16_t a);
void transmitSCIAMessage(unsigned char * msg);

// --- Funcion interrupcion por tiempo ---
void setup_TimeInterrupt(void (*isr_ptr)(void), uint32_t T);

#endif /* PERIFERICOS_H_ */
