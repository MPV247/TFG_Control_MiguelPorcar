#include "perifericos.h"

//
// --- PINES ENTRADA/SALIDA ---
//
void init_GPIOs(void)
{
    GPIO_SetupPinMux(28, GPIO_MUX_CPU1, 1); // Configuracion PIN RX
    GPIO_SetupPinOptions(28, GPIO_INPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(29, GPIO_MUX_CPU1, 1); //Configuracion PIN TX
    GPIO_SetupPinOptions(29, GPIO_OUTPUT, GPIO_ASYNC);

    GPIO_SetupPinMux(10, GPIO_MUX_CPU1, 5); // EQEP1A (Canal A) PIN
    GPIO_SetupPinOptions(10, GPIO_INPUT, GPIO_SYNC | GPIO_PULLUP); 

    GPIO_SetupPinMux(11, GPIO_MUX_CPU1, 5); // EQEP1B (Canal B) PIN
    GPIO_SetupPinOptions(11, GPIO_INPUT, GPIO_SYNC | GPIO_PULLUP);

    GPIO_SetupPinMux(14, GPIO_MUX_CPU1, 10); // EQEP2A (Canal A) PIN
    GPIO_SetupPinOptions(14, GPIO_INPUT, GPIO_SYNC | GPIO_PULLUP); 

    GPIO_SetupPinMux(15, GPIO_MUX_CPU1, 10); // EQEP2B (Canal B) PIN
    GPIO_SetupPinOptions(15, GPIO_INPUT, GPIO_SYNC | GPIO_PULLUP);  

    GPIO_SetupPinMux(0, GPIO_MUX_CPU1, 1); //PWM (Pin EN en PTE H)
    GPIO_SetupPinOptions(0, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(1, GPIO_MUX_CPU1, 0); // IN1
    GPIO_SetupPinOptions(1, GPIO_OUTPUT, GPIO_PUSHPULL);

    GPIO_SetupPinMux(6, GPIO_MUX_CPU1, 0); // IN2
    GPIO_SetupPinOptions(6, GPIO_OUTPUT, GPIO_PUSHPULL);
}

//
//--- COMUNICACION SERIE ---
//
void init_SCI(void)
{
    SciaRegs.SCICCR.all = 0x0007;           // 1 stop bit,  No loopback
                                            // No parity, 8 char bits,
                                            // async mode, idle-line protocol
    SciaRegs.SCICTL1.all = 0x0003;          // enable TX, RX, internal SCICLK,
                                            // Disable RX ERR, SLEEP, TXWAKE
    SciaRegs.SCICTL2.all = 0x0003;
    SciaRegs.SCICTL2.bit.TXINTENA = 1;
    SciaRegs.SCICTL2.bit.RXBKINTENA = 1;

    //
    // SCIA at 115200 baud
    // @LSPCLK = 25 MHz (100 MHz SYSCLK) HBAUD = 0x00  and LBAUD = 0x1A.
    //
    SciaRegs.SCIHBAUD.all = 0x0000;
    SciaRegs.SCILBAUD.all = 0x001A;

    SciaRegs.SCICTL1.all = 0x0023;          // Relinquish SCI from Reset
}

// transmitSCIAChar - Transmit a character from the SCI
void transmitSCIAChar(uint16_t a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0)
    {

    }
    
    // 8 bits menos significativos para utilizar sprintf
    SciaRegs.SCITXBUF.all = (a & 0x00FF);
}

// transmitSCIAMessage - Transmit message via SCIA
void transmitSCIAMessage(unsigned char * msg)
{
    int i;
    i = 0;
    while(msg[i] != '\0')
    {
        transmitSCIAChar(msg[i]);
        i++;
    }
}

// initSCIAFIFO - Initialize the SCI FIFO
void init_SCIA_FIFO(void)
{
    SciaRegs.SCIFFTX.all = 0xE040;
    SciaRegs.SCIFFRX.all = 0x2044;
    SciaRegs.SCIFFCT.all = 0x0;
}

//
// --- ENCODERS ---
//
void init_EQEP1(void)
{
    EALLOW;                              //Desbloqueo de registros criticos del sistema
    CpuSysRegs.PCLKCR4.bit.EQEP1 = 1;    //Habilitar señal de reloj al periferico
    EDIS;                                //Deshabilitar escritura en registros críticos
    EQep1Regs.QDECCTL.bit.QSRC = 0;      //Doble precision
    EQep1Regs.QDECCTL.bit.SWAP = 0;      //Cambio entre canal A y B internamente
    EQep1Regs.QEPCTL.bit.FREE_SOFT = 2; 
    EQep1Regs.QPOSMAX = 0xFFFFFFFF;      // Numero de cuentas maximas
    EQep1Regs.QPOSINIT = 0;             //Inicializacion pulsos encoder
    EQep1Regs.QPOSCNT = 0;  
    EQep1Regs.QEPCTL.bit.QPEN = 1;      //Habiltar el periferico
}

void init_EQEP2(void)
{
    EALLOW;                              //Desbloqueo de registros criticos del sistema
    CpuSysRegs.PCLKCR4.bit.EQEP2 = 1;    //Habilitar señal de reloj al periferico
    EDIS;                                //Deshabilitar escritura en registros críticos
    EQep2Regs.QDECCTL.bit.QSRC = 0;      //Doble precision
    EQep2Regs.QDECCTL.bit.SWAP = 0;      //Cambio entre canal A y B internamente
    EQep2Regs.QEPCTL.bit.FREE_SOFT = 2; 
    EQep2Regs.QPOSMAX = 0xFFFFFFFF;      // Numero de cuentas maximas
    EQep2Regs.QPOSINIT = 0;             //Inicializacion pulsos encoder
    EQep2Regs.QPOSCNT = 0;  
    EQep2Regs.QEPCTL.bit.QPEN = 1;      //Habiltar el periferico
}

//
// --- PWM ---
//
void init_EPWM(void)
{
    EALLOW;
    CpuSysRegs.PCLKCR2.bit.EPWM1 = 1; // Habilitar reloj al módulo ePWM1
    EDIS;

    // 1. Time-Base: Frecuencia a 1 kHz (SYSCLK a 100MHz)
    EPwm1Regs.TBPRD = 5000;          // Periodo (Resolucion)
    EPwm1Regs.TBPHS.bit.TBPHS = 0;   // Fase 0 (Para un solo motor no afecta)
    EPwm1Regs.TBCTR = 0;             // Resetear contador (buenas practicas)

    // Modo Triangular (Up-Down) para control simétrico de motores
    EPwm1Regs.TBCTL.bit.CTRMODE = TB_COUNT_UPDOWN; 
    EPwm1Regs.TBCTL.bit.PHSEN = TB_DISABLE;
    EPwm1Regs.TBCTL.bit.HSPCLKDIV = 5;
    EPwm1Regs.TBCTL.bit.CLKDIV = 1;

    // 2. Counter-Compare: Duty Cycle Inicial al 0%
    EPwm1Regs.CMPA.bit.CMPA = 0; 

    // 3. Action-Qualifier: Crear la onda
    EPwm1Regs.AQCTLA.bit.CAU = AQ_CLEAR;   // Forzar a BAJO (0V) cuando el contador sube y cruza CMPA
    EPwm1Regs.AQCTLA.bit.CAD = AQ_SET;     // Forzar a ALTO (3.3V) cuando el contador baja y cruza CMPA
}

//
// --- RUTINA INTERRUPCION PERIODICA ---
//
void setup_TimeInterrupt(void (*isr_ptr)(void), uint32_t T)
{
    EALLOW;
    PieVectTable.TIMER0_INT = isr_ptr;  
    EDIS; 

    InitCpuTimers();
    ConfigCpuTimer(&CpuTimer0, 100, T); // CPU-Timer 0, 100 MHZ (Freq reloj DSP), T
    CpuTimer0Regs.TCR.all = 0x4000; 

    IER |= M_INT1; 
    PieCtrlRegs.PIEIER1.bit.INTx7 = 1;
    
    EINT;
    ERTM;
}

//
// --- SISTEMA COMPLETO ---
//
void init_System(void)
{
    InitSysCtrl(); // Configuración del Reloj.
    InitGpio();    //Pines GPIO inicializados a valores de fabrica.
    init_GPIOs(); 

    // --- INTERRUPCIONES ---
    DINT;               //Disable Interrupts
    InitPieCtrl();      //PIE --> Reinicio del gestor de alarmas
    IER = 0x0000;       // Interrupt Enable Register a 0
    IFR = 0x0000;       //Interrupt Flag Register a 0
    InitPieVectTable(); //Reinicio del vector de interrupciones.

    init_SCIA_FIFO();                                             
    init_SCI();
    init_EQEP1(); 
    init_EQEP2();
    init_EPWM();
}

