#include "Skybot_servos.h"

extern uint32_t ui32Period, ui32DutyCycle[2];
unsigned short RawValueDistance_0A41F [32] = {0,1,2,3,4,5,6,7,8,9,10,11,12,13,14,15,16,17,18,19,20,21,22,23,24,25,26,27,28,29,30,31};
unsigned short RawValueADC_0A41F [32] = {0x398,0x4B0,0x6A4,0x9C8,0x960,0x7D0,0x708,0x5F0,0x540,0x4B0,0x448,0x3E8,0x398,0x364,0x308,0x2D0,0x2A8,0x280,0x258,0x244,0x220,0x1FC,0x1E8,0x1D0,0x1B8,0x1A8,0x198,0x190,0x184,0x174,0x16C};
unsigned short RawValueDistance_0A51F [32] = {0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10,10.5,11,11.5,12,12.5,13,13.5,14,14.5,15};
unsigned short RawValueADC_0A51F [32] = {0x498,0x764,0x8D4,0x7D0,0x6A4,0x5CC,0x528,0x488,0x428,0x3CC,0x374,0x320,0x2F0,0x2D0,0x294,0x260,0x244,0x230,0x208,0x1E8,0x1D8,0x1B8,0x1A4,0x194,0x190,0x17C,0x16C,0x15C,0x14C,0x138,0x12C};


// Diferencial positivo es derecha, negativo es izquierda
void girar(int16_t diferencial){
    int16_t contador;
    if(diferencial > 0){
        for(contador=0; contador < diferencial ;contador ++ ){
            // Incrementa diferencial positivo en el motor izquierdo
            if((ui32DutyCycle[MOTOR_IZQUIERDO]+CYCLE_INCREMENTS_LEFT)<MAXCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] += CYCLE_INCREMENTS_LEFT;
            // Incrementa diferencial positivo en el motor derecho
            if((ui32DutyCycle[MOTOR_DERECHO]+CYCLE_INCREMENTS_RIGHT)<MAXCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] += CYCLE_INCREMENTS_RIGHT;
        }
    }else{
        for(contador=0; contador < abs(diferencial) ;contador ++ ){
            // Incrementa diferencial negativo en el motor izquierdo
            if((ui32DutyCycle[MOTOR_IZQUIERDO]-CYCLE_INCREMENTS_LEFT)>MINCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] -= CYCLE_INCREMENTS_LEFT;
            // Incrementa diferencial negativo en el motor derecho
            if((ui32DutyCycle[MOTOR_DERECHO]-CYCLE_INCREMENTS_RIGHT)>MINCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] -= CYCLE_INCREMENTS_RIGHT;
        }
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
}

void avanzar(int16_t velocidad){
    int16_t contador;
    if(velocidad > 0){
        for(contador=0; contador < velocidad ;contador ++ ){
            // Incrementa el ciclo de trabajo para el motor izquierdo y lo disminuye en el motor derecho
            if(((ui32DutyCycle[MOTOR_IZQUIERDO]+CYCLE_INCREMENTS_LEFT)<MAXCOUNT_LEFT) && ((ui32DutyCycle[MOTOR_DERECHO]-CYCLE_INCREMENTS_RIGHT)>MINCOUNT_RIGHT)){
                ui32DutyCycle[MOTOR_DERECHO] -= CYCLE_INCREMENTS_RIGHT;
                ui32DutyCycle[MOTOR_IZQUIERDO] += CYCLE_INCREMENTS_LEFT;
            }
        }
    }else{
        for(contador=0; contador < abs(velocidad) ;contador ++ ){
            // Decrementa el ciclo de trabajo para el motor izquierdo y lo incrementa en el motor derecho
            if(((ui32DutyCycle[MOTOR_IZQUIERDO]-CYCLE_INCREMENTS_LEFT)>MINCOUNT_LEFT) && ((ui32DutyCycle[MOTOR_DERECHO]+CYCLE_INCREMENTS_RIGHT)<MAXCOUNT_RIGHT)){
                ui32DutyCycle[MOTOR_DERECHO] += CYCLE_INCREMENTS_RIGHT;
                ui32DutyCycle[MOTOR_IZQUIERDO] -= CYCLE_INCREMENTS_LEFT;
            }
        }
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
}

void configServos_init(void){
    //Configure PWM Options
    //Configure PWM Clock to match system
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    //Inicializa el puerto F (Para puertos PWM)
    //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the PF2 and PF3
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    // Configuramos PF2 y PF3 como PWM
    // Desactivado solo para tarea de LEDS
    // ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    // Comentar para tarea final
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    ROM_GPIOPinConfigure(GPIO_PF2_M1PWM6);
    ROM_GPIOPinConfigure(GPIO_PF3_M1PWM7);
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);
    // Ponemos valores personalizados
    ui32Period = PERIOD_PWM;
    // Motor Derecho
    ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
    // Motor Izquierdo
    ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Period);
    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);
    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);
}

unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax)
{
  unsigned int imid;

  while (imin < imax)
    {
      imid= (imin+imax)>>1;

      if (A[imid] < key)
        imin = imid + 1;
      else
        imax = imid;
    }
    return imax;    //Al final imax=imin y en dicha posicion hay un numero mayor o igual que el buscado
}
