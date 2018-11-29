#include "Skybot_servos.h"


extern uint32_t ui32Period, ui32DutyCycle[2];
unsigned short RawValueDistance_0A41F [32] = {31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};
// unsigned short RawValueADC_0A41F [32] = {0x16C,0x174,0x184,0x190,0x198,0x1A8,0x1B8,0x1D0,0x1E8,0x1FC,0x220,0x244,0x258,0x280,0x2A8,0x2D0,0x308,0x364,0x398,0x3E8,0x448,0X4B0,0x540,0x5F0,0x708,0x7D0,0x960,0x9C8,0x6A4,0x4B0,0x398};
unsigned short RawValueDistance_0A51F [32] = {0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10,10.5,11,11.5,12,12.5,13,13.5,14,14.5,15};
unsigned short RawValueADC_0A51F [32] = {0x498,0x764,0x8D4,0x7D0,0x6A4,0x5CC,0x528,0x488,0x428,0x3CC,0x374,0x320,0x2F0,0x2D0,0x294,0x260,0x244,0x230,0x208,0x1E8,0x1D8,0x1B8,0x1A4,0x194,0x190,0x17C,0x16C,0x15C,0x14C,0x138,0x12C};

#define M_PI           3.14159265358979323846
#define RADIO 3
#define SEPARACION 9.5
#define EcDistancia(pL,pR) (RADIO/2)* ( ( (PulsosContados[MOTOR_IZQUIERDO]-pL) * 0.4188) + ((PulsosContados[MOTOR_DERECHO]-pR) *0.349) )
#define EcGiro(pL,pR) (RADIO/SEPARACION) * ( ( ((PulsosContados[MOTOR_DERECHO]-pR) *0.349) - (PulsosContados[MOTOR_IZQUIERDO]-pL) * 0.4188) ) * (180/M_PI)
volatile int PulsosContados[2];

// Diferencial positivo es derecha, negativo es izquierda
void acelerar_giro_robot(int diferencial,bool isAbsolute){
    int incrementos = NUM_STEPS*diferencial*0.01;

    if(isAbsolute){
        ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
        ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
    }
    ui32DutyCycle[MOTOR_DERECHO] += CYCLE_INCREMENTS_RIGHT*incrementos;
    ui32DutyCycle[MOTOR_IZQUIERDO] += CYCLE_INCREMENTS_LEFT*incrementos;

    if(ui32DutyCycle[MOTOR_DERECHO]>MAXCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] = MAXCOUNT_RIGHT;
    else if(ui32DutyCycle[MOTOR_DERECHO]<MINCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] = MINCOUNT_RIGHT;

    if(ui32DutyCycle[MOTOR_IZQUIERDO]>MAXCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] = MAXCOUNT_LEFT;
    else if(ui32DutyCycle[MOTOR_IZQUIERDO]<MINCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] = MINCOUNT_LEFT;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
}

void acelerar_velocidad_robot(int velocidad,bool isAbsolute){
    int incrementos = NUM_STEPS*velocidad*0.01;

    if(isAbsolute){
        ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
        ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
    }
    ui32DutyCycle[MOTOR_DERECHO] -= CYCLE_INCREMENTS_RIGHT*incrementos;
    ui32DutyCycle[MOTOR_IZQUIERDO] += CYCLE_INCREMENTS_LEFT*incrementos;

    if(ui32DutyCycle[MOTOR_DERECHO]>MAXCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] = MAXCOUNT_RIGHT;
    else if(ui32DutyCycle[MOTOR_DERECHO]<MINCOUNT_RIGHT) ui32DutyCycle[MOTOR_DERECHO] = MINCOUNT_RIGHT;

    if(ui32DutyCycle[MOTOR_IZQUIERDO]>MAXCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] = MAXCOUNT_LEFT;
    else if(ui32DutyCycle[MOTOR_IZQUIERDO]<MINCOUNT_LEFT) ui32DutyCycle[MOTOR_IZQUIERDO] = MINCOUNT_LEFT;


    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
}


void mover_robot(int distancia){
    int pInit[2] = {PulsosContados[MOTOR_DERECHO],PulsosContados[MOTOR_IZQUIERDO]};
    int MeasuredDistance = EcDistancia(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    while( abs(MeasuredDistance) < abs( distancia ) ){
        int velocidad = 110 - ((abs(MeasuredDistance)*100)/abs(distancia));

        if( distancia > 0) acelerar_velocidad_robot(20,true);
        else acelerar_velocidad_robot(-20,true);

        MeasuredDistance = EcDistancia(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    }
    acelerar_velocidad_robot(0,true);
}

void girar_robot(int grados){
    int pInit[2] = {PulsosContados[MOTOR_DERECHO],PulsosContados[MOTOR_IZQUIERDO]};
    int MeasuredDegrees = EcGiro(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    while( abs(MeasuredDegrees) < abs( grados )){
        int diferencial = 110 - ((abs(MeasuredDegrees)*100)/abs(grados));

        if( grados > 0) acelerar_giro_robot(20,true);
        else acelerar_giro_robot(-20,true);

        MeasuredDegrees = EcGiro(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    }
    acelerar_giro_robot(0,true);
}

void RutinaEncoders_ISR(void)
{
    if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_2){
        PulsosContados[MOTOR_DERECHO] += (ui32DutyCycle[MOTOR_IZQUIERDO] >= STOPCOUNT)? 1 : -1;
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,!ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1) * 255 );
    }
    if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_3){
        PulsosContados[MOTOR_DERECHO] += (ui32DutyCycle[MOTOR_DERECHO] <= STOPCOUNT)? 1 : -1;
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,!ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1) * 255 );
    }
    GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);
}

void configEncoders_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    // ROM_GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // La interrupcion se activa con flanco como de bajada.
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2,GPIO_BOTH_EDGES);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);

    IntEnable(INT_GPIOA);
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
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    // Comentar para tarea final
    // ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1 | GPIO_PIN_2 | GPIO_PIN_3);
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
