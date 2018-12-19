#include "Skybot_servos.h"


extern uint32_t ui32Period, ui32DutyCycle[2];
// 0.349
#define M_PI           3.14159265358979323846
#define RADIO 3
#define SEPARACION 9.5
#define EcDistancia(pL,pR) (RADIO/2)* ( ( abs( PulsosContados[MOTOR_IZQUIERDO]-pL ) * 0.349) + ( abs(PulsosContados[MOTOR_DERECHO]-pR) * 0.25) )
#define EcGiro(pL,pR) (RADIO/SEPARACION) * ( ( abs(PulsosContados[MOTOR_IZQUIERDO]-pL) * 0.349) + ( abs(PulsosContados[MOTOR_DERECHO]-pR) * 0.25) ) * (180/M_PI)
volatile int PulsosContados[2];

static long GetMiliseconds(void) {
    return (xTaskGetTickCountFromISR());
}

// Diferencial positivo es derecha, negativo es izquierda
void acelerar_giro_robot(int diferencial,bool isAbsolute){
    int incrementos = 0.5*NUM_STEPS*diferencial*0.01;

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
    int incrementos = 0.5*NUM_STEPS*velocidad*0.01;

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
    int pInit[2] = {PulsosContados[0],PulsosContados[1]};
    int MeasuredDistance = EcDistancia(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);

    while( abs(MeasuredDistance) < abs( distancia ) ){
        int velocidad = 110 - ((abs(MeasuredDistance)*100)/abs(distancia));

        if( distancia > 0) acelerar_velocidad_robot(50,true);
        else acelerar_velocidad_robot(-50,true);

        int giro = EcGiro(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
        UARTprintf ("Distancia: %d Giro %d\n",MeasuredDistance,giro);

        MeasuredDistance = EcDistancia(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    }
    UARTprintf ("Distancia: %d Giro %d\n",MeasuredDistance);
    acelerar_velocidad_robot(0,true);
}

void girar_robot(int grados){
    int pInit[2] = {PulsosContados[0],PulsosContados[1]};
    int MeasuredDegrees = EcGiro(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    while( abs(MeasuredDegrees) < abs( grados )){
        int diferencial = 110 - ((abs(MeasuredDegrees)*100)/abs(grados));

        if( grados > 0) acelerar_giro_robot(50,true);
        else acelerar_giro_robot(-50,true);

        UARTprintf ("Giro: %d\n",MeasuredDegrees);

        MeasuredDegrees = EcGiro(pInit[MOTOR_IZQUIERDO],pInit[MOTOR_DERECHO]);
    }
    UARTprintf ("Giro: %d\n",MeasuredDegrees);
    acelerar_giro_robot(0,true);
}

void RutinaEncoders_ISR(void)
{
    long now = GetMiliseconds();
    if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_3){
        PulsosContados[MOTOR_IZQUIERDO] += (ui32DutyCycle[MOTOR_IZQUIERDO] <= STOPCOUNT)? -1 : 1;
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,!ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1) * 255 );
    }
    if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_2){
        PulsosContados[MOTOR_DERECHO] += (ui32DutyCycle[MOTOR_DERECHO] >= STOPCOUNT)? -1 : 1;
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
