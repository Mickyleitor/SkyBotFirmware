#include "Skybot_servos.h"


uint32_t ui32Period, ui32DutyCycle[2];
int direction_right,direction_left;

extern EventGroupHandle_t TickServoDone;
extern QueueHandle_t QueueServoTicksRight,QueueServoTicksLeft;
extern QueueHandle_t QueueServoSpeedRight,QueueServoSpeedLeft;

void acelerar_robot(int izquierda,int derecha){

    ui32DutyCycle[MOTOR_DERECHO] = SPEED_TICK_RIGHT(derecha) ;
    ui32DutyCycle[MOTOR_IZQUIERDO] = SPEED_TICK_LEFT(izquierda) ;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
    direction_left = (izquierda>=0) ? 1 : -1;
    direction_right = (derecha>=0) ? 1 : -1;
}

void acelerar_motor_izquierda(int izquierda){
    if(izquierda > 100) izquierda = 100;
    else if(izquierda < -100) izquierda = -100;

    ui32DutyCycle[MOTOR_IZQUIERDO] = SPEED_TICK_LEFT(izquierda) ;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
    direction_left = (izquierda>=0) ? 1 : -1;
}

void acelerar_motor_derecha(int derecha){
    if(derecha > 100) derecha = 100;
    else if(derecha < -100) derecha = -100;

    ui32DutyCycle[MOTOR_DERECHO] = SPEED_TICK_RIGHT(derecha) ;

    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    direction_right = (derecha>=0) ? 1 : -1;
}

void mover_robot(int distancia){
    int ticks = CM_TO_TICK(distancia);
    xQueueSend(QueueServoTicksRight,&ticks,portMAX_DELAY);
    xQueueSend(QueueServoTicksLeft,&ticks,portMAX_DELAY);
    while(xEventGroupGetBits(TickServoDone)!=0b11){}
    // xEventGroupWaitBits(TickServoDone,0b11,pdTRUE,pdTRUE,portMAX_DELAY);
}

bool motor_stopped(int motor){
    return (ui32DutyCycle[motor] == STOPCOUNT);
}

void girar_robot(int grados){
    int ticks_left = DEGREES_TO_TICK(grados);
    int ticks_right = -ticks_left;
    xQueueSend(QueueServoTicksRight,&ticks_right,portMAX_DELAY);
    xQueueSend(QueueServoTicksLeft,&ticks_left,portMAX_DELAY);
    // xEventGroupWaitBits(TickServoDone,0b11,pdTRUE,pdTRUE,portMAX_DELAY);
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
    ui32Period = FREQ_PWM;
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

    direction_right = 1;
    direction_left = 1;
}
