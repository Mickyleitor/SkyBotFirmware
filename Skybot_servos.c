#include "Skybot_servos.h"


uint32_t ui32Period, ui32DutyCycle[2];
int direction[2];
extern int OOFMode;

extern EventGroupHandle_t TickServoDone;
extern QueueHandle_t QueueServoTicksRequest[2];
extern QueueHandle_t QueueServoTicksDone[2];
extern QueueHandle_t QueueServoSpeed[2];

void acelerar_robot(int izquierda,int derecha){
    acelerar_motor(MOTOR_DERECHO,derecha);
    acelerar_motor(MOTOR_IZQUIERDO,izquierda);
}

void acelerar_velocidad(int izquierda,int derecha){
    xQueueOverwrite(QueueServoSpeed[MOTOR_IZQUIERDO],&izquierda);
    xQueueOverwrite(QueueServoSpeed[MOTOR_DERECHO],&derecha);
}

void acelerar_motor(int motor, int speed){
    if(speed > 100) speed = 100;
    else if(speed < -100) speed = -100;

    if(motor == MOTOR_DERECHO){
        ui32DutyCycle[MOTOR_DERECHO] = SPEED_TICK_RIGHT(speed) ;
        direction[MOTOR_DERECHO] = (speed>=0) ? 1 : -1;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    }else{
        ui32DutyCycle[MOTOR_IZQUIERDO] = SPEED_TICK_LEFT(speed) ;
        direction[MOTOR_IZQUIERDO] = (speed>=0) ? 1 : -1;
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
    }
}

bool motor_stopped(int motor){
    return (ui32DutyCycle[motor] == STOPCOUNT);
}

void mover_robot(int distancia){
    if(distancia != 0){
        int ticks = CM_TO_TICK(distancia);
        xEventGroupClearBits(TickServoDone,0b11);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_DERECHO],&ticks);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_IZQUIERDO],&ticks);
        xEventGroupWaitBits(TickServoDone,0b11,pdTRUE,pdTRUE,portMAX_DELAY);
        /*
        int TicksDone = 0;
        while(TicksDone <= ticks){ xQueueReceive(QueueServoTicksDone[MOTOR_DERECHO],&TicksDone,portMAX_DELAY); }
        while(TicksDone <= ticks){ xQueueReceive(QueueServoTicksDone[MOTOR_IZQUIERDO],&TicksDone,portMAX_DELAY); }
        */
    }
}
void mover_motor(int motor, int distancia){
    if(distancia != 0){
        int ticks = CM_TO_TICK(distancia);
        xQueueOverwrite(QueueServoTicksRequest[motor],&ticks);
        int TicksDone = 0;
        while(TicksDone < ticks){ xQueueReceive(QueueServoTicksDone[motor],&TicksDone,portMAX_DELAY); }
    }
}
void mover_motor_IT(int motor, int distancia){
    if(distancia != 0){
        int ticks = CM_TO_TICK(distancia);
        xQueueOverwrite(QueueServoTicksRequest[motor],&ticks);
    }
}

void girar_robot(int grados){
    if(grados != 0){
        int ticks_left = DEGREES_TO_TICK(grados);
        int ticks_right = -ticks_left;
        xEventGroupClearBits(TickServoDone,0b11);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_DERECHO],&ticks_right);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_IZQUIERDO],&ticks_left);
        xEventGroupWaitBits(TickServoDone,0b11,pdTRUE,pdTRUE,portMAX_DELAY);

        int TicksDone = 0;
        while(abs(TicksDone) < abs(ticks_right)){ xQueuePeek(QueueServoTicksDone[MOTOR_DERECHO],&TicksDone,portMAX_DELAY);}
        TicksDone = 0;
        while(abs(TicksDone) < abs(ticks_left)){ xQueuePeek(QueueServoTicksDone[MOTOR_IZQUIERDO],&TicksDone,portMAX_DELAY); }

    }
}

void mover_robot_IT(int distancia){
    if(distancia != 0){
        int ticks = CM_TO_TICK(distancia);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_DERECHO],&ticks);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_IZQUIERDO],&ticks);
    }
}

void girar_robot_IT(int grados){
    if(grados != 0){
        int ticks_left = DEGREES_TO_TICK(grados);
        int ticks_right = -ticks_left;
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_DERECHO],&ticks_right);
        xQueueOverwrite(QueueServoTicksRequest[MOTOR_IZQUIERDO],&ticks_left);
    }

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

    direction[MOTOR_DERECHO] = 1;
    direction[MOTOR_IZQUIERDO] = 1;
}
