#include "Skybot_servos.h"

extern uint32_t ui32Period, ui32DutyCycle[2];

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
