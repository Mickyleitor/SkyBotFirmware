#include <stdint.h>
#include <stdbool.h>
#include <time.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de periférico y otros que se vaya a usar para control PWM y gestion de botones
#include "driverlib/gpio.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"
#include "driverlib/qei.h"

// Formula STOP_VALUE - (VALOR/(PERIOD_PWM/20))
#define LEFT_VALUE 0.12                         // Desviacion maxima en ms del ciclo de trabajo del motor izquierdo
#define RIGHT_VALUE 0.15                         // Desviacion maxima en ms del ciclo de trabajo del motor derecho
#define STOP_VALUE 1.52                          // Ciclos para amplitud de pulso de parada (1.2ms)
#define NUM_STEPS 200                            // Numero de pasos totales requeridos para ir de max velocidad en un sentido hasta el otro sentido

#define M_PI           3.14159265358979323846
#define RADIO 3
#define SEPARACION 10

// Ciclos de reloj para conseguir una senal periodica de 50Hz (segun reloj de periferico usado)
#define FREQ_PWM 50000
// Ciclos para amplitud de pulso minimo para motor izquierdo  (max velocidad en un sentido)
#define MAXCOUNT_LEFT  (FREQ_PWM / 20 ) * (STOP_VALUE + LEFT_VALUE)
// Ciclos para amplitud de pulso maximo para motor izquierdo  (max velocidad en el sentido contrario)
#define MINCOUNT_LEFT  (FREQ_PWM / 20 ) * (STOP_VALUE - LEFT_VALUE)
// Ciclos para amplitud de pulso minimo para motor derecho    (max velocidad en un sentido)
#define MAXCOUNT_RIGHT  (FREQ_PWM / 20 ) * (STOP_VALUE + RIGHT_VALUE)
// Ciclos para amplitud de pulso maximo para motor derecho    (max velocidad en el sentido contrario)
#define MINCOUNT_RIGHT  (FREQ_PWM / 20 ) * (STOP_VALUE - RIGHT_VALUE)
// Ciclos para amplitud de pulso de parada para ambos motores (calibrados con el potenciometro)
#define STOPCOUNT (FREQ_PWM / 20 ) * STOP_VALUE
// Variacion de amplitud tras pulsacion para motor izquierdo
#define CYCLE_INCREMENTS_LEFT (abs(MAXCOUNT_LEFT-MINCOUNT_LEFT))/NUM_STEPS
// Variacion de amplitud tras pulsacion para motor derecho
#define CYCLE_INCREMENTS_RIGHT (abs(MAXCOUNT_RIGHT-MINCOUNT_RIGHT))/NUM_STEPS

#define SPEED_TICK_RIGHT(speed) STOPCOUNT-(NUM_STEPS*0.5*speed*0.01*CYCLE_INCREMENTS_RIGHT)
#define SPEED_TICK_LEFT(speed) STOPCOUNT+(NUM_STEPS*0.5*speed*0.01*CYCLE_INCREMENTS_LEFT)

#define CM_TO_TICK(distancia) (distancia*18) / (2*M_PI*RADIO)
#define DEGREES_TO_TICK(degrees) CM_TO_TICK((degrees * M_PI * SEPARACION) / 360)

#define MOTOR_DERECHO 0
#define MOTOR_IZQUIERDO 1

// Velocidad positivo es acelerar, negativo desacelerar
void acelerar_robot(int izquierda,int derecha);
void acelerar_motor_izquierda(int izquierda);
void acelerar_motor_derecha(int derecha);

bool motor_stopped(int motor);

void mover_robot(int distancia);
void girar_robot(int grados);
// Non blocking command version
void mover_robot_IT(int distancia);
// Non blocking command version
void girar_robot_IT(int grados);

void configServos_init(void);

unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax);
