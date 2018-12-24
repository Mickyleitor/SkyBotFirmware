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

// Formula STOP_VALUE - (VALOR/(PERIOD_PWM/20))
// #define LEFT_VALUE 0.08                          // Desviacion maxima en ms del ciclo de trabajo del motor izquierdo
// #define RIGHT_VALUE 0.15                         // Desviacion maxima en ms del ciclo de trabajo del motor derecho
#define LEFT_VALUE 0.08
#define RIGHT_VALUE 0.09                         // Desviacion maxima en ms del ciclo de trabajo del motor derecho
#define STOP_VALUE 1.53                          // Ciclos para amplitud de pulso de parada (1.2ms)
// #define STOP_VALUE 1.20                       // OLD
#define NUM_STEPS 300                            // Numero de pasos totales requeridos para ir de max velocidad en un sentido hasta el otro sentido


// Ciclos de reloj para conseguir una senal periodica de 50Hz (segun reloj de periferico usado)
#define PERIOD_PWM SysCtlPWMClockGet() / 29
// Ciclos para amplitud de pulso minimo para motor izquierdo (max velocidad en un sentido)
#define MAXCOUNT_LEFT  (PERIOD_PWM / 20 ) * (STOP_VALUE + LEFT_VALUE)
// Ciclos para amplitud de pulso maximo para motor izquierdo (max velocidad en el sentido contrario)
#define MINCOUNT_LEFT  (PERIOD_PWM / 20 ) * (STOP_VALUE - LEFT_VALUE)
// Ciclos para amplitud de pulso minimo para motor derecho (max velocidad en un sentido)
#define MAXCOUNT_RIGHT  (PERIOD_PWM / 20 ) * (STOP_VALUE + RIGHT_VALUE)
// Ciclos para amplitud de pulso maximo para motor derecho (max velocidad en el sentido contrario)
#define MINCOUNT_RIGHT  (PERIOD_PWM / 20 ) * (STOP_VALUE - RIGHT_VALUE)
// Ciclos para amplitud de pulso de parada para ambos motores (calibrados con el potenciometro)
#define STOPCOUNT (PERIOD_PWM / 20 ) * STOP_VALUE
// Variacion de amplitud tras pulsacion para motor izquierdo
#define CYCLE_INCREMENTS_LEFT (abs(MAXCOUNT_LEFT-MINCOUNT_LEFT))/NUM_STEPS
// Variacion de amplitud tras pulsacion para motor derecho
#define CYCLE_INCREMENTS_RIGHT (abs(MAXCOUNT_RIGHT-MINCOUNT_RIGHT))/NUM_STEPS

#define MOTOR_DERECHO 0
#define MOTOR_IZQUIERDO 1

// Diferencial positivo es derecha, negativo es izquierda
void acelerar_giro_robot(int diferencial, bool isAbsolute);
// Velocidad positivo es acelerar, negativo desacelerar
void acelerar_velocidad_robot(int velocidad,bool isAbsolute);

void acelerar_robot(int izquierda,int derecha,bool isAbsolute);

void mover_robot(int distancia);
void girar_robot(int grados);
//
void configEncoders_init(void);
void RutinaEncoders_ISR(void);
void configServos_init(void);

unsigned short binary_lookup(unsigned short *A, unsigned short key, unsigned short imin, unsigned short imax);
