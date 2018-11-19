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


#define LEFT_VALUE 0.29                          // Desviacion maxima en ms del ciclo de trabajo del motor izquierdo
#define RIGHT_VALUE 0.172                         // Desviacion maxima en ms del ciclo de trabajo del motor derecho
#define STOP_VALUE 1.20                          // Ciclos para amplitud de pulso de parada (1.2ms)
#define NUM_STEPS 300                            // Numero de pasos totales requeridos para ir de max velocidad en un sentido hasta el otro sentido


// Ciclos de reloj para conseguir una senal periodica de 50Hz (segun reloj de periferico usado)
#define PERIOD_PWM SysCtlPWMClockGet() / 50
// Ciclos para amplitud de pulso minimo para motor izquierdo (max velocidad en un sentido)
#define MAXCOUNT_LEFT PERIOD_PWM / ( 20 * (STOP_VALUE - LEFT_VALUE) )
// Ciclos para amplitud de pulso maximo para motor izquierdo (max velocidad en el sentido contrario)
#define MINCOUNT_LEFT PERIOD_PWM / ( 20 * (STOP_VALUE + LEFT_VALUE) )
// Ciclos para amplitud de pulso minimo para motor derecho (max velocidad en un sentido)
#define MAXCOUNT_RIGHT PERIOD_PWM / ( 20 * (STOP_VALUE - RIGHT_VALUE) )
// Ciclos para amplitud de pulso maximo para motor derecho (max velocidad en el sentido contrario)
#define MINCOUNT_RIGHT PERIOD_PWM / ( 20 * (STOP_VALUE + RIGHT_VALUE) )
// Ciclos para amplitud de pulso de parada para ambos motores (calibrados con el potenciometro)
#define STOPCOUNT PERIOD_PWM / ( 20 * STOP_VALUE)
// Variacion de amplitud tras pulsacion para motor izquierdo
#define CYCLE_INCREMENTS_LEFT (abs(MAXCOUNT_LEFT-MINCOUNT_LEFT))/NUM_STEPS
// Variacion de amplitud tras pulsacion para motor derecho
#define CYCLE_INCREMENTS_RIGHT (abs(MAXCOUNT_RIGHT-MINCOUNT_RIGHT))/NUM_STEPS

#define MOTOR_DERECHO 0
#define MOTOR_IZQUIERDO 1

#define PACKED __attribute__ ((packed))
typedef union{
    struct {
                uint32_t PF0:1;
                uint32_t PF4:1;
                uint32_t PB0:1;
                uint32_t PB1:1;
                uint32_t PB2:1;
    } PACKED flags;
    uint32_t ui32Valor;
} PACKED PARAM_COMANDO_FLAGALARM;

// Diferencial positivo es derecha, negativo es izquierda
void girar(int16_t diferencial);
// Velocidad positivo es acelerar, negativo desacelerar
void avanzar(int16_t velocidad);
