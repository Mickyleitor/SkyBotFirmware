/* ************************************************************************** */
/* UNIVERSIDAD DE MALAGA               DEPARTAMENTO DE TECNOLOGIA ELECTRONICA */
/* http://www.uma.es                                    http://www.dte.uma.es */
/* ========================================================================== */
/* PROGRAMA :  SkyBotFirmware                                                 */
/* VERSION  : 1.0                                                             */
/* TARGET   : Kit  TIVA Launchpad IDE CCSv8                                   */
/* RECURSOS :                                                                 */
/* AUTOR    : Ignacio Herrero Reder & Michele La Malva Moreno                 */
/* ************************************************************************** */

#include <stdint.h>
#include <stdbool.h>

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
void acelerar_motor(int motor,int speed);
void acelerar_velocidad(int izquierda,int derecha);

bool motor_stopped(int motor);

void mover_robot(int distancia);
void mover_motor(int motor, int distancia);
void girar_robot(int grados);

// Non blocking command version
void mover_robot_IT(int distancia);
// Non blocking command version
void girar_robot_IT(int grados);
// Non blocking command version
void mover_motor_IT(int motor, int distancia);
// Configuracion Servos
void configServos_init(void);
