/* ************************************************************************** */
/* UNIVERSIDAD DE MALAGA               DEPARTAMENTO DE TECNOLOGIA ELECTRONICA */
/* http://www.uma.es                                    http://www.dte.uma.es */
/* ========================================================================== */
/* PROGRAMA :  PWM-Servo                                                      */
/* VERSION  : 1.0                                                             */
/* TARGET   : Kit  TIVA Launchpad IDE CCSv7                                   */
/* DESCRIPCION : Este programas genera dos salidas PWM a través de los        */
/* terminales PF2 y PF3 usando el Timer adecuado en modo PWM, o un modulo PWM */
/*  Al pulsar los botones de la placa, deberña aumentar/reducir el ciclo de   */
/*  trabajo, provocando un aumento/reducción de la velocidad e incluso cambio */
/*  sentido                                                                   */
/* RECURSOS :                                                                 */
/* AUTOR    : Ignacio Herrero Reder (iherrero@uma.es)                         */
/* FECHA    : 08/10/17                                                        */
/* COMENTARIOS  : 1 tabulador = 8 espacios                                    */

/* **************************************************************************   */
/* **************************************************************************   */
/* (ver cabecera en el código del programa)                                     */
/* **************************************************************************   */
#include <stdint.h>
#include <stdbool.h>
// Librerias que se incluyen tipicamente para configuracion de perifericos y pinout
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/pin_map.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de periférico y otros que se vaya a usar para control PWM y gestion de botones
#include "driverlib/gpio.h"
#include "driverlib/buttons.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"

#define PERIOD_PWM SysCtlPWMClockGet() / 50  // Ciclos de reloj para conseguir una señal periódica de 50Hz (según reloj de periférico usado)
#define COUNT_1MS PERIOD_PWM / ( 20 * 1.22 )     // Ciclos para amplitud de pulso de 1ms (max velocidad en un sentido)
#define STOPCOUNT PERIOD_PWM / ( 20 * 1.489) // Ciclos para amplitud de pulso de parada (1.52ms)
#define COUNT_2MS PERIOD_PWM / ( 20 * 1.8 )   // Ciclos para amplitud de pulso de 2ms (max velocidad en el otro sentido)
#define NUM_STEPS 50    // Pasos para cambiar entre el pulso de 2ms al de 1ms
#define CYCLE_INCREMENTS (abs(COUNT_1MS-COUNT_2MS))/NUM_STEPS  // Variacion de amplitud tras pulsacion

bool PWMenabled = 1;
uint8_t ui8Buttons, ui8Changed;
uint32_t ui32Period, ui32DutyCycle[2];

int main(void){

    // Elegir reloj adecuado para los valores de ciclos sean de tamaño soportable ( 40 MHz )
    SysCtlClockSet(SYSCTL_SYSDIV_5|SYSCTL_USE_PLL|SYSCTL_XTAL_16MHZ|SYSCTL_OSC_MAIN);

    // Configura pulsadores placa TIVA (int. por flanco de bajada)
    // Configuracion de puertos (Botones)
    // Habilita puerto GPIOF (Botones)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // La interrupcion se activa con flanco como de bajada.
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);
    // Funcion API de la placa para inicializacion de botones (definida en /drivers/buttons).
    // Documentacion de estas funciones en EK-TM4C123GXL Firmware Development Packages Users Guide, pag 11 y 13
    ButtonsInit();
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable( GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4 );
    // Borra Interrupciones (por si acaso)
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);

  // Configuracion  ondas PWM: frecuencia 50Hz, anchura inicial= valor STOPCOUNT, 1540us para salida por PF2, y COUNT_1MS (o COUNT_2MS ) para salida por PF3(puedes ponerlo inicialmente a  PERIOD_PWM/10)
    // Opcion 2: Usar un módulo PWM (no dado en Sist. Empotrados pero mas sencillo)
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    //Configure PWM Clock to match system
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);  //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the PF2 and PF3
    //Configure PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    //Configure PWM Options
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Ponemos valores personalizados
    ui32Period = PERIOD_PWM;
    ui32DutyCycle[0] = STOPCOUNT;
    ui32DutyCycle[1] = STOPCOUNT;

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Period);

    //Set PWM duty
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[0]);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[1]);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);

   // Habilita interrupcion del modulo Puerto F
   IntEnable(INT_GPIOF);
   IntMasterEnable();
  // Codigo principal, (poner en bucle infinito o bajo consumo)
    while(1){}
}

void RutinaISR(void)
{
    // Rutinas de interrupcion de pulsadores
    // Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
    // Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS
    ButtonsPoll(&ui8Changed,&ui8Buttons);
    // Las etiquetas LEFT_BUTTON, RIGHT_BUTTON, y ALL_BUTTONS estan definidas en /driverlib/buttons.h
    if((RIGHT_BUTTON & ui8Buttons) && (ui32DutyCycle[0]>COUNT_2MS)){ // Boton derecho pulsado?
        ui32DutyCycle[0] -= CYCLE_INCREMENTS;
    }else if((LEFT_BUTTON & ui8Buttons) && (ui32DutyCycle[0]<COUNT_1MS)){     // Boton izquierdo pulsado?
        ui32DutyCycle[0] += CYCLE_INCREMENTS;
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[0] );

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4 );
}

