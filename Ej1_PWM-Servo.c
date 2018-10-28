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
#include "driverlib/buttons.h"
#include "driverlib/interrupt.h"
#include "driverlib/pwm.h"
#include "driverlib/rom.h"
#include "driverlib/uart.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "utils/uartstdio.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"


#define LEFT_VALUE 0.29                          // Desviacion maxima en ms del ciclo de trabajo del motor izquierdo
#define RIGHT_VALUE 0.172                         // Desviacion maxima en ms del ciclo de trabajo del motor derecho
#define STOP_VALUE 1.2                          // Ciclos para amplitud de pulso de parada (1.2ms)
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
#define LED1TASKPRIO 1
#define LED2TASKPRIO 1
#define LED1TASKSTACKSIZE 128
#define LED2TASKSTACKSIZE 128

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
xQueueHandle QUEUE_GPIO;
TaskHandle_t handle = NULL;
uint8_t ui8Buttons, ui8Changed;
uint32_t ui32Period, ui32DutyCycle[2];

void rotar(int8_t diferencial);
void avanzar(int8_t velocidad);

//*****************************************************************************
//
// The error routine that is called if the driver library encounters an error.
//
//*****************************************************************************
#ifdef DEBUG
void
__error__(char *pcFilename, unsigned long ulLine)
{
}

#endif

//*****************************************************************************
//
// Aqui incluimos los "ganchos" a los diferentes eventos del FreeRTOS
//
//*****************************************************************************

//Esto es lo que se ejecuta cuando el sistema detecta un desbordamiento de pila
//
void vApplicationStackOverflowHook(xTaskHandle *pxTask, signed char *pcTaskName)
{
    //
    // This function can not return, so loop forever.  Interrupts are disabled
    // on entry to this function, so no processor interrupts will interrupt
    // this loop.
    //
    while(1)
    {
    }
}

//Esto se ejecuta cada Tick del sistema. LLeva la estadistica de uso de la CPU (tiempo que la CPU ha estado funcionando)
void vApplicationTickHook( void )
{
    static uint8_t ui8Count = 0;

    if (++ui8Count == 10)
    {
        g_ui32CPUUsage = CPUUsageTick();
        ui8Count = 0;
    }
}

//Esto se ejecuta cada vez que entra a funcionar la tarea Idle
void vApplicationIdleHook (void )
{
        SysCtlSleep();
}

//*****************************************************************************
//
// A continuacion van las tareas...
//
//*****************************************************************************

static portTASK_FUNCTION(LEDTask,pvParameters)
{

    int32_t i32Estado_led=0;

    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        i32Estado_led=!i32Estado_led;

        if (i32Estado_led)
        {
            GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
            vTaskDelay(0.1*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
                                                     //Esta espera es de unos 100ms aproximadamente.
        }
        else
        {
            GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_1,0);
            vTaskDelay(2*configTICK_RATE_HZ);        //Espera del RTOS (eficiente, no gasta CPU)
                                                   //Esta espera es de unos 2s aproximadamente.
        }
    }
}

// MLMM: Tarea que interpreta primer elemento de la FIFO y realiza tantas
// Iteraciones como se hayan pasado por parámetros en funcion de los
// Segundos y la frecuencia. Lo mismo se aplica a la frecuencia.
void LEDTask2(void *pvParameters)
{
    uint32_t Recibido[2];
    uint32_t Contador,SemiPeriodo;
    //
    // Bucle infinito, las tareas en FreeRTOS no pueden "acabar", deben "matarse" con la funcion xTaskDelete().
    //
    while(1)
    {
        if (xQueueReceive(QUEUE_GPIO,&Recibido,0)==pdTRUE){
                // Suspendemos tarea del LED1TASK (la volveremos a llamar cuando haya un comando que lo active de nuevo)
                vTaskSuspend(handle);
                // Numero de veces que se enciende un LED sera
                // Segundos * Frecuencia
                Contador = Recibido[0] * Recibido[1];
                // Tiempo de Semiperiodo = 1 / (f(hz) * 2 )
                // NOTA: Se multiplica por 1000 y despues por 0.001 porque siendo uint32_t
                // NO se pueden almacenar decimales y no se podría imprimir los milisegundos.
                SemiPeriodo = 1000/(Recibido[1]*2);
                UARTprintf("Activada alarma de %d segundos a %d Hz\n",Recibido[0],Recibido[1]);

                while(Contador>0){
                    GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3 , GPIO_PIN_3);
                    vTaskDelay(SemiPeriodo*0.001*configTICK_RATE_HZ);

                    GPIOPinWrite(GPIO_PORTF_BASE,  GPIO_PIN_3,0);
                    vTaskDelay(SemiPeriodo*0.001*configTICK_RATE_HZ);
                    Contador--;
                }
                // Aqui se supone que se acaban de terminar los segundos
                // Y se activa la alarma
                UARTprintf("ALARMA\n");
                // Aqui volvemos a desactivar tarea, ya que las especificaciones dice que una vez
                // La alarma está activada el led rojo permanecerá encendido (y hemos podido "resumir"
                // la tarea de parpadeo gpio rojo mientras estaba en cuenta atras la alarma)
                vTaskSuspend(handle);
                // Encedemos LED rojo por la fuerza
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1 , GPIO_PIN_1);
        }
        // Esto es necesario ya que si no, el uP está todo el rato esperando (bucle infinito)
        // A un caracter, y como esta espera es eficiente, pues resuelve nuestro problema.
        vTaskDelay(configTICK_RATE_HZ); /// Esto equivale a un segundo.
    }
}

//Esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos.
//Aqui solo la declaramos para poderla crear en la funcion main.
extern void vUARTTask( void *pvParameters );

//Aqui podria definir y/o declarar otras tareas definidas en otro fichero....

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

    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);

    //Configure PWM Clock to match system
    SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the PF2 and PF3
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    //Configure PF2,PF3 Pins as PWM
    GPIOPinConfigure(GPIO_PF2_M1PWM6);
    GPIOPinConfigure(GPIO_PF3_M1PWM7);
    GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
    //Configure PWM Options
    //PWM_GEN_3 Covers M1PWM6 and M1PWM7 See page 207 4/11/13 DriverLib doc
    PWMGenConfigure(PWM1_BASE, PWM_GEN_3, PWM_GEN_MODE_UP_DOWN | PWM_GEN_MODE_NO_SYNC);

    // Ponemos valores personalizados
    ui32Period = PERIOD_PWM;
    // Motor Derecho
    ui32DutyCycle[MOTOR_DERECHO] = 1108;
    // Motor Izquierdo
    ui32DutyCycle[MOTOR_IZQUIERDO] = 1356;

    //Set the Period (expressed in clock ticks)
    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_3, ui32Period);

    //Set PWM duty
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);

    // Enable the PWM generator
    PWMGenEnable(PWM1_BASE, PWM_GEN_3);

    // Turn on the Output pins
    PWMOutputState(PWM1_BASE, PWM_OUT_6_BIT|PWM_OUT_7_BIT, true);



   // Habilita interrupcion del modulo Puerto F
   IntEnable(INT_GPIOF);
   IntMasterEnable();
  // Codigo principal, (poner en bucle infinito o bajo consumo)
    while(1){
    }
}

void RutinaISR(void)
{
    // Rutinas de interrupcion de pulsadores
    // Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
    // Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS
    ButtonsPoll(&ui8Changed,&ui8Buttons);
    // Las etiquetas LEFT_BUTTON, RIGHT_BUTTON, y ALL_BUTTONS estan definidas en /driverlib/buttons.h
    if(RIGHT_BUTTON & ui8Buttons){ // Boton derecho pulsado?
        rotar(1);
    }else if(LEFT_BUTTON & ui8Buttons){     // Boton izquierdo pulsado?
        rotar(-1);
    }

    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4 );
}

// Diferencial positivo es derecha, negativo es izquierda
void rotar(int8_t diferencial){
    int8_t contador;
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

void avanzar(int8_t velocidad){
    int8_t contador;
    if(velocidad > 0){
        for(contador=0; contador < velocidad ;contador ++ ){
            // Incrementa el ciclo de trabajo para el motor izquierdo y lo disminuye en el motor derecho
            if(((ui32DutyCycle[MOTOR_IZQUIERDO]+CYCLE_INCREMENTS_LEFT)<MAXCOUNT_LEFT) && ((ui32DutyCycle[MOTOR_DERECHO]-CYCLE_INCREMENTS_RIGHT)>MINCOUNT_RIGHT)){
                ui32DutyCycle[MOTOR_DERECHO] -= CYCLE_INCREMENTS_RIGHT;
                ui32DutyCycle[MOTOR_IZQUIERDO] += CYCLE_INCREMENTS_LEFT;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            }else{
                // Enciende luz roja si ha llegado a maxima velocidad en alguno de los motores
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            }
        }
    }else{
        for(contador=0; contador < abs(velocidad) ;contador ++ ){
            // Decrementa el ciclo de trabajo para el motor izquierdo y lo incrementa en el motor derecho
            if(((ui32DutyCycle[MOTOR_IZQUIERDO]-CYCLE_INCREMENTS_LEFT)>MINCOUNT_LEFT) && ((ui32DutyCycle[MOTOR_DERECHO]+CYCLE_INCREMENTS_RIGHT)<MAXCOUNT_RIGHT)){
                ui32DutyCycle[MOTOR_DERECHO] += CYCLE_INCREMENTS_RIGHT;
                ui32DutyCycle[MOTOR_IZQUIERDO] -= CYCLE_INCREMENTS_LEFT;
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0);
            }else{
                // Enciende luz roja si ha llegado a maxima velocidad en alguno de los motores
                GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
            }
        }
    }
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6, ui32DutyCycle[MOTOR_DERECHO] );
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7, ui32DutyCycle[MOTOR_IZQUIERDO] );
}

