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
#include <stdio.h>
#include <math.h>
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
#include "usb_dev_serial.h"
#include "SkyBot_servos.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128
// Macro para "hacerlo bonito" y no reservar espacios para variables ""casi"" inutiles. (100 ms)
#define T_ANTIREBOTE (SysCtlClockGet() * 0.04)

//Globales
uint32_t g_ui32CPUUsage;
uint32_t g_ui32SystemClock;
TaskHandle_t handle = NULL;
TaskHandle_t CircuitoTask_handle = NULL;
uint8_t ui8Buttons, ui8Changed;
uint32_t ui32Period, ui32DutyCycle[2];
bool TaskInitialized = false;
uint8_t selected_circuit = 0;

void configButtons_init(void);
void configPWM_init(void);
void configUART_init(void);
void configSensores_init(void);

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
void vCircuitoTask( void *pvParameters )
{
    //
    // Funcion que hace que el robot de vueltas en cuadrado
    //
    while(1)
    {
        switch (selected_circuit) {
            case 2 : {
                avanzar(-1000);
                vTaskDelay(1*configTICK_RATE_HZ);
                girar(300);
                vTaskDelay(0.6*configTICK_RATE_HZ);
                ui32DutyCycle[MOTOR_DERECHO] = 1108;
                ui32DutyCycle[MOTOR_IZQUIERDO] = 1356;
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);
                selected_circuit = 0;
            }
            break;
            case 3 : {
                ui32DutyCycle[MOTOR_DERECHO] = 1108;
                ui32DutyCycle[MOTOR_IZQUIERDO] = 1356;
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
                PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);
                vTaskDelay(5*configTICK_RATE_HZ);
                girar(-300);
                vTaskDelay(0.6*configTICK_RATE_HZ);
            }
            break;
            default : {}
        }

    }
}


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
//Esta tarea esta definida en el fichero command.c, es la que se encarga de procesar los comandos.
//Aqui solo la declaramos para poderla crear en la funcion main.
extern void vUARTTask( void *pvParameters );

//Aqui podria definir y/o declarar otras tareas definidas en otro fichero....

int main(void){

    //
    // Set the clocking to run at 40 MHz from the PLL.
    //
    ROM_SysCtlClockSet(SYSCTL_SYSDIV_5 | SYSCTL_USE_PLL | SYSCTL_XTAL_16MHZ | SYSCTL_OSC_MAIN);

    // Get the system clock speed.
    g_ui32SystemClock = SysCtlClockGet();

    //Habilita el clock gating de los perifericos durante el bajo consumo --> Hay que decirle los perifericos que queramos que sigan andando usando la funcion SysCtlSleepEnable(...) en cada uno de ellos
    SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER5 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(g_ui32SystemClock, configTICK_RATE_HZ/10, 5);

    // configSensores_init();
    configUART_init();
    configButtons_init();
    configPWM_init();


   // Habilita interrupcion del modulo TIMER y Puerto F y E
   IntEnable(INT_TIMER0A);
   // IntEnable(INT_TIMER1A);
   IntEnable(INT_GPIOF);
   IntEnable(INT_GPIOB);
   IntMasterEnable();

   //
   // Create la tarea que gestiona los comandos (definida en el fichero commands.c)
   //
   if(xTaskCreate(vUARTTask, "Uart", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE)
   {
       while(1)
       {
       }
   }

   if(xTaskCreate(vCircuitoTask, "Circuito", 256,NULL,tskIDLE_PRIORITY + 1, CircuitoTask_handle) != pdTRUE)
   {
       while(1)
       {
       }
   }
   //
   // Arranca el  scheduler.  Pasamos a ejecutar las tareas que se hayan activado.
   //
   vTaskStartScheduler();  //el RTOS habilita las interrupciones al entrar aqui, asi que no hace falta habilitarlas

   //De la funcion vTaskStartScheduler no se sale nunca... a partir de aqui pasan a ejecutarse las tareas.
   while(1)
   {
       //Si llego aqui es que algo raro ha pasado
   }
}

void RutinaButtons_ISR(void)
{
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
    // Activamos timer para que comience tiempo de antirebote
    // (OJO, aqui solo va a entrar una vez aunque pulsemos los dos botones, porque desactivamos interrupcion!!)
    TimerEnable(TIMER0_BASE, TIMER_A);
}
void Timer0IntHandler(void)
{
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya estan pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER0_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Miramos estado de los botones
    ButtonsPoll(&ui8Changed,&ui8Buttons);
    // Las etiquetas LEFT_BUTTON, RIGHT_BUTTON, y ALL_BUTTONS estan definidas en /driverlib/buttons.h
    if(RIGHT_BUTTON & ui8Buttons){ // Boton derecho pulsado?
        if(selected_circuit == 3){
            selected_circuit = 2;
        }else{
            selected_circuit = 3;
        }
    }else if(LEFT_BUTTON & ui8Buttons){     // Boton izquierdo pulsado?
        if(selected_circuit > 0){
            ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
            ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
            selected_circuit = 0;
        }else{
            ui32DutyCycle[MOTOR_DERECHO] = 1108;
            ui32DutyCycle[MOTOR_IZQUIERDO] = 1356;
            selected_circuit = 1;
        }
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);
    }
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
}

/*
void RutinaSensores_ISR(void)
{
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_5);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTB_BASE,GPIO_PIN_5);
    // Activamos timer para que comience tiempo de antirebote
    // (OJO, aqui solo va a entrar una vez aunque pulsemos los dos botones, porque desactivamos interrupcion!!)
    TimerEnable(TIMER1_BASE, TIMER_A);
}

void Timer0IntHandler(void)
{
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya están pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER0_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);

    // Rutinas de interrupcion de pulsadores
    // Boton Izquierdo: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a  COUNT_1MS
    // Boton Derecho: modifica  ciclo de trabajo en CYCLE_INCREMENTS para el servo conectado a PF2, hasta llegar a COUNT_2MS
    ButtonsPoll(&ui8Changed,&ui8Buttons);
    // Las etiquetas LEFT_BUTTON, RIGHT_BUTTON, y ALL_BUTTONS estan definidas en /driverlib/buttons.h
    if(RIGHT_BUTTON & ui8Buttons){ // Boton derecho pulsado?

    }else if(LEFT_BUTTON & ui8Buttons){     // Boton izquierdo pulsado?
        if(selected_circuit > 0){
            vTaskSuspend(CircuitoTask_handle);
            selected_circuit = 0;
            ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
            ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
            xTaskResumeFromISR(CircuitoTask_handle);
        }else{
            selected_circuit = 1;
        }
    }
    GPIOIntClear(GPIO_PORTF_BASE, GPIO_PIN_0|GPIO_PIN_4 );
}

void Timer1IntHandler(void)
{
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya están pulsados correctamente.

    // Borra la interrupcion de Timer
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER1_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);

    // Lee estado del pulsador
    if(ROM_GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_5) == 1){
        selected_circuit = 2;
    }
    GPIOIntClear(GPIO_PORTB_BASE, GPIO_PIN_5);
}
*/

void configButtons_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    // Timer para antirebote
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    // Configuramos LED rojo para mostrar datos de DEBUG
    ROM_GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    // Configuramos pines PF0 y PF4 (Botones)
    // La interrupcion se activa con flanco como de bajada.
    ROM_GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);
    // Funcion API de la placa para inicializacion de botones (definida en /drivers/buttons).
    // Documentacion de estas funciones en EK-TM4C123GXL Firmware Development Packages Users Guide, pag 11 y 13
    ButtonsInit();
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable( GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4 );
    // Borra Interrupciones (por si acaso)
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);
}
/*
void configSensores_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_5);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // La interrupcion se activa con flanco como de bajada.
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_5,GPIO_FALLING_EDGE);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTB_BASE,GPIO_PIN_5);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_5);
    // Timer para antirebote
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
}
*/

void configPWM_init(void){
    //Configure PWM Options
    //Configure PWM Clock to match system
    ROM_SysCtlPWMClockSet(SYSCTL_PWMDIV_16);
    //Inicializa el puerto F (Para puertos PWM)
    //The Tiva Launchpad has two modules (0 and 1). Module 1 covers the PF2 and PF3
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_PWM1);
    // Configuramos PF2 y PF3 como PWM
    ROM_GPIOPinTypePWM(GPIO_PORTF_BASE, GPIO_PIN_2 | GPIO_PIN_3);
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
}

void configUART_init(void){
    //
    // Inicializa la UARTy la configura a 115.200 bps, 8-N-1 .
    //se usa para mandar y recibir mensajes y comandos por el puerto serie
    // Mediante un programa terminal como gtkterm, putty, cutecom, etc...
    //
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);
    //Inicializa el puerto A para UART
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_GPIOPinConfigure(GPIO_PA0_U0RX);
    ROM_GPIOPinConfigure(GPIO_PA1_U0TX);
    ROM_GPIOPinTypeUART(GPIO_PORTA_BASE, GPIO_PIN_0 | GPIO_PIN_1);

    //Esta funcion habilita la interrupcion de la UART y le da la prioridad adecuada si esta activado el soporte para FreeRTOS
    UARTStdioConfig(0,115200,SysCtlClockGet());
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_UART0);   // La UART tiene que seguir funcionando aunque el micro esta dormido
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);   // El GPIO A tiene que seguir funcionando aunque el micro este dormido
}
