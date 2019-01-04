/* ************************************************************************** */
/* UNIVERSIDAD DE MALAGA               DEPARTAMENTO DE TECNOLOGIA ELECTRONICA */
/* http://www.uma.es                                    http://www.dte.uma.es */
/* ========================================================================== */
/* PROGRAMA :  PWM-Servo                                                      */
/* VERSION  : 1.0                                                             */
/* TARGET   : Kit  TIVA Launchpad IDE CCSv7                                   */
/* DESCRIPCION : Este programas genera dos salidas PWM a trav√©s de los        */
/* terminales PF2 y PF3 usando el Timer adecuado en modo PWM, o un modulo PWM */
/*  Al pulsar los botones de la placa, deber√±a aumentar/reducir el ciclo de   */
/*  trabajo, provocando un aumento/reducci√≥n de la velocidad e incluso cambio */
/*  sentido                                                                   */
/* RECURSOS :                                                                 */
/* AUTOR    : Ignacio Herrero Reder (iherrero@uma.es)                         */
/* FECHA    : 08/10/17                                                        */
/* COMENTARIOS  : 1 tabulador = 8 espacios                                    */

/* **************************************************************************   */
/* **************************************************************************   */
/* (ver cabecera en el c√≥digo del programa)                                     */
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
#include "inc/hw_gpio.h"
#include "driverlib/pin_map.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
// Incluir librerias de perif√©rico y otros que se vaya a usar para control PWM y gestion de botones
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
#include "event_groups.h"
#include "queue.h"
#include "semphr.h"
#include "utils/cpu_usage.h"
#include "SkyBot_servos.h"
#include "configADC.h"
#include "driverlib/qei.h"
#include "configQEI.h"

#define LED1TASKPRIO 1
#define LED1TASKSTACKSIZE 128
#define T_ANTIREBOTE (SysCtlClockGet() * 0.04)


//Globales
uint32_t g_ui32CPUUsage;
EventGroupHandle_t FlagsAlarm,TickServoDone;
QueueHandle_t QueueServoTicksRequest[2];
QueueHandle_t QueueServoTicksDone[2];
QueueHandle_t QueueServoSpeed[2];
extern long CurrentTicks[2];
float CurrentLongRange = 0;
float CurrentShortRange = 0;
float CurrentRange = 100;

void configButtons_init(void);
void configPWM_init(void);
void configUART_init(void);
void configSensores_init(void);
void QEI_Init(void);
float map[120][120];
int servo[2];

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
static portTASK_FUNCTION(ButtonsTask,pvParameters)
{
    uint8_t WakedUpBitsGroup  = 0;
    int i,y;
    int turn = 0;
    //
    // Funcion que se encarga de los interacciones con los botones (principalmente para debug)
    //
    while(1)
    {
        WakedUpBitsGroup= xEventGroupWaitBits(FlagsAlarm,ALL_BUTTONS,pdTRUE,pdFALSE,portMAX_DELAY);
        if((WakedUpBitsGroup & LEFT_BUTTON) > 0){
            // Circuito 18x12 4 veces
            for(i=0;i<4;i++){
                // Recta de 18 cm
                mover_robot(18);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(50,50);
                girar_robot(90);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(100,100);

                // Recta de 12 cm
                mover_robot(12);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(50,50);
                girar_robot(90);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(100,100);

                // Recta de 18 cm
                mover_robot(18);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(50,50);
                girar_robot(90);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(100,100);

                // Recta de 12 cm
                mover_robot(12);
                vTaskDelay(500 * portTICK_PERIOD_MS);
                acelerar_velocidad(50,50);
                girar_robot(90);
            }
        }
        if((WakedUpBitsGroup & RIGHT_BUTTON) > 0){
            #define giros 12
            float closeUp[360/giros];
            float max_value = 0;
            for(y=0;y<10;y++){
                max_value = 0;
                turn = 0;
                acelerar_velocidad(70,70);
                for(i=0;i<360/giros;i++){
                    girar_robot(giros);
                    vTaskDelay(100*portTICK_PERIOD_MS);
                    if(i>0){
                        closeUp[i] = (CurrentRange + closeUp[i-1])/2;
                    }else{
                        closeUp[i] = CurrentRange;
                    }
                    if(closeUp[i] > max_value){
                        max_value = closeUp[i];
                        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
                        turn = (i+1)*giros;
                    }else{
                        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
                    }
                }
                for(i=0;i<360/giros;i++){
                    UARTprintf("%d,",(int)closeUp[i]);
                }
                UARTprintf("\n");
                UARTprintf("Free Space at %d by %d\n",turn,(int)max_value);
                if(turn > 180){
                    girar_robot(turn-360);
                }else{
                    girar_robot(turn);
                }
                acelerar_velocidad(100,100);
                mover_robot(max_value/2);
            }
        }

    }
}
/*
static portTASK_FUNCTION(ProbMapTask,pvParameters)
{
    // Funcion que se encarga de un mapeado probabilistico en tiempo real del terreno,
    // retroalimentado por los sensores de distancia
    int x=0;
    int y=0;
    for(x = 0; x < 120;x++){
        for(y=0; y < 120;y++){
            map[x][y]=1;
        }
    }
    while(1)
    {
        vTaskDelay(1000*portTICK_PERIOD_MS);
    }
}
static portTASK_FUNCTION(ServoMainTask,pvParameters)
{
    // Esta tarea controlar· los dos motores.
    while(1)
    {
        vTaskDelay(1000*portTICK_PERIOD_MS);
    }
}
*/
// Tarea que envia recoge los datos del ADC y realiza la media
static portTASK_FUNCTION(ADCTask,pvParameters)
{
    // Tarea encargada de monitorizar los sensores que necesiten ADC en tiempo real
    // Se dispara timer para la conversiÛn
    MuestrasADC muestras;
    TimerEnable(TIMER2_BASE,TIMER_A);
    while(1)
    {
        configADC_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        CurrentLongRange = (muestras.chan1 - 1028.9)/(-23.453) + 10;
        CurrentShortRange =(muestras.chan2 - 1045.3)/(-53.371) + 4;

        CurrentRange = (CurrentLongRange <= 15) ? CurrentShortRange : CurrentLongRange;
        // UARTprintf("%d\n",muestras.chan3);
    }
}
static portTASK_FUNCTION(ServoTask,pvParameters)
{
    // Tarea doble para cada motor, aqui se reciben las ordenes y se acusan las ordenes
    // Las ordenes son de posicion relativa y velocidad del servo ("Request" y "Speed")
    // Solo se acusa las posiciones (ticks) del servo. Se pueden sobreescribir ordenes incluso si
    // A˙n no ha terminado la tarea anterior (lo que se haya hecho se envÌa por la cola "Done")
    int pos_setpoint = 0;
    int pos_current = 0;
    int error = 0;
    int speed_setpoint = 100;
    int myMotor = *((int *)pvParameters);
    while(1)
    {
        if(xQueueReceive(QueueServoTicksRequest[myMotor],&pos_setpoint,portTICK_PERIOD_MS)==pdTRUE){
            xQueueSend(QueueServoTicksDone[myMotor],&error,portMAX_DELAY);
            CurrentTicks[myMotor] = 0;
        }
        pos_current = CurrentTicks[myMotor];
        error = pos_setpoint - pos_current;
        if(abs(error)>0){
            xQueueReceive(QueueServoSpeed[myMotor],&speed_setpoint,portTICK_PERIOD_MS);
            if(error>0)  acelerar_motor(myMotor,speed_setpoint);
            else acelerar_motor(myMotor,-speed_setpoint);
        }else{
            if(!motor_stopped(myMotor)){
                acelerar_motor(myMotor,0);
                xQueueSend(QueueServoTicksDone[myMotor],&pos_setpoint,portMAX_DELAY);
                xEventGroupSetBits(TickServoDone,1 << myMotor);
            }
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

    //Habilita el clock gating de los perifericos durante el bajo consumo --> Hay que decirle los perifericos que queramos que sigan andando usando la funcion SysCtlSleepEnable(...) en cada uno de ellos
    SysCtlPeripheralClockGating(true);

    // Inicializa el subsistema de medida del uso de CPU (mide el tiempo que la CPU no esta dormida)
    // Para eso utiliza un timer, que aqui hemos puesto que sea el TIMER5 (ultimo parametro que se pasa a la funcion)
    // (y por tanto este no se deberia utilizar para otra cosa).
    CPUUsageInit(SysCtlClockGet(), configTICK_RATE_HZ/10, 5);

    configSensores_init();
    configUART_init();
    configButtons_init();
    configServos_init();
    configADC_IniciaADC();
    QEI_Init();

   // Habilita interrupcion del master
   IntMasterEnable();

   FlagsAlarm=xEventGroupCreate();
   if(FlagsAlarm == NULL) while(1);

   TickServoDone=xEventGroupCreate();
   if(TickServoDone == NULL) while(1);

   int i;
   for(i = 0; i < 2; i++){
       QueueServoTicksRequest[i]=xQueueCreate(10,sizeof(int));
       if (QueueServoTicksRequest[i]==NULL)  while(1);

       QueueServoTicksDone[i]=xQueueCreate(10,sizeof(int));
       if (QueueServoTicksDone[i]==NULL)  while(1);

       QueueServoSpeed[i]=xQueueCreate(10,sizeof(int));
       if (QueueServoSpeed[i]==NULL)  while(1);

       servo[i] = i;
       if(xTaskCreate(ServoTask, "Servo", 128,(void*)&servo[i],tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   }
   //
   // Create la tarea que gestiona los comandos (definida en el fichero commands.c)
   //
   if(xTaskCreate(vUARTTask, "Uart", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE){ while(1); }
   if(xTaskCreate(ButtonsTask, "Botones", 256,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   if(xTaskCreate(ADCTask, "ADC", 128,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   // if(xTaskCreate(ServoMainTask, "ServoMain", 128,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   // if(xTaskCreate(ProbMapTask, "ProbMap", 128,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }

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
void Timer0AIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya estan pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER0_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);
    uint8_t uiChanged, uiButtons;
    ButtonsPoll(&uiChanged,&uiButtons);
    if(RIGHT_BUTTON & uiButtons){
        xEventGroupSetBitsFromISR(FlagsAlarm,RIGHT_BUTTON,&xHigherPriorityTaskWoken);
    }else if(LEFT_BUTTON & uiButtons){
        xEventGroupSetBitsFromISR(FlagsAlarm,LEFT_BUTTON,&xHigherPriorityTaskWoken);
    }
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
void RutinaSensores_ISR(void)
{
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_0);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTB_BASE,GPIO_PIN_0);
    // Activamos timer para que comience tiempo de antirebote
    // (OJO, aqui solo va a entrar una vez aunque pulsemos los dos botones, porque desactivamos interrupcion!!)
    TimerEnable(TIMER1_BASE, TIMER_A);
}
void Timer1AIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya estan pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER1_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    xEventGroupSetBitsFromISR(FlagsAlarm,0b100,&xHigherPriorityTaskWoken);
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void configButtons_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    // Timer para antirebote
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor ser√° el de antirebote.
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER0A);

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

    IntEnable(INT_GPIOF);
}

void configSensores_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    // Timer para antirebote
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor ser√° el de antirebote.
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER1A);

    ROM_GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_0);
    ROM_GPIOPadConfigSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // La interrupcion se activa con flanco como de bajada.
    ROM_GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_0,GPIO_FALLING_EDGE);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTB_BASE,GPIO_PIN_0);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_0);

    IntEnable(INT_GPIOB);
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
