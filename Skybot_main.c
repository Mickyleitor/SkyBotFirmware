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
// Incluir librerias de perifÃ©rico y otros que se vaya a usar para control PWM y gestion de botones
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
#include "configQEI.h"
#include "configOOFS.h"
#include "configWhisker.h"
#include "configButtons.h"

#define RADIO_TARIMA 45
// #define DEBUG_MODE

// Modos de trabajo
#define TEST -1
#define BUSQUEDA 0
#define OOFS_SR_DER 1
#define OOFS_SR_IZQ 2
#define OOFS_SL_DER 3
#define OOFS_SL_IZQ 4
#define ATAQUE 5
#define EMPUJAR 6


//Globales
uint32_t g_ui32CPUUsage;
EventGroupHandle_t FlagsAlarm,TickServoDone;
QueueHandle_t QueueServoTicksRequest[2];
QueueHandle_t QueueServoTicksDone[2];
QueueHandle_t QueueServoSpeed[2];
extern long CurrentTicks[2];
float CurrentLongRange = 0;
float CurrentShortRange = 0;
int FSM_Mode = BUSQUEDA;

void configUART_init(void);
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

#ifndef DEBUG_MODE
// Tarea que envia recoge los datos del ADC y realiza la media
static portTASK_FUNCTION(ADCTask,pvParameters)
{
    // Tarea encargada de monitorizar los sensores que necesiten ADC en tiempo real
    // Se dispara timer para la conversión
    MuestrasADC muestras;
    while(1)
    {
        configADC0_LeeADC(&muestras);    //Espera y lee muestras del ADC (BLOQUEANTE)
        // Recogemos muestra y hacemos conversión lineal (formula aplicada a region lineal)
        CurrentLongRange =  (((muestras.chan1 + muestras.chan3)*0.5) - 1566 ) / ( -31.067 );
        CurrentShortRange =  (((muestras.chan2 + muestras.chan4)*0.5) - 1732 ) / ( -80.5 );
        // UARTprintf("Long: [%d] - Short [%d]\n",(int)CurrentLongRange,(int)CurrentShortRange);

        vTaskDelay(portTICK_PERIOD_MS);
    }
}
static portTASK_FUNCTION(FSMTask,pvParameters)
{
    long TimeOutSearch = xTaskGetTickCount();
    long TimeOutAttacking = xTaskGetTickCount();
    float distancia_seguridad = 35;
    float distancia_ataque = 15;
    int direccion = 1;
    int FastSearchs = 0;
    while(1)
    {
        switch (FSM_Mode) {
            // Modo busqueda
            case BUSQUEDA : {
                acelerar_velocidad(100,100);
                mover_robot_IT(20000);
                FastSearchs = 0;
                while(FSM_Mode == BUSQUEDA){
                    if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL)){
                        // Si detecta que se ha salido del tatami, pasar al modo OOFS (Out Of Field System)
                        FSM_Mode = OOFS_SL_DER;
                        mover_robot_IT(10000);
                    }else if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FR)){
                        // Si detecta que se ha salido del tatami, pasar al modo OOFS (Out Of Field System)
                        FSM_Mode = OOFS_SR_IZQ;
                        mover_robot_IT(10000);
                    }else if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_BL) == SENSOR_BL){
                        // Si detecta que un whisker trasero se ha activado, girar hacia ese lado y pasar la modo ataque
                        acelerar_velocidad(100,100);
                        FSM_Mode = ATAQUE;
                        girar_robot(-125);
                        mover_robot_IT(10000);
                    }else if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_BR) == SENSOR_BR){
                        // Si detecta que un whisker trasero se ha activado, girar hacia ese lado y pasar la modo ataque
                        FSM_Mode = ATAQUE;
                        acelerar_velocidad(100,100);
                        girar_robot(125);
                        mover_robot_IT(10000);
                    }else if((xTaskGetTickCount() - TimeOutSearch) > 5000){
                        // Si se ha tirado 5 segundos buscando sin resultado, avanzar un poco
                        acelerar_velocidad(100,100);
                        TimeOutSearch = xTaskGetTickCount();
                        while(((xTaskGetTickCount() - TimeOutSearch) < 2000)&&(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FR | SENSOR_FL) == 0)){
                            mover_robot_IT(1000);
                            vTaskDelay(portTICK_PERIOD_MS);
                        }
                        // mover_robot(RADIO_TARIMA/2);
                        TimeOutSearch = xTaskGetTickCount();
                        FastSearchs ++;
                        if(FastSearchs > 1){
                            // Si se ha tirado 15 segundos buscando sin resultado, realizar busqueda lenta
                            FastSearchs = 0;
                            acelerar_velocidad(30,30);
                        }
                    }else if(CurrentLongRange <= distancia_seguridad){
                        // Si ha detectado obstaculo con el sensor de distancia, pasar al modo ataque
                        FSM_Mode = ATAQUE;
                    }else{
                        // Si no ha ocurrido nada hast aqui, continuar girando
                        girar_robot_IT(180*direccion);
                        vTaskDelay(portTICK_PERIOD_MS);
                    }
                }
                break;
            }
            case ATAQUE : {
                mover_robot_IT(20000);
                while(CurrentLongRange < distancia_seguridad && (FSM_Mode == ATAQUE )&& (GPIOPinRead(GPIO_PORTB_BASE,SENSOR_BL | SENSOR_BR) == 0)){
                    // Mientras se detecte obstaculo, este en modo ataque sin que los sensores TCRT1000 traseros se activen...
                    // Avanzar de manera moderada
                    acelerar_velocidad(50,50);
                    if(CurrentShortRange < distancia_ataque){
                        // Si el obstaculo se encuentra muy cerca, incrementar velocidad al maximo
                        direccion = -direccion;
                        acelerar_velocidad(100,100);
                        TimeOutSearch = xTaskGetTickCount();
                        if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FL | SENSOR_FR) > 0){
                            // Si estamos chocando con el obstaculo , pasar al modo empuje
                            TimeOutAttacking = xTaskGetTickCount();
                            FSM_Mode = EMPUJAR;
                        }
                    }
                    vTaskDelay(portTICK_PERIOD_MS);
                }
                if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_BL | SENSOR_BR) > 0){
                    /*
                    if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL | SENSOR_FR) == (SENSOR_FL | SENSOR_FR)){
                        // Aqui el robot "cree" que ha expulsado al contrincante y se va de nuevo al centro de la tarima
                        mover_robot(-RADIO_TARIMA);
                        FSM_Mode = BUSQUEDA;
                    }
                    */
                    mover_robot(-RADIO_TARIMA);
                    // FSM_Mode = BUSQUEDA;
                }
                FSM_Mode = BUSQUEDA;
                break;
            }
            case EMPUJAR : {
                mover_robot_IT(20000);
                while(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FL | SENSOR_FR) > 0){
                    // Mientras tengamos contacto directo con los whiskers...
                    acelerar_velocidad(100,100);
                    if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FL | SENSOR_FR) == (SENSOR_FL | SENSOR_FR)){
                        // Si ambos whisker, avanzar con velocidad máxima recto
                        acelerar_velocidad(100,100);
                    }else if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FL) == SENSOR_FL){
                        // Si solo whisker izquierdo, avanzar con velocidad máxima recto y ligeramente a la izquierda
                        acelerar_velocidad(30,120);
                    }else if(GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FR) == SENSOR_FR){
                        // Si solo whisker derecho, avanzar con velocidad máxima recto y ligeramente a la derecha
                        acelerar_velocidad(120,30);
                    }
                    if((xTaskGetTickCount() - TimeOutAttacking) > 10000){
                        if( GPIOPinRead(GPIO_PORTB_BASE,SENSOR_BL | SENSOR_BR) == 0){
                            // Si se ha mantenido empujando durante 10 segundos y uno de los sensores TCRT1000 se encuentra
                            // en el borde del tatami, girar 360 para intentar desempatar la situacion
                            acelerar_velocidad(100,100);
                            girar_robot(direccion*360);
                            mover_robot_IT(20000);
                        }
                        TimeOutAttacking = xTaskGetTickCount();
                    }
                    vTaskDelay(portTICK_PERIOD_MS);
                }
                acelerar_velocidad(100,100);
                FSM_Mode = ATAQUE;
                break;
            }
            case OOFS_SL_DER : {
                // Detectado robot con TCRT1000 izquierdo delantero en el borde,
                // Intentar colocarse en un angulo de 135 con respecto al centro del tatami
                if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FR)){
                    acelerar_velocidad(80,1);
                    FSM_Mode = OOFS_SL_IZQ;
                }else if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL) == 0){
                    FSM_Mode = BUSQUEDA;
                }
                break;
            }
            case OOFS_SL_IZQ : {
                // Detectado robot con TCRT1000 derecho delantero en el borde
                if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FR) == 0){
                    acelerar_velocidad(1,80);
                    FSM_Mode = OOFS_SL_DER;
                }else if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_BL)){
                    // En este punto, el robot se encuentra a 135 grados con respecto al centro del tatami.
                    // Por lo que gira y avanza hacia el centro para volver a pasar al modo de busqueda
                    acelerar_velocidad(80,80);
                    girar_robot(135);
                    mover_robot(RADIO_TARIMA);
                    FSM_Mode = BUSQUEDA;
                }
                break;
            }
            case OOFS_SR_IZQ : {
                // Detectado robot con TCRT1000 derecho delantero en el borde,
                // Intentar colocarse en un angulo de 135 con respecto al centro del tatami
                if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL)){
                    acelerar_velocidad(1,80);
                    FSM_Mode = OOFS_SR_DER;
                }else if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FR) == 0){
                    FSM_Mode = BUSQUEDA;
                }
                break;
            }
            case OOFS_SR_DER : {
                // Detectado robot con TCRT1000 izquierdo delantero en el borde
                if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL) == 0){
                    acelerar_velocidad(80,1);
                    FSM_Mode = OOFS_SR_IZQ;
                }else if(GPIOPinRead(GPIO_PORTB_BASE,SENSOR_BR)){
                    // En este punto, el robot se encuentra a 135 grados con respecto al centro del tatami.
                    // Por lo que gira y avanza hacia el centro para volver a pasar al modo de busqueda
                    acelerar_velocidad(80,80);
                    girar_robot(-135);
                    mover_robot(RADIO_TARIMA);
                    FSM_Mode = BUSQUEDA;
                }
                break;
            }
        }
        vTaskDelay(portTICK_PERIOD_MS);
    }
}
#endif
static portTASK_FUNCTION(ServoTask,pvParameters)
{
    // Tarea doble para cada motor, aqui se reciben las ordenes y se acusan las ordenes
    // Las ordenes son de posicion relativa y velocidad del servo ("Request" y "Speed")
    // Solo se acusa las posiciones (ticks) del servo. Se pueden sobreescribir ordenes incluso si
    // Aún no ha terminado la tarea anterior (lo que se haya hecho se envía por la cola "Done")
    int pos_setpoint = 0;
    int error = 0;
    int speed_setpoint = 100;
    int myMotor = *((int *)pvParameters);
    while(1)
    {
        if(xQueueReceive(QueueServoTicksRequest[myMotor],&pos_setpoint,portTICK_PERIOD_MS)==pdTRUE){
            int itemToSend = CurrentTicks[myMotor];
            xQueueOverwrite(QueueServoTicksDone[myMotor],&itemToSend);
            CurrentTicks[myMotor] = 0;
        }
        error = pos_setpoint - CurrentTicks[myMotor];
        if(abs(error)>0){
            xQueueReceive(QueueServoSpeed[myMotor],&speed_setpoint,portTICK_PERIOD_MS);
            if(error>0)  acelerar_motor(myMotor,speed_setpoint);
            else acelerar_motor(myMotor,-speed_setpoint);
        }else{
            if(!motor_stopped(myMotor)){
                acelerar_motor(myMotor,0);
                int itemToSend = CurrentTicks[myMotor];
                xQueueOverwrite(QueueServoTicksDone[myMotor],&itemToSend);
                xEventGroupSetBits(TickServoDone,1 << myMotor);
                xQueuePeek( QueueServoTicksRequest[myMotor], &pos_setpoint, portMAX_DELAY);
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

    configUART_init();
    configServos_init();
    QEI_Init();
    configButtons_init();
#ifndef DEBUG_MODE
    configADC0_IniciaADC();
#endif
    configOOFS_init();
    configWhisker_init();

   // Habilita interrupcion del master
   IntMasterEnable();

   FlagsAlarm=xEventGroupCreate();
   if(FlagsAlarm == NULL) while(1);

   TickServoDone=xEventGroupCreate();
   if(TickServoDone == NULL) while(1);

   SysCtlDelay( (SysCtlClockGet() * 2));

   int i;
   for(i = 0; i < 2; i++){
       QueueServoTicksRequest[i]=xQueueCreate(1,sizeof(int));
       if (QueueServoTicksRequest[i]==NULL)  while(1);

       QueueServoTicksDone[i]=xQueueCreate(1,sizeof(int));
       if (QueueServoTicksDone[i]==NULL)  while(1);

       QueueServoSpeed[i]=xQueueCreate(1,sizeof(int));
       if (QueueServoSpeed[i]==NULL)  while(1);

       servo[i] = i;
       if(xTaskCreate(ServoTask, "Servo", 128,(void*)&servo[i],tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   }
   //
   // Create la tarea que gestiona los comandos (definida en el fichero commands.c)
   //
   if(xTaskCreate(vUARTTask, "Uart", 256,NULL,tskIDLE_PRIORITY + 1, NULL) != pdTRUE){ while(1); }
#ifndef DEBUG_MODE
   if(xTaskCreate(ADCTask, "ADC", 128,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
   if(xTaskCreate(FSMTask, "FSM Task", 128,NULL,tskIDLE_PRIORITY + 2, NULL) != pdTRUE){ while(1); }
#endif

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

void Timer3AIntHandler(void){
    TimerIntClear(TIMER3_BASE,TIMER_TIMA_TIMEOUT);
    GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,~GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1));
    switch (FSM_Mode) {
        case -1 : {
            TimerLoadSet(TIMER3_BASE, TIMER_A, (SysCtlClockGet() * 0.05) -1);
            break;
        }
        case 0 : {
            TimerLoadSet(TIMER3_BASE, TIMER_A, (SysCtlClockGet() * 2) -1);
            break;
        }
        default : {
            TimerLoadSet(TIMER3_BASE, TIMER_A, (SysCtlClockGet() * 0.5) -1);
            break;
        }
    }
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
