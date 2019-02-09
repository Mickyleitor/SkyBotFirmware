#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "configOOFS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#define T_ANTIREBOTE (SysCtlClockGet() * 0.02)
#define SENSOR_FL GPIO_PIN_3
#define SENSOR_FR GPIO_PIN_0
#define SENSOR_BL GPIO_PIN_2
#define SENSOR_BR GPIO_PIN_1

extern EventGroupHandle_t FlagsAlarm;
extern int FSM_Mode;


void GPIOPortBIntHandler(void)
{
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Activamos timer para que comience tiempo de antirebote
    // (OJO, aqui solo va a entrar una vez aunque pulsemos los dos botones, porque desactivamos interrupcion!!)
    TimerEnable(TIMER1_BASE, TIMER_A);
}
void Timer1AIntHandler(void)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los pines ya estan pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER1_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    if( GPIOPinRead(GPIO_PORTB_BASE,SENSOR_FL | SENSOR_FR) > 0){
        xEventGroupSetBitsFromISR(FlagsAlarm,0b100,&xHigherPriorityTaskWoken);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
    }
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void configOOFS_init(){
    //Inicializa el puerto B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    // Timer para antirebote
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    // IntEnable(INT_TIMER1A);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Temporal para practicar con circuito de casa
    GPIOPadConfigSet(GPIO_PORTB_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
    // La interrupcion se activa con flanco como de bajada.
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0,GPIO_RISING_EDGE);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

    // IntEnable(INT_GPIOB);
}
