#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_types.h"
#include "inc/hw_ints.h"
#include "inc/hw_adc.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/rom.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "configOOFS.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "Skybot_servos.h"

extern EventGroupHandle_t FlagsAlarm;
#define T_ANTIREBOTE (SysCtlClockGet() * 0.04)
volatile uint8_t valor;

void GPIOPortBIntHandler(void)
{
    valor = GPIOIntStatus(GPIO_PORTB_BASE,true);
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
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
    // GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1 | GPIO_PIN_0,GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_1 | GPIO_PIN_0));
    valor = GPIOPinRead(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // xEventGroupSetBitsFromISR(FlagsAlarm,0xF0,&xHigherPriorityTaskWoken);
    /*
    if(valor > 0){
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
    }
    */
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void configOOFS_init(){
    //Inicializa el puerto B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);
    // Timer para antirebote
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER1);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER1);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER1_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER1_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER1_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER1_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER1A);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE, GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // La interrupcion se activa con flanco como de bajada.
    GPIOIntTypeSet(GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0,GPIO_BOTH_EDGES);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTB_BASE,GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

    IntEnable(INT_GPIOB);
}
