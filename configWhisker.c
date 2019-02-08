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
#include "configWhisker.h"

void GPIOPortDIntHandler(void)
{
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Desactivamos interrupcion (hasta que pase el tiempo para antirebote)
    GPIOIntDisable(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Activamos timer para que comience tiempo de antirebote
    // (OJO, aqui solo va a entrar una vez aunque pulsemos los dos botones, porque desactivamos interrupcion!!)
    TimerEnable(TIMER4_BASE, TIMER_A);
}
void Timer4AIntHandler(void)
{
    // BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    // Aqui se "supone" que ha pasado el tiempo de antirebote y ambos|uno de los botones ya estan pulsados correctamente.
    // Borra la interrupcion de Timer
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    // Desactivamos el Timer para que no vuelva a saltar este ISR.
    TimerDisable(TIMER4_BASE, TIMER_A);
    // Recargamos el Timer a 0
    TimerLoadSet(TIMER4_BASE, TIMER_A, T_ANTIREBOTE -1);
    if( GPIOPinRead(GPIO_PORTD_BASE,SENSOR_FL | SENSOR_FR) > 0){
        // xEventGroupSetBitsFromISR(FlagsAlarm,0b100,&xHigherPriorityTaskWoken);
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,GPIO_PIN_1);
    }else{
        GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,0);
    }
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void configWhisker_init(){
    //Inicializa el puerto B
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);
    // Timer para antirebote
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER4);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER4);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER4_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER4_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER4_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER4_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER4A);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Temporal para practicar con circuito de casa
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
    // La interrupcion se activa con flanco como de bajada.
    GPIOIntTypeSet(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0,GPIO_FALLING_EDGE);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

    IntEnable(INT_GPIOD);
}
