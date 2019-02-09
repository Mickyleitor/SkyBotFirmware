#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/pin_map.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/timer.h"
#include "driverlib/buttons.h"
#include "configButtons.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "event_groups.h"

#define T_ANTIREBOTE (SysCtlClockGet() * 0.02)

extern EventGroupHandle_t FlagsAlarm;

void GPIOPortFIntHandler(void)
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
    if(( (RIGHT_BUTTON | LEFT_BUTTON) & uiButtons ) == (RIGHT_BUTTON | LEFT_BUTTON)){
        xEventGroupSetBitsFromISR(FlagsAlarm,0b11,&xHigherPriorityTaskWoken);
    }else if(RIGHT_BUTTON & uiButtons){
        xEventGroupSetBitsFromISR(FlagsAlarm,0b1,&xHigherPriorityTaskWoken);
    }else if(LEFT_BUTTON & uiButtons){
        xEventGroupSetBitsFromISR(FlagsAlarm,0b10,&xHigherPriorityTaskWoken);
    }
    // Borramos mascara de interrupcion del puerto
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    // Activamos interrupcion de los puertos para "coger nueva secuencia"
    GPIOIntEnable(GPIO_PORTF_BASE,GPIO_PIN_4|GPIO_PIN_0);
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void configButtons_init(void){
    //Inicializa el puerto F (Para botones)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOF);
    // Timer para antirebote
    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER0);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER0);
    // Configura el Timer0 para cuenta periodica de 32 bits
    TimerConfigure(TIMER0_BASE, TIMER_CFG_PERIODIC);
    // Carga la cuenta en el Timer0A. El valor será el de antirebote.
    TimerLoadSet(TIMER0_BASE, TIMER_A, T_ANTIREBOTE -1);
    // Borra mascara de interrupciones (por si acaso)
    TimerIntClear(TIMER0_BASE, TIMER_TIMA_TIMEOUT);
    // Y habilita, dentro del modulo TIMER0, la interrupcion de particular de "fin de cuenta" y lo mismo para los puertos
    TimerIntEnable(TIMER0_BASE, TIMER_TIMA_TIMEOUT);

    IntEnable(INT_TIMER0A);

    // Configuramos pines PF0 y PF4 (Botones)
    // La interrupcion se activa con flanco como de bajada.
    GPIOIntTypeSet(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4,GPIO_FALLING_EDGE);
    // Funcion API de la placa para inicializacion de botones (definida en /drivers/buttons).
    // Documentacion de estas funciones en EK-TM4C123GXL Firmware Development Packages Users Guide, pag 11 y 13
    ButtonsInit();
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable( GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4 );
    // Borra Interrupciones (por si acaso)
    GPIOIntClear(GPIO_PORTF_BASE,GPIO_PIN_0|GPIO_PIN_4);

    IntEnable(INT_GPIOF);
}
