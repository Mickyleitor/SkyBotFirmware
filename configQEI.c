#include <stdbool.h>
#include <stdint.h>
#include <math.h>
#include "inc/hw_ints.h"
#include "inc/hw_memmap.h"
#include "inc/hw_qei.h"
#include "inc/hw_types.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_gpio.h"
// Libreria de control del sistema
#include "driverlib/sysctl.h"
#include "driverlib/gpio.h"
#include "driverlib/debug.h"
#include "driverlib/interrupt.h"
#include "driverlib/qei.h"
#include "driverlib/pin_map.h"
#include "FreeRTOS.h"
#include "event_groups.h"
#include "task.h"
#include "queue.h"
#include "Skybot_servos.h"

extern int direction_right,direction_left;
long CurrentTicksRight,CurrentTicksLeft;
unsigned long SpeedTicksRight,SpeedTicksLeft;
volatile unsigned long Last_TickRight,Last_TickLeft;
volatile unsigned long now;

void QEI_ISR(void){
    now = xTaskGetTickCountFromISR();
    if((GPIOIntStatus(GPIO_PORTA_BASE,true)&GPIO_PIN_2)==GPIO_PIN_2){
        GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2);
        SpeedTicksRight = (now - Last_TickRight);
        Last_TickRight = now;
        CurrentTicksRight += direction_right;
    }
    if((GPIOIntStatus(GPIO_PORTA_BASE,true)&GPIO_PIN_3)==GPIO_PIN_3){
        GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3);
        SpeedTicksLeft = (now - Last_TickLeft);
        Last_TickLeft = now;
        CurrentTicksLeft += direction_left;
    }
}

void QEI_Init (void)
{
    //Inicializa el puerto F (Para botones)
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    // La interrupcion se activa con flanco de subida como de bajada.
    GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2,GPIO_BOTH_EDGES);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);

    GPIOIntRegister(GPIO_PORTA_BASE,QEI_ISR);
    SpeedTicksRight = 12000;
    SpeedTicksLeft = 12000;
    Last_TickRight = 0;
    Last_TickLeft = 0;
}
