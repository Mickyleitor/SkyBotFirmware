#include <stdbool.h>
#include <stdint.h>
#include <math.h>

// Libreria de control del sistema
#include "inc/hw_memmap.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/timer.h"
#include "Skybot_servos.h"
#include "configQEI.h"

extern int direction[2];
long CurrentTicks[2];
// unsigned long SpeedTicks[2];
// volatile unsigned long Last_Tick[2];
volatile unsigned long now;

void QEI_ISR(void){
    // now = xTaskGetTickCountFromISR();
    if((GPIOIntStatus(GPIO_PORTA_BASE,true)&GPIO_PIN_2)==GPIO_PIN_2){
        GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_2);
        // SpeedTicks[MOTOR_DERECHO] = (now - Last_Tick[MOTOR_DERECHO]);
        // Last_Tick[MOTOR_DERECHO] = now;
        CurrentTicks[MOTOR_DERECHO] += direction[MOTOR_DERECHO];
    }
    if((GPIOIntStatus(GPIO_PORTA_BASE,true)&GPIO_PIN_3)==GPIO_PIN_3){
        GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3);
        // SpeedTicks[MOTOR_IZQUIERDO] = (now - Last_Tick[MOTOR_IZQUIERDO]);
        // Last_Tick[MOTOR_IZQUIERDO] = now;
        CurrentTicks[MOTOR_IZQUIERDO] += direction[MOTOR_IZQUIERDO];
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

    /*
    SpeedTicks[MOTOR_IZQUIERDO] = 12000;
    SpeedTicks[MOTOR_DERECHO] = 12000;
    Last_Tick[MOTOR_IZQUIERDO] = 0;
    Last_Tick[MOTOR_DERECHO] = 0;
    */
}
