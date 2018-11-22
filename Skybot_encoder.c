#include "Skybot_encoder.h"

void configEncoders_init(void){
    //Inicializa el puerto F (Para botones)
    ROM_SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    ROM_SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOA);

    ROM_GPIOPinTypeGPIOInput(GPIO_PORTA_BASE, GPIO_PIN_3 | GPIO_PIN_2);
    // ROM_GPIOPadConfigSet(GPIO_PORTA_BASE,GPIO_PIN_3,GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD_WPU);
    // La interrupcion se activa con flanco como de bajada.
    ROM_GPIOIntTypeSet(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2,GPIO_BOTH_EDGES);
    // Y habilita, dentro del modulo GPIO, la interrupcion de particular del boton
    GPIOIntEnable (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);
    // Borra Interrupciones (por si acaso)
    GPIOIntClear (GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);

    IntEnable(INT_GPIOA);
}

void RutinaEncoders_ISR(void)
{
    uint32_t masked2 = (GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_2);
    uint32_t masked3 = (GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_3);
    if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_2){
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_1,!ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_1) * 255 );
    }else if(GPIOIntStatus(GPIO_PORTA_BASE,true) & GPIO_PIN_3){
        ROM_GPIOPinWrite(GPIO_PORTF_BASE,GPIO_PIN_2,!ROM_GPIOPinRead(GPIO_PORTF_BASE,GPIO_PIN_2) * 255 );
    }
    GPIOIntClear(GPIO_PORTA_BASE,GPIO_PIN_3 | GPIO_PIN_2);
}
