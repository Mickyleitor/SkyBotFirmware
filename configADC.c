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

#include<stdint.h>
#include<stdbool.h>

#include "inc/hw_memmap.h"
#include "inc/hw_ints.h"
#include "driverlib/gpio.h"
#include "driverlib/sysctl.h"
#include "driverlib/interrupt.h"
#include "driverlib/adc.h"
#include "driverlib/timer.h"
#include "configADC.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"


// Este fichero implementa la configuracion del ADC y la ISR asociada. Las tareas pueden llamar a la funcion configADC_LeeADC (bloqueante) para leer datos del ADC
// La funcion configADC_DisparaADC(...) (no bloqueante) realiza el disparo software del ADC
// La funcion configADC_IniciaADC realiza la configuraci�n del ADC: Los Pines E0 a E3 se ponen como entradas anal�gicas (AIN3 a AIN0 respectivamente). Ademas crea la cola de mensajes necesaria para que la funcion configADC_LeeADC sea bloqueante


static QueueHandle_t cola_adc;
unsigned short RawValueDistance_0A41F [32] = {31,30,29,28,27,26,25,24,23,22,21,20,19,18,17,16,15,14,13,12,11,10,9,8,7,6,5,4,3,2,1,0};
unsigned short RawValueADC_0A41F [32] = {0x16C,0x174,0x184,0x190,0x198,0x1A8,0x1B8,0x1D0,0x1E8,0x1FC,0x220,0x244,0x258,0x280,0x2A8,0x2D0,0x308,0x364,0x398,0x3E8,0x448,0X4B0,0x540,0x5F0,0x708,0x7D0,0x960,0x9C8,0x6A4,0x4B0,0x398};
// unsigned short RawValueADC_0A41F [32] = {0x398,0x4B0,0x6A4,0x9C8,0x960,0x7D0,0x708,0x5F0,0x540,0x4B0,0x448,0x3E8,0x398,0x364,0x308,0x2D0,0x2A8,0x280,0x258,0x244,0x220,0x1FC,0x1E8,0x1D0,0x1B8,0x1A8,0x198,0x190,0x184,0x174,0x16C};
unsigned short RawValueDistance_0A51F [32] = {0.5,1,1.5,2,2.5,3,3.5,4,4.5,5,5.5,6,6.5,7,7.5,8,8.5,9,9.5,10,10.5,11,11.5,12,12.5,13,13.5,14,14.5,15};
// unsigned short RawValueADC_0A51F [32] = {0x498,0x764,0x8D4,0x7D0,0x6A4,0x5CC,0x528,0x488,0x428,0x3CC,0x374,0x320,0x2F0,0x2D0,0x294,0x260,0x244,0x230,0x208,0x1E8,0x1D8,0x1B8,0x1A4,0x194,0x190,0x17C,0x16C,0x15C,0x14C,0x138,0x12C};
unsigned short RawValueADC_0A51F [32] = {0x12C,0x138,0x14C,0x15C,0x16C,0x17C,0x190,0x194,0x1A4,0x1B8,0x1D8,0x1E8,0x208,0x230,0x244,0x260,0x294,0x2D0,0x2F0,0x320,0x374,0x3CC,0x428,0x488,0x528,0x5CC,0x6A4,0x7D0,0x8D4,0x764};


//Provoca el disparo de una conversion (hemos configurado el ADC con "disparo software" (Processor trigger)
void configADC0_IniciaADC(void)
{
     SysCtlPeripheralEnable(SYSCTL_PERIPH_ADC0);
     SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_ADC0);

    //HABILITAMOS EL GPIOE
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOE);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOE);
    // Enable pin PE3 for ADC AIN0|AIN1|AIN2|AIN3
    GPIOPinTypeADC(GPIO_PORTE_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);

    // IntEnable(INT_GPIOE);

    //CONFIGURAR SECUENCIADOR 1
    ADCSequenceDisable(ADC0_BASE,1);

    //Configuramos la velocidad de conversion al maximo (1MS/s)
    //1 muestra por segundo y podemos dividir la velocidad en el ultimo parametro
    ADCClockConfigSet(ADC0_BASE, ADC_CLOCK_RATE_FULL, 1);

    //Configuracion de secuencia
    //TRIGGER PROCESSOR disparar por proceso y si ponemos TRIGGER Timer disparamos por timer (NOS HARA FALTA)
    // TimerControlTrigger(TIMER2_BASE,TIMER_A,true);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_TIMER2);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_TIMER2);

    TimerConfigure(TIMER2_BASE, TIMER_CFG_PERIODIC);
    TimerLoadSet(TIMER2_BASE,TIMER_A,SysCtlClockGet()*0.000001);
    TimerControlTrigger(TIMER2_BASE,TIMER_A,true);

    ADCSequenceConfigure(ADC0_BASE,1,ADC_TRIGGER_TIMER,0); // En este momento disparo por software (en tiempo real se cambia esto para poder disparar por timer)

    ADCHardwareOversampleConfigure(ADC0_BASE,64);

    ADCSequenceStepConfigure(ADC0_BASE,1,0,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,1,1,ADC_CTL_CH1);
    ADCSequenceStepConfigure(ADC0_BASE,1,2,ADC_CTL_CH0);
    ADCSequenceStepConfigure(ADC0_BASE,1,3,ADC_CTL_CH1 | ADC_CTL_IE |ADC_CTL_END);
    ADCSequenceEnable(ADC0_BASE,1); //ACTIVO LA SECUENCIA

    //Habilita las interrupciones
    ADCIntClear(ADC0_BASE,1);
    ADCIntEnable(ADC0_BASE,1);
    IntPrioritySet(INT_ADC0SS1,configMAX_SYSCALL_INTERRUPT_PRIORITY);
    IntEnable(INT_ADC0SS1);


    //Creamos una cola de mensajes para la comunicacion entre la ISR y la tara que llame a configADC_LeeADC(...)
    cola_adc=xQueueCreate(1,sizeof(MuestrasADC));
    if (cola_adc==NULL)
    {
     while(1);
    }
    TimerEnable(TIMER2_BASE, TIMER_A);
}

void configADC0_LeeADC(MuestrasADC *datos)
{
	xQueueReceive(cola_adc,datos,portMAX_DELAY);
}

void ADC0Seq1IntHandler(void)
{
    portBASE_TYPE higherPriorityTaskWoken=pdFALSE;

    MuestrasLeidasADC leidas;
    MuestrasADC finales;

    ADCIntClear(ADC0_BASE,1);//LIMPIAMOS EL FLAG DE INTERRUPCIONES
    ADCSequenceDataGet(ADC0_BASE,1,(uint32_t *)&leidas);//COGEMOS LOS DATOS GUARDADOS

    //Pasamos de 32 bits a 16 (el conversor es de 12 bits, as� que s�lo son significativos los bits del 0 al 11)
    finales.chan1=leidas.chan1;
    finales.chan2=leidas.chan2;
    finales.chan3=leidas.chan3;
    finales.chan4=leidas.chan4;

    //Guardamos en la cola
    xQueueOverwriteFromISR(cola_adc,&finales,&higherPriorityTaskWoken);
    portEND_SWITCHING_ISR(higherPriorityTaskWoken);

    // configADC_DisparaADC();
}
