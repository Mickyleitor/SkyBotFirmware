#include <stdbool.h>
#include <stdint.h>
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

volatile static int dummy = 0;

#define ONE_SEC (16000000)

void QEI0_ISR(void){
    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    dummy ++;
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}
void QEI1_ISR(void){
    QEIIntClear(QEI1_BASE, QEI_INTTIMER);
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    dummy ++;
    portEND_SWITCHING_ISR(xHigherPriorityTaskWoken);
}

void QEI_Init (void)
{
    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI0);

    // Enable QEI Peripherals
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_QEI1);

    //Unlock GPIOD7 - Like PF0 its used for NMI - Without this step it doesn't work
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = GPIO_LOCK_KEY; //In Tiva include this is the same as "_DD" in older versions (0x4C4F434B)
    HWREG(GPIO_PORTD_BASE + GPIO_O_CR) |= 0x80;
    HWREG(GPIO_PORTD_BASE + GPIO_O_LOCK) = 0;

    //Set Pins to be PHA0 and PHB0
    GPIOPinConfigure(GPIO_PD6_PHA0);
    GPIOPinConfigure(GPIO_PD7_PHB0);

    //Set Pins to be PHA1 and PHB1
    GPIOPinConfigure(GPIO_PC5_PHA1);
    GPIOPinConfigure(GPIO_PC6_PHB1);

    //Set GPIO pins for QEI. PhA0 -> PD6, PhB0 ->PD7. I believe this sets the pull up and makes them inputs
    // GPIOPinTypeQEI(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7);
    GPIODirModeSet(GPIO_PORTD_BASE, GPIO_PIN_6  | GPIO_PIN_7, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_6 | GPIO_PIN_7, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIODirModeSet(GPIO_PORTC_BASE, GPIO_PIN_5  | GPIO_PIN_6, GPIO_DIR_MODE_HW);
    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_5 | GPIO_PIN_6, GPIO_STRENGTH_2MA,GPIO_PIN_TYPE_STD);

    GPIOPinTypeGPIOOutput(GPIO_PORTE_BASE, GPIO_PIN_4 | GPIO_PIN_5);

    //DISable peripheral and int before configuration
    QEIDisable(QEI0_BASE);
    QEIIntDisable(QEI0_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI0_BASE, (QEI_CONFIG_CAPTURE_A  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP), 1000);

    //DISable peripheral and int before configuration
    QEIDisable(QEI1_BASE);
    QEIIntDisable(QEI1_BASE,QEI_INTERROR | QEI_INTDIR | QEI_INTTIMER | QEI_INTINDEX);
    // Configure quadrature encoder, use an arbitrary top limit of 1000
    QEIConfigure(QEI1_BASE, (QEI_CONFIG_CAPTURE_A  | QEI_CONFIG_NO_RESET  | QEI_CONFIG_CLOCK_DIR | QEI_CONFIG_NO_SWAP), 1000);

    QEIVelocityDisable(QEI0_BASE);
    QEIVelocityConfigure(QEI0_BASE,QEI_VELDIV_1,0.5*ONE_SEC);
    QEIVelocityEnable(QEI0_BASE);

    QEIVelocityDisable(QEI1_BASE);
    QEIVelocityConfigure(QEI1_BASE,QEI_VELDIV_1,0.5*ONE_SEC);
    QEIVelocityEnable(QEI1_BASE);

    QEIIntClear(QEI0_BASE, QEI_INTTIMER);
    QEIIntEnable(QEI0_BASE, QEI_INTTIMER);

    QEIIntClear(QEI1_BASE, QEI_INTTIMER);
    QEIIntEnable(QEI1_BASE, QEI_INTTIMER);

    // Enable the quadrature encoder.
    QEIEnable(QEI0_BASE);
    QEIEnable(QEI1_BASE);

    IntEnable(INT_QEI0);
    IntEnable(INT_QEI1);


    //Set position to a middle value so we can see if things are working
    QEIPositionSet(QEI0_BASE, 0);
    QEIPositionSet(QEI1_BASE, 0);
}  // QEI0_Init()
