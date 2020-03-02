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

#ifndef CONFIGWHISKER_H_
#define CONFIGWHISKER_H_


#define SENSOR_FL GPIO_PIN_3
#define SENSOR_FR GPIO_PIN_0
#define SENSOR_BL GPIO_PIN_2
#define SENSOR_BR GPIO_PIN_1

void configWhisker_init()
{
    //Inicializa el puerto B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0);
    // Temporal para practicar con circuito de casa
    GPIOPadConfigSet(GPIO_PORTD_BASE, GPIO_PIN_3 | GPIO_PIN_2 | GPIO_PIN_1 | GPIO_PIN_0, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);
};

#endif /* CONFIGWHISKER_H_ */
