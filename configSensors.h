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

#ifndef CONFIGSENSORS_H_
#define CONFIGSENSORS_H_


#define SENSOR_FL GPIO_PIN_3
#define SENSOR_FR GPIO_PIN_0
#define SENSOR_BL GPIO_PIN_2
#define SENSOR_BR GPIO_PIN_1

void configSensors_init()
{
    //Inicializa el puerto B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOD);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOD);

    GPIOPinTypeGPIOInput(GPIO_PORTD_BASE,SENSOR_FL | SENSOR_FR | SENSOR_BL | SENSOR_BR);
    // Temporal para practicar con circuito de casa
    GPIOPadConfigSet(GPIO_PORTD_BASE, SENSOR_FL | SENSOR_FR | SENSOR_BL | SENSOR_BR, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPU);

    //Inicializa el puerto B
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);
    SysCtlPeripheralSleepEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinTypeGPIOInput(GPIO_PORTB_BASE,SENSOR_FL | SENSOR_FR | SENSOR_BL | SENSOR_BR);
    // Temporal para practicar con circuito de casa
    GPIOPadConfigSet(GPIO_PORTB_BASE, SENSOR_FL | SENSOR_FR | SENSOR_BL | SENSOR_BR, GPIO_STRENGTH_2MA, GPIO_PIN_TYPE_STD_WPD);
};

#endif /* CONFIGSENSORS_H_ */
