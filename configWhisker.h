#ifndef CONFIGWHISKER_H_
#define CONFIGWHISKER_H_


#define T_ANTIREBOTE (SysCtlClockGet() * 0.02)
#define SENSOR_FL GPIO_PIN_3
#define SENSOR_FR GPIO_PIN_0
#define SENSOR_BL GPIO_PIN_2
#define SENSOR_BR GPIO_PIN_1

void configWhisker_init(void);
void Timer4AIntHandler(void);
void GPIOPortDIntHandler(void);

#endif /* CONFIGWHISKER_H_ */
