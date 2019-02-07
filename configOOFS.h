#ifndef CONFIGOOFS_H_
#define CONFIGOOFS_H_

#include <stdint.h>

#define T_ANTIREBOTE (SysCtlClockGet() * 0.02)
#define SENSOR_FL GPIO_PIN_3
#define SENSOR_FR GPIO_PIN_0
#define SENSOR_BL GPIO_PIN_2
#define SENSOR_BR GPIO_PIN_1

void configOOFS_init(void);
void Timer1AIntHandler(void);
void GPIOPortBIntHandler(void);

#endif /* CONFIGOOFS_H_ */
