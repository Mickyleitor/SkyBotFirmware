#ifndef CONFIGBUTTONS_H_
#define CONFIGBUTTONS_H_

#include <stdint.h>

#define T_ANTIREBOTE (SysCtlClockGet() * 0.02)

void GPIOPortFIntHandler(void);
void Timer0AIntHandler(void);
void configButtons_init(void);

#endif /* CONFIGBUTTONS_H_ */
