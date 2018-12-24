/*
 * configQEI.h
 *
 *  Created on: 22/12/2018
 *      Author: Micky
 */

#ifndef CONFIGQEI_H_
#define CONFIGQEI_H_

void QEI_init(void);
void QEI0_ISR(void);
void QEI1_ISR(void);
int QEI0_meanvel(void);
int QEI1_meanvel(void);


#endif /* CONFIGQEI_H_ */
