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

#ifndef CONFIGADC_H_
#define CONFIGADC_H_

typedef struct
{
	uint16_t chan1;
	uint16_t chan2;
    uint16_t chan3;
    uint16_t chan4;
} MuestrasADC;

typedef struct
{
	uint32_t chan1;
	uint32_t chan2;
	uint32_t chan3;
    uint16_t chan4;
} MuestrasLeidasADC;

void configADC0_LeeADC(MuestrasADC *datos);

void configADC0_IniciaADC(void);

void ADC0Seq1IntHandler(void);

#endif /* CONFIGADC_H_ */
