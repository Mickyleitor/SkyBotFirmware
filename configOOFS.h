/*
 *  Libreria para sensores de deteccion de borde de tarima
 *  (Out Of Field System)
 *
 *  Autor : Michele La Malva Moreno
 *
 */

#ifndef CONFIGOOFS_H_
#define CONFIGOOFS_H_

/*
 * Configuración del GPIO
 */
void configOOFS_init(void);
/*
 * Manejador de la interrupcion del Timer Antirebote
 */
void Timer1AIntHandler(void);
/*
 * Manejador de la interrupcion del puerto de los sensores
 */
void GPIOPortBIntHandler(void);

#endif /* CONFIGOOFS_H_ */
