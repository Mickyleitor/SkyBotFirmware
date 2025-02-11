//*****************************************************************************
//
// commands.c - FreeRTOS porting example on CCS4
//
//*****************************************************************************


#include <stdbool.h>
#include <stdint.h>

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <assert.h>

/* FreeRTOS includes */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Standard TIVA includes */
#include "inc/hw_memmap.h"
#include "inc/hw_sysctl.h"
#include "inc/hw_types.h"

#include "driverlib/debug.h"
#include "driverlib/gpio.h"
#include "driverlib/uart.h"
#include "driverlib/sysctl.h"
#include "driverlib/rom.h"
#include "driverlib/pwm.h"

/* Other TIVA includes */
#include "utils/cpu_usage.h"
#include "utils/cmdline.h"
#include "utils/uartstdio.h"
#include "Skybot_servos.h"

// ==============================================================================
// The CPU usage in percent, in 16.16 fixed point format.
// ==============================================================================
extern uint32_t g_ui32CPUUsage;
extern uint32_t ui32DutyCycle[2];

// MLMM: Tarea led rojo parpadeando
extern TaskHandle_t handle;
extern QueueHandle_t QueueServoTicksRight;
extern QueueHandle_t QueueServoTicksLeft;


// ==============================================================================
// Implementa el comando velocity que cambia la velocidad de los servomotores
// ==============================================================================
int  Cmd_velocity(int argc, char *argv[])
{
    int speed = 1;

    if (argc > 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" [w|s|a|d] [veces (opcional - MAX 65535)]\n");
        // Return success.
        return(1);
    }
    else if((argc == 2) && (strtoul(argv[1], NULL, 10))<65535){
        speed = strtoul(argv[1], NULL, 10);
    }
    if (0==strncmp( argv[0], "w",1)) acelerar_robot(speed,speed);
    else if (0==strncmp( argv[0], "s",1)) acelerar_robot(-speed,-speed);
    else if (0==strncmp( argv[0], "a",1)) acelerar_robot(-speed,speed);
    else if (0==strncmp( argv[0], "d",1)) acelerar_robot(speed,-speed);
    else{
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" [w|s|a|d] [veces (opcional - MAX 65535)]\n");
        // Return success.
        return(1);
    }
    return(0);
}

int  Cmd_mover(int argc, char *argv[])
{
    if (argc > 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" mover [distancia en cm]\n");
        // Return success.
        return(1);
    }
    int distancia = strtol(argv[1], NULL,10);

    mover_robot(distancia);

    return(0);
}
int  Cmd_girar(int argc, char *argv[])
{
    if (argc > 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" girar [distancia en cm]\n");
        // Return success.
        return(1);
    }
    float grados = strtof(argv[1], NULL);

    girar_robot(grados);

    return(0);
}
// ==============================================================================
// Implementa el comando velocity que cambia la velocidad de los servomotores
// ==============================================================================
int  Cmd_pause (int argc, char *argv[])
{
    // selected_circuit = 0;
    ui32DutyCycle[MOTOR_DERECHO] = STOPCOUNT;
    ui32DutyCycle[MOTOR_IZQUIERDO] = STOPCOUNT;
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_6,ui32DutyCycle[MOTOR_DERECHO]);
    PWMPulseWidthSet(PWM1_BASE, PWM_OUT_7,ui32DutyCycle[MOTOR_IZQUIERDO]);
    return(0);
}
// ==============================================================================
// Implementa el comando velocity que cambia la velocidad de los servomotores
// ==============================================================================
int  Cmd_circuit(int argc, char *argv[])
{
    if (argc != 2)
    {
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" [circuito] [numero del circuito predefinido]\n");
        // Return success.
        return(1);
    }else if((strtoul(argv[1], NULL, 10))<2){
        // selected_circuit = strtoul(argv[1], NULL, 10);
        UARTprintf("Circuito numero N/A seleccionado\n");
    }else{
        //Si los parametros no son suficientes o son demasiados, muestro la ayuda
        UARTprintf(" [circuito] [numero del circuito predefinido]\n");
        // Return success.
        return(1);
    }
    return(0);
}
// ==============================================================================
// Implementa el comando cpu que muestra el uso de CPU
// ==============================================================================
int  Cmd_cpu(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("ARM Cortex-M4F %u MHz - ",SysCtlClockGet() / 1000000);
    UARTprintf("%2u%% de uso\n", (g_ui32CPUUsage+32768) >> 16);

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando free, que muestra cuanta memoria "heap" le queda al FreeRTOS
// ==============================================================================
int Cmd_free(int argc, char *argv[])
{
    //
    // Print some header text.
    //
    UARTprintf("%d bytes libres\n", xPortGetFreeHeapSize());

    // Return success.
    return(0);
}

// ==============================================================================
// Implementa el comando task. S�lo es posible si la opci�n configUSE_TRACE_FACILITY de FreeRTOS est� habilitada
// ==============================================================================
#if ( configUSE_TRACE_FACILITY == 1 )

extern char *__stack;
int Cmd_tasks(int argc, char *argv[])
{
	char*	pcBuffer;
	uint8_t*	pi8Stack;
	portBASE_TYPE	x;

	pcBuffer = pvPortMalloc(1024);
	vTaskList(pcBuffer);
	UARTprintf("\t\t\t\tUnused\nTaskName\tStatus\tPri\tStack\tTask ID\n");
	UARTprintf("=======================================================\n");
	UARTprintf("%s", pcBuffer);

	// Calculate kernel stack usage
	x = 0;
	pi8Stack = (uint8_t *) &__stack;
	while (*pi8Stack++ == 0xA5)
	{
		x++;	//Esto s�lo funciona si hemos rellenado la pila del sistema con 0xA5 en el arranque
	}
	sprintf((char *) pcBuffer, "%%%us", configMAX_TASK_NAME_LEN);
	sprintf((char *) &pcBuffer[10], (const char *) pcBuffer, "kernel");
	UARTprintf("%s\t-\t*%u\t%u\t-\n", &pcBuffer[10], configKERNEL_INTERRUPT_PRIORITY, x/sizeof(portBASE_TYPE));
	vPortFree(pcBuffer);
	return 0;
}

#endif /* configUSE_TRACE_FACILITY */

#if configGENERATE_RUN_TIME_STATS
// ==============================================================================
// Implementa el comando "Stats"
// ==============================================================================
Cmd_stats(int argc, char *argv[])
{
	char*	pBuffer;

	pBuffer = pvPortMalloc(1024);
	if (pBuffer)
	{
		vTaskGetRunTimeStats(pBuffer); //Es un poco inseguro, pero por ahora nos vale...
		UARTprintf("TaskName\tCycles\t\tPercent\r\n");
		UARTprintf("===============================================\r\n");
		UARTprintf("%s", pBuffer);
		vPortFree(pBuffer);
	}
	return 0;
}
#endif

// ==============================================================================
// Implementa el comando help
// ==============================================================================
int Cmd_help(int argc, char *argv[])
{
    tCmdLineEntry *pEntry;

    //
    // Print some header text.
    //
    UARTprintf("Comandos disponibles\n");
    UARTprintf("------------------\n");

    //
    // Point at the beginning of the command table.
    //
    pEntry = &g_psCmdTable[0];

    //
    // Enter a loop to read each entry from the command table.  The end of the
    // table has been reached when the command name is NULL.
    //
    while(pEntry->pcCmd)
    {
        //
        // Print the command name and the brief description.
        //
        UARTprintf("%s%s\n", pEntry->pcCmd, pEntry->pcHelp);

        //
        // Advance to the next entry in the table.
        //
        pEntry++;
    }

    //
    // Return success.
    //
    return(0);
}
// ==============================================================================
// Tabla con los comandos y su descripcion. Si quiero anadir alguno, debo hacerlo aqui
// ==============================================================================
tCmdLineEntry g_psCmdTable[] =
{
    { "help",     Cmd_help,      "       : Lista de comandos" },
    { "?",        Cmd_help,      "          : lo mismo que help" },
    { "w",      Cmd_velocity,       "        : Aumenta velocidad en un sentido" },
    { "s",      Cmd_velocity,       "        : Aumento velocidad en el otro sentido " },
    { "d",      Cmd_velocity,       "        : Gira hacia la derecha " },
    { "a",      Cmd_velocity,       "        : Gira hacia la izquierda " },
    { "p",      Cmd_pause,       "        : Pausa o reanuda el robot " },
    { "mover",      Cmd_mover,       "        : Mueve el robot ciertos centimetros " },
    { "girar",      Cmd_girar,       "        : Gira el robot ciertos grados " },
    { "circuito",      Cmd_circuit,       " : Inicia un circuito predefinido " },
    { "cpu",      Cmd_cpu,       "        : Muestra el uso de  CPU " },
    { "free",     Cmd_free,      "       : Muestra la memoria libre" },
#if ( configUSE_TRACE_FACILITY == 1 )
	{ "tasks",    Cmd_tasks,     "      : Muestra informacion de las tareas" },
#endif
#if (configGENERATE_RUN_TIME_STATS)
	{ "stats",    Cmd_stats,     "      : Muestra estadisticas de las tareas" },
#endif
    { 0, 0, 0 }
};

// ==============================================================================
// Tarea UARTTask.  Espera la llegada de comandos por el puerto serie y los ejecuta al recibirlos...
// ==============================================================================
//extern xSemaphoreHandle g_xRxLineSemaphore;
void UARTStdioIntHandler(void);

void vUARTTask( void *pvParameters )
{
    char    pcCmdBuf[64];
    int32_t i32Status;

    //
    // Mensaje de bienvenida inicial.
    //
    UARTprintf("\n\nWelcome to the TIVA FreeRTOS Demo!\n");
	UARTprintf("\n\n FreeRTOS %s \n",
	tskKERNEL_VERSION_NUMBER);
	UARTprintf("\n Teclee ? para ver la ayuda \n");
	UARTprintf("> ");

	/* Loop forever */
	while (1)
	{

		/* Read data from the UART and process the command line */
		UARTgets(pcCmdBuf, sizeof(pcCmdBuf));
		if (strlen(pcCmdBuf) == 0)
		{
			UARTprintf("> ");
			continue;
		}

		//
		// Pass the line from the user to the command processor.  It will be
		// parsed and valid commands executed.
		//
		i32Status = CmdLineProcess(pcCmdBuf);

		//
		// Handle the case of bad command.
		//
		if(i32Status == CMDLINE_BAD_CMD)
		{
			UARTprintf("Comando erroneo!\n");	//No pongo acentos adrede
		}

		//
		// Handle the case of too many arguments.
		//
		else if(i32Status == CMDLINE_TOO_MANY_ARGS)
		{
			UARTprintf("El interprete de comandos no admite tantos parametros\n");	//El maximo, CMDLINE_MAX_ARGS, esta definido en cmdline.c
		}

		//
		// Otherwise the command was executed.  Print the error code if one was
		// returned.
		//
		else if(i32Status != 0)
		{
			UARTprintf("El comando devolvio el error %d\n",i32Status);
		}

		UARTprintf("> ");

	}
}
