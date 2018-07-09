/*
===============================================================================
 Name        : FRTOS.c
 Author      : $(author)
 Version     :
 Copyright   : $(copyright)
 Description : main definition
===============================================================================
*/

/****************************************INCLUCIONES DE BIBLIOTECAS************************************************/
#include "chip.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/***********************************************MANEJADORES********************************************************/
SemaphoreHandle_t Semaphore_ADC;
SemaphoreHandle_t Semaphore_I2C;

QueueHandle_t 	Cola_Aperturas;
QueueHandle_t 	Cola_Aperturas_Finales;

/*******************************************OTRAS INCLUCIONES***************************************************/
#include <cr_section_macros.h>

/****************************************DEFINICIONES PARA GPIO************************************************/
#define PORT(x) 	((uint8_t) x)
#define PIN(x)		((uint8_t) x)

#define OUTPUT		((uint8_t) 1)
#define INPUT		((uint8_t) 0)

#define DESACTIVADO				0
#define ACTIVADO				1
#define CERRADA					0
#define ABIERTA					1
#define OFF						0
#define ON						1

/****************************************DEFINICIONES DE PINES************************************************/
#define LED_PORT		((uint8_t) 0)
#define LED_PIN			((uint8_t) 22)

#define MOTOR_PORT		((uint8_t) 0)
#define MOTOR_PIN		((uint8_t) 23)

#define VENTI_PORT		((uint8_t) 0)
#define VENTI_PIN		((uint8_t) 24)

#define BOTON_PORT		((uint8_t) 0)
#define BOTON_PIN		((uint8_t) 25)

#define PUERTA_PORT		((uint8_t) 0)
#define PUERTA_PIN		((uint8_t) 26)

/****************************************DEFINICIONES PARA TIMERS************************************************/
#define MATCH0		0
#define MATCH1		1
#define MATCH2		2
#define MATCH3		3

/****************************************DEFINICIONES PARA ADC************************************************/
#define SLAVE_ADDRESS		0x50
#define W_ADDRESS			0x00B0

unsigned char Datos_Tx [6] = { (W_ADDRESS & 0xFF00) >> 8 , W_ADDRESS & 0x00FF}; //, DATO
unsigned char Datos_Rx [4] = {0};

static ADC_CLOCK_SETUP_T ADCSetup;

/****************************************PROTOTIPO DE FUNCIONES************************************************/
void Motor_On(void);
void Motor_Off(void);

void Ventilador_On(void);
void Ventilador_Off(void);

void Luz_On(void);
void Luz_Off(void);

uint32_t EstadoPuerta(void);
uint32_t EstadoMotor(void);
uint32_t Boton_Apagado(void);

/****************************************FUNCIONES DE INTERRUPCION************************************************/
void ADC_IRQHandler(void) // 	USAR TODO FROM_ISR
{
	BaseType_t FuerzaCC = pdFALSE;

	NVIC_DisableIRQ(ADC_IRQn);

	xSemaphoreGiveFromISR( Semaphore_ADC, &FuerzaCC ); // ANALIZA SI HAY UNA TAREA DE MAYOR PRIORIDAD A LA QUE SE ESTABA EJECUTANDO
													// BLOQUEADA POR ÉSTE SEMAFORO Y EN EL CASO DE QUE SI, FUERZA EL CAMBIO
													// DE CONTEXTO

	portYIELD_FROM_ISR( FuerzaCC );					//
}

void TIMER0_IRQHandler(void)
{
	BaseType_t FuerzaCC = pdFALSE;

	if (Chip_TIMER_MatchPending(LPC_TIMER0, MATCH0))
	{
		Chip_TIMER_ClearMatch(LPC_TIMER0, MATCH0);

		xSemaphoreGiveFromISR( Semaphore_I2C, &FuerzaCC);

		portYIELD_FROM_ISR( FuerzaCC );
	}
}

/****************************************FUNCIONES DE CONFIGURACION************************************************/
void ADC_Config (void)
{
	Chip_IOCON_PinMux(LPC_IOCON, 0, 23, IOCON_MODE_INACT, IOCON_FUNC1);

	//Chip_ADC_ReadStatus(_LPC_ADC_ID, _ADC_CHANNLE, ADC_DR_DONE_STAT)

	Chip_ADC_Init(LPC_ADC, &ADCSetup);
	Chip_ADC_EnableChannel(LPC_ADC, ADC_CH0, ENABLE);
	Chip_ADC_SetSampleRate(LPC_ADC, &ADCSetup, 50000); 		//		Ésta frecuencia de muestreo no bajarla
	Chip_ADC_Int_SetChannelCmd(LPC_ADC, ADC_CH0, ENABLE);
	Chip_ADC_SetBurstCmd(LPC_ADC, DISABLE);

	NVIC_ClearPendingIRQ(ADC_IRQn);
	NVIC_EnableIRQ(ADC_IRQn);

	Chip_ADC_SetStartMode (LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);

}

void Timer_Config(void)
{
	uint32_t timerFreq;

	Chip_TIMER_Init(LPC_TIMER0);

	/* Timer rate is system clock rate */
	timerFreq = Chip_Clock_GetSystemClockRate();

	/* Timer setup for match and interrupt at TICKRATE_HZ */
	Chip_TIMER_Reset(LPC_TIMER0);
	Chip_TIMER_MatchEnableInt(LPC_TIMER0, MATCH0);
	Chip_TIMER_SetMatch(LPC_TIMER0, 1, (timerFreq * 300)); //cada 5 min
	Chip_TIMER_ResetOnMatchEnable(LPC_TIMER0, MATCH0);
	Chip_TIMER_Enable(LPC_TIMER0);

	/* Enable timer interrupt */
	NVIC_ClearPendingIRQ(TIMER0_IRQn);
	NVIC_EnableIRQ(TIMER0_IRQn);
}

void GPIO_Config (void)
{
	/*INICIALIZO PERIFERICO*/
	Chip_GPIO_Init (LPC_GPIO);

	/*SALIDAS*/

		/*LED*/
		Chip_IOCON_PinMux (LPC_IOCON , PORT(LED_PORT) , PIN(LED_PIN), IOCON_MODE_INACT , IOCON_FUNC0);
		Chip_GPIO_SetDir (LPC_GPIO , PORT(LED_PORT) , PIN(LED_PIN) , OUTPUT);

		/*MOTOR*/
		Chip_IOCON_PinMux (LPC_IOCON , PORT(MOTOR_PORT) , PIN(MOTOR_PIN), IOCON_MODE_INACT , IOCON_FUNC0);
		Chip_GPIO_SetDir (LPC_GPIO , PORT(MOTOR_PORT) , PIN(MOTOR_PIN) , OUTPUT);

		/*VENTILADOR*/
		Chip_IOCON_PinMux (LPC_IOCON , PORT(VENTI_PORT) , PIN(VENTI_PIN), IOCON_MODE_INACT , IOCON_FUNC0);
		Chip_GPIO_SetDir (LPC_GPIO , PORT(VENTI_PORT) , PIN(VENTI_PIN) , OUTPUT);

	/*ENTRADAS*/

		/*BOTON*/
		Chip_IOCON_PinMux (LPC_IOCON , PORT(BOTON_PORT) , PIN(BOTON_PIN), IOCON_MODE_INACT , IOCON_FUNC0);
		Chip_GPIO_SetDir (LPC_GPIO , PORT(BOTON_PORT) , PIN(BOTON_PIN) , INPUT);
}

/****************************************FUNCION DE INICIALIZACION************************************************/
void uC_StartUp (void)
{
	GPIO_Config ();

	ADC_Config ();

	Timer_Config();
}

/****************************************MAQUINA DE ESTADOS************************************************/
static void vTaskSM(void *pvParameters)
{
	uint32_t Estado_Puerta_Anterior = CERRADA;
	uint32_t Aperturas_Totales;
	uint16_t Temperatura;

	xSemaphoreTake(Semaphore_ADC, 0); // CON CERO TOMA EL SEMAFORO Y NO IMPORTA SI ESTABA TOMADO, NO SE BLOQUEA

	while (uxQueueSpacesAvailable(Cola_Aperturas) > 0)
		portYIELD();

	xQueueReceive(Cola_Aperturas, &Aperturas_Totales, 0);

	while (1)
	{
		xSemaphoreTake(Semaphore_ADC, portMAX_DELAY);

		Chip_ADC_ReadValue(LPC_ADC, ADC_CH0, &Temperatura);  // SOLO EN EL ADC SE BORRA AUTOMATICAMENTE EL FLAG DE INTERRUPCION

		if (Temperatura > 5)
		{
			Motor_On();

			if (EstadoPuerta() == CERRADA)
				Ventilador_On();
		}
		if(Temperatura < 1)
		{
			Motor_Off();
			Ventilador_Off();
		}

		if (EstadoPuerta() == ABIERTA)
		{
			Luz_On ();

			Ventilador_Off();

			if(Estado_Puerta_Anterior == CERRADA)
			{
				Aperturas_Totales++;
				xQueueOverwrite(Cola_Aperturas_Finales, &Aperturas_Totales);
			}
		}
		else
		{
			Luz_Off ();

			if(EstadoMotor() == ON)
				Ventilador_On();
		}

		if ( Boton_Apagado() == ACTIVADO )
		{
			Motor_Off();
			Ventilador_Off();
			Luz_Off ();

			xSemaphoreGive( Semaphore_I2C );
		}

		Estado_Puerta_Anterior = EstadoPuerta();

		vTaskDelay( pdMS_TO_TICKS(200) );

		NVIC_ClearPendingIRQ(ADC_IRQn);
		NVIC_EnableIRQ(ADC_IRQn);

		Chip_ADC_SetStartMode (LPC_ADC, ADC_START_NOW, ADC_TRIGGERMODE_RISING);
	}
}

/****************************************TAREAS************************************************/
static void DatosI2C(void *pvParameters)
{
	uint32_t Aperturas_Iniciales;

	xSemaphoreTake(Semaphore_I2C, 0); // CON CERO TOMA EL SEMAFORO Y NO IMPORTA SI ESTABA TOMADO, NO SE BLOQUEA

    Chip_I2C_MasterSend (I2C1, SLAVE_ADDRESS, Datos_Tx, 2); // Nos posicionamos en la Posicion a leer.
    Chip_I2C_MasterRead (I2C1, SLAVE_ADDRESS, Datos_Rx, 4); // Se leen 05 bytes a partir de la Posición.

    Aperturas_Iniciales = Datos_Rx[0] + Datos_Rx[1] * 256 + Datos_Rx[2] * 256 *256  + Datos_Rx[3] * 256 *256 *256;

    xQueueSendToBack(Cola_Aperturas, &Aperturas_Iniciales, 0);

	while (1)
	{
		xSemaphoreTake(Semaphore_I2C, portMAX_DELAY);

		xQueueReceive( Cola_Aperturas_Finales, &Aperturas_Iniciales, portMAX_DELAY );

		Datos_Tx[2] = Aperturas_Iniciales % 256;
		Aperturas_Iniciales /= 256;

		Datos_Tx[3] = Aperturas_Iniciales % 256;
		Aperturas_Iniciales /= 256;

		Datos_Tx[4] = Aperturas_Iniciales % 256;
		Aperturas_Iniciales /= 256;

		Datos_Tx[5] = Aperturas_Iniciales % 256;

	    Chip_I2C_MasterSend (I2C1, SLAVE_ADDRESS, Datos_Tx, 6);
	}
}

/****************************************FUNCION PRINCIPAL************************************************/
int main(void)
{
	uC_StartUp ();
	SystemCoreClockUpdate();

	vSemaphoreCreateBinary(Semaphore_ADC);
	vSemaphoreCreateBinary(Semaphore_I2C);

	//xSemaphoreTake(Semaforo_1 , portMAX_DELAY );

	Cola_Aperturas = xQueueCreate( 1 , sizeof(uint32_t) );
	Cola_Aperturas_Finales = xQueueCreate( 1 , sizeof(uint32_t) );

	xTaskCreate(vTaskSM, (char *) "vTaskSM",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	xTaskCreate(DatosI2C, (char *) "DatosI2C",
				configMINIMAL_STACK_SIZE, NULL, (tskIDLE_PRIORITY + 1UL),
				(xTaskHandle *) NULL);

	/* Start the scheduler */
	vTaskStartScheduler();

	/* Nunca debería arribar aquí */

    return 0;
}

/****************************************DECLARACION DE FUNCIONES************************************************/
void Motor_On(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, MOTOR_PORT, MOTOR_PIN, ON);
}
void Motor_Off(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, MOTOR_PORT, MOTOR_PIN, OFF);
}

void Ventilador_On(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, VENTI_PORT, VENTI_PIN, ON);
}
void Ventilador_Off(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, VENTI_PORT, VENTI_PIN, OFF);
}

void Luz_On(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, ON);
}
void Luz_Off(void)
{
	Chip_GPIO_SetPinState(LPC_GPIO, LED_PORT, LED_PIN, OFF);
}

uint32_t EstadoPuerta(void)
{
	uint32_t estado;
	estado = Chip_GPIO_GetPinState(LPC_GPIO, PUERTA_PORT, PUERTA_PIN);
	return(estado);
}
uint32_t EstadoMotor(void)
{
	uint32_t estado;
	estado = Chip_GPIO_GetPinState(LPC_GPIO, MOTOR_PORT, MOTOR_PIN);
	return(estado);
}
uint32_t Boton_Apagado(void)
{
	uint32_t estado;
	estado = Chip_GPIO_GetPinState(LPC_GPIO, BOTON_PORT, BOTON_PIN);
	return(estado);
}

