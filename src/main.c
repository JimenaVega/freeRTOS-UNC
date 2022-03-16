
/*
 * FreeRTOS V202104.00
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 */

/* Environment includes. */
#include "DriverLib.h"

/* Scheduler includes. */
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "semphr.h"

/* Demo app includes. */
#include "integer.h"
#include "PollQ.h"
#include "semtest.h"
#include "BlockQ.h"
#include <stdlib.h>
#include <stdio.h>
#include <string.h>


//mine
//#include <math.h>

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 57600 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

// /* Demo board specifics. */
// #define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define mainTOP_DELAY				( ( TickType_t ) 2000 / portTICK_PERIOD_MS )  
#define mainDISPLAY_DELAY			( ( TickType_t ) 200 / portTICK_PERIOD_MS ) 

//#define N 10
#define STEP 8
#define INIT_N 10

//--------------CPU load------------------------
static volatile unsigned int uxPercentLoadCPU = (unsigned int) 0U; // Last CPU load calculated
static volatile unsigned int uxIdleTickCount  = (unsigned int) 0U; // How many ticks were in idle task
static volatile unsigned int uxCPULoadCount   = (unsigned int) 0U; // For 0 to configTICKRATE_HZ we will count idle tasks

//----------------------------
/* String that is transmitted on the UART. */
static char *cMessage = "pos me mato QAQ\n";
static volatile char *pcNextChar;



//Funciones
static void prvSetupHardware( void );
static char* intToChar(int val);
char* longToChar(unsigned long value, char *ptr, int base);
void printStatistics(char* taskName, UBaseType_t waterMark);
static void testTask(void *p);
void getBaseGraph(unsigned char *pucImage, int avg);
void sendToUART(char* data);
void vTaskGetStats( char *pcWriteBuffer );
//Task
static void vDisplayTask( void *pvParameter );
static void vWaterMark(void *pvParameters);
static void vFilterTask(void *pvParameters);
static void vTempSensorTask(void *p);
//static void vUARTTask(void* p);
static void vTopTask(void *p);



/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xAvgQueue;
QueueHandle_t xUARTQueue;
QueueHandle_t xPrintQueue;

TaskHandle_t filterHandle;
TaskHandle_t sensorHandle;
TaskHandle_t displayHandle;


/*-----------------------------------------------------------*/

int main(void){
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the queue used to pass message to vDisplayTask. */
	xAvgQueue = xQueueCreate(mainQUEUE_SIZE, sizeof( char * ));
	xPrintQueue = xQueueCreate(mainQUEUE_SIZE, sizeof( char * ));
	xUARTQueue = xQueueCreate(mainQUEUE_SIZE, sizeof( char * ));
	//1 Filter
	//2 Sensor
	//3 Display
	//4 Top
	//5 Water mark
	/* Start the tasks defined within the file. */
	xTaskCreate(vFilterTask, "1", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &filterHandle );
	xTaskCreate(vTempSensorTask, "2", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &sensorHandle);
	xTaskCreate(vDisplayTask, "3", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &displayHandle );
	xTaskCreate(vWaterMark, "5", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);
	xTaskCreate(vTopTask, "4", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);
	
	//Start the scheduler.
	vTaskStartScheduler();

	return 0;
}


static void prvSetupHardware( void ){

	//Setup the PLL. 
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );

	// Enable the UART.
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

	//Configure the UART for 8-N-1 operation.
	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

	/* We don't want to use the fifo.  This is for test purposes to generate
	as many interrupts as possible. */
	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

	//Enable Tx interrupts.
	// UARTIntEnable(UART0_BASE, UART_INT_RX);
	// UARTIntEnable(UART0_BASE, UART_INT_TX);
	HWREG( UART0_BASE + UART_O_IM ) |= ( UART_INT_TX | UART_INT_RX );
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable(INT_UART0);

	//Initialise the LCD
    OSRAMInit(false);
    OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}


//Mide entre 18 y 3 grados celsius
static void vTempSensorTask(void *p){

	int temp = 0;
	int t = 1;
	const TickType_t xDelay = 1000; //100 Hz
	int delta = 1;

	for(;;){

		if(delta){

			t++;
			if(t >= 128){
				delta = 0;
			}
			
		}
		else{
			t--;
			if(t <= 0){
				delta = 1;
			}
		}
	
		xQueueSend(xAvgQueue, &t, portMAX_DELAY);
		vTaskDelay(xDelay);
		
	}
}

static char* intToChar(int val){

    static char buf[32] = {0};
    int i = 30;
	int base = 10;

    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];

    return &buf[i+1];

}

char *longToChar(unsigned long value, char *ptr, int base) {
    unsigned long t = 0, res = 0;
	unsigned long tmp = value;
	int count = 0;

	if (NULL == ptr) {
	return NULL;
	}

	if (tmp == 0) {
	count++;
	}

	while(tmp > 0){
	tmp = tmp/base;
	count++;
	}

	ptr += count;

	*ptr = '\0';

	do {
	res = value - base * (t = value / base);

	if (res < 10 ){
		* -- ptr = '0' + res;
	}
	else if ((res >= 10) && (res < 16)) {
		* --ptr = 'A' - 10 + res;
	}
	} while ((value = t) != 0);

	return(ptr);
}

static void vFilterTask(void *pvParameters){

	
	char* output = "";
	const TickType_t xDelay = 10; //100ms
	unsigned char *pucImage;
	int lastSamples[INIT_N] = {0};

	int N = INIT_N;
	char* N_char;

	uint8_t y_axis=0;

	int rcv = 0;
	int avg = 0;
	
	for(;;){

		int acum = 0;

		//xQueueReceive(xAvgQueue, &rcv, portMAX_DELAY);
		xQueueReceive(xUARTQueue, &N, 0);
		N_char = intToChar(N);
		
		OSRAMClear();
		OSRAMStringDraw(N_char , 0, 0);
		
		//shifting array elements
		for(int i=(N-1); i>0; i--){
			lastSamples[i] = lastSamples[i-1];
		}
		lastSamples[0] = rcv;

		for(int i=0; i<N; i++){
			acum += lastSamples[i];
		}
		avg = acum / N;
		output = intToChar(avg);

		xQueueSend(xPrintQueue, &avg, portMAX_DELAY);
		vTaskDelay(xDelay);
		
		
	}

}

static void vDisplayTask(void *pvParameters){

    char *message;
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
    unsigned char pucImage[128] = {0};
    uint8_t value_height;
	TickType_t xLastExecutionTime;

	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();


    for( ;; ){
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainDISPLAY_DELAY );
       
        xQueueReceive(xPrintQueue, &message, portMAX_DELAY);
        
        value_height = (uint8_t)message/8;

        if(value_height < 8){
            pucImage[127] = (0b10000000 >> value_height);
            pucImage[63] = 0;
        }
        else{
            pucImage[63] = (0b10000000 >> (value_height-8));
            pucImage[127] = 0;
        }

        for(int i = 0; i<128; i++){
            if(i != 63 && i != 127){
                pucImage[i] = pucImage[i+1];
            }
        }
        
        OSRAMImageDraw(pucImage, 16, 0, 64, 2);
    }
}
/**
 * @brief 
 * Calcular el stack necesario para cada task. Realizar el análisis utilizando
 * uxTaskGetStackHighWaterMark o vApplicationStackOverflowHook
 * @param pvParameters 
 */
static void vWaterMark(void *pvParameters){

	UBaseType_t taskWaterMark;
	
	for(;;){

		taskWaterMark = uxTaskGetStackHighWaterMark(filterHandle);
		printStatistics("Filter", taskWaterMark);

		taskWaterMark = uxTaskGetStackHighWaterMark(sensorHandle);
		printStatistics("Sensor", taskWaterMark);

	}

}

void printStatistics(char* taskName, UBaseType_t waterMark){

		char* usedStack = "";

		usedStack = intToChar((configMINIMAL_STACK_SIZE - waterMark) * 4);

		OSRAMClear();
		OSRAMStringDraw(taskName, 0, 0);
        OSRAMStringDraw(usedStack, 0x3f, 0);
		vTaskDelay(1000);

}


static void vTopTask( void *pvParameters )
{
	
	TickType_t xLastExecutionTime;
	char pcWriteBuffer[150];


	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();

	for( ;; )
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainTOP_DELAY );
		//vTaskDelay(1000);

		vTaskGetStats(pcWriteBuffer);

		UARTIntDisable( UART0_BASE, UART_INT_TX );
		{
			pcNextChar = pcWriteBuffer;

			/* Send the first character. */
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}

			pcNextChar++;
		}
		UARTIntEnable(UART0_BASE, UART_INT_TX);	
	}
}

/*-----------------------------------------------------------*/

void vTaskGetStats( char *pcWriteBuffer )
{
	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	uint32_t ulTotalRunTime, ulStatsAsPercentage;
	char buffer[10];

	// Make sure the write buffer does not contain a string.
	*pcWriteBuffer = 0x00;

	// Take a snapshot of the number of tasks in case it changes while this
	// function is executing.
	uxArraySize = uxTaskGetNumberOfTasks();

	// Allocate a TaskStatus_t structure for each task.  An array could be
	// allocated statically at compile time.
	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL )
	{
		// Generate raw status information about each task.
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

		// For percentage calculations.
		ulTotalRunTime /= 100UL;

		// Avoid divide by zero errors.
		if( ulTotalRunTime > 0 )
		{
			//strcat(pcWriteBuffer,"Task\t\tAbs Time\t%Time\n");
			//strcat(pcWriteBuffer,"Task\t\tAbs Time\n");
			// strcat(pcWriteBuffer,"-------------------------------------\n");
			// For each populated position in the pxTaskStatusArray array,
			// format the raw data as human readable ASCII data
			for( x = 0; x < uxArraySize; x++ )
			{
				// What percentage of the total run time has the task used?
				// This will always be rounded down to the nearest integer.
				// ulTotalRunTimeDiv100 has already been divided by 100.
				ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

				if( ulStatsAsPercentage > 0UL )
				{
					strcat(pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName);
					strcat(pcWriteBuffer,"\n");
					strcat(pcWriteBuffer, longToChar(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
					//strcat(pcWriteBuffer,"\n");
					strcat(pcWriteBuffer,"\t\t");
					strcat(pcWriteBuffer,longToChar(ulStatsAsPercentage, buffer, 10));
					strcat(pcWriteBuffer,"%\r\n");
				}
				else
				{
				// 	// If the percentage is zero here then the task has
				// 	// consumed less than 1% of the total run time.
					strcat(pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName);
					strcat(pcWriteBuffer,"\t\t");
					strcat(pcWriteBuffer, longToChar(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
					//strcat(pcWriteBuffer,"\n");
					strcat(pcWriteBuffer,"\t\t");
					strcat(pcWriteBuffer,"<1%");
					strcat(pcWriteBuffer,"\r\n");
				}
				
				pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
			}
			strcat(pcWriteBuffer, "\n\n");
			pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
		}

		// The array is no longer needed, free the memory it consumes.
		//vPortFree( pxTaskStatusArray );
	}
}
void sendToUART(char* data){

	UARTIntDisable(UART0_BASE, UART_INT_TX );
	{
		pcNextChar = data;

		/* Send the first character. */
		if(!( HWREG(UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) ){
			HWREG(UART0_BASE + UART_O_DR ) = *pcNextChar;
		}

		pcNextChar++;
	}
	UARTIntEnable(UART0_BASE, UART_INT_TX);
}

void vUART_ISR(void) {

	unsigned long ulStatus;
	char* rxChar;
	int rxNum;
	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;

	// What caused the interrupt. 
	ulStatus = UARTIntStatus(UART0_BASE, pdTRUE );
	
	//Clear the interrupt.
	UARTIntClear(UART0_BASE, ulStatus );

	//Was a Tx interrupt pending? 
	if(ulStatus & UART_INT_TX) {
		//Send the next character in the string.  We are not using the FIFO. 
		if( *pcNextChar != 0 ){

			if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF)){

				HWREG(UART0_BASE + UART_O_DR) = *pcNextChar;
			}
			pcNextChar++;
		}
	}

	if(ulStatus & UART_INT_RX){

		if((HWREG(UART0_BASE + UART_O_FR ) & UART_FR_RXFF)){

			rxNum = UARTCharGet(UART0_BASE) - 48;
			// rxChar = intToChar(rxNum);
			// OSRAMClear();
			// OSRAMStringDraw(rxChar , 0, 0);
			xQueueSend(xUARTQueue, &rxNum, portMAX_DELAY);
		}
	}
	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
}