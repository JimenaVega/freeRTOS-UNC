/**
 * @file main.c
 * @author Jimena Vega
 * @brief 
 *  El objetivo del presente trabajo práctico es que el estudiante sea capaz de
 *	diseñar, crear, comprobar y validar una aplicación de tiempo real sobre un
 *	RTOS.
 * @version 0.1
 * @date 2021
 * @copyright FreeRTOS V202104.00
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

#include <string.h>

/* STACK SIZES */
#define configSENSOR_STACK_SIZE     ( ( unsigned short ) (40))  //watermark = 32
#define configFILTER_STACK_SIZE     ( ( unsigned short ) (60)) // watermark = 12
#define configDISPLAY_STACK_SIZE    ( ( unsigned short ) (86)) //en un comienzo dio 0
#define configTOP_STACK_SIZE        ( ( unsigned short ) (100)) //en un comienzo dio 0

/* Delay between cycles of the 'check' task. */
#define mainCHECK_DELAY						( ( TickType_t ) 5000 / portTICK_PERIOD_MS )

/* UART configuration - note this does not use the FIFO so is not very
efficient. */
#define mainBAUD_RATE				( 19200 )
#define mainFIFO_SET				( 0x10 )

/* Demo task priorities. */
#define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
#define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
#define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
#define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define mainTOP_DELAY				( ( TickType_t ) 2000 / portTICK_PERIOD_MS )   

/* Filter's initial value */
#define INIT_N 10

#define UNUSED(x) (void)(x)

/* CPU load */
static volatile unsigned int uxPercentLoadCPU = (unsigned int) 0U; // Last CPU load calculated
static volatile unsigned int uxIdleTickCount  = (unsigned int) 0U; // How many ticks were in idle task
static volatile unsigned int uxCPULoadCount   = (unsigned int) 0U; // For 0 to configTICKRATE_HZ we will count idle tasks

/* String transmitted on to the UART. */
static volatile char *pcNextChar;

/* FUNCTIONS */
static void prvSetupHardware( void );
static char* intToChar(int val);
char* longToChar(unsigned long value, char *ptr, int base);
void getTasksStats( char *writeUART );
void printStatistics(char* taskName, UBaseType_t waterMark);
static void sendToUART(char* data);

/* TASK DECLARATION */
static void vDisplayTask( void *pvParameter );
static void vWaterMark(void *pvParameters);
static void vFilterTask(void *pvParameters);
static void vTempSensorTask(void *pvParameters);
static void vTopTask( void *pvParameters );

/* QUEUES */
QueueHandle_t xAvgQueue;
QueueHandle_t xUARTQueue;
QueueHandle_t xPrintQueue;

/* TASKS */
TaskHandle_t filterHandle;
TaskHandle_t sensorHandle;
TaskHandle_t displayHandle;


int main(void){
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the queue used to pass message to vDisplayTask. */
	xAvgQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ));
	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ));
	xUARTQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ));

	/* Start the tasks defined within the file. */
	xTaskCreate(vFilterTask, "Filter", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &filterHandle );
	xTaskCreate(vTempSensorTask, "Sensor", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &sensorHandle);
	xTaskCreate(vDisplayTask, "Display", configDISPLAY_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, &displayHandle );
	xTaskCreate(vTopTask,"Top", configTOP_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);
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

	//Enable Tx interrupts.
	UARTIntEnable(UART0_BASE, UART_INT_RX);
	UARTIntEnable(UART0_BASE, UART_INT_TX);
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable(INT_UART0);

	//Initialise the LCD
    OSRAMInit(false);
    OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	OSRAMStringDraw("LM3S811 demo", 16, 1);
}


/**
 * @brief Simulates a sensor that measures values between
 * 18 and 3 celsius
 * 
 * @param p 
 */
static void vTempSensorTask(void *pvParameters){

	int temp = 0;
	int t = 1;
	const TickType_t xDelay = 10; //100 Hz
	int delta = 1;
	// UBaseType_t taskWaterMark;
	// char aux[16];

	for(;;) {

		if(delta) {

			t++;
			if(t >= 128){
				delta = 0;
			}
			
		}
		else {
			t--;
			if(t <= 0){
				delta = 1;
			}
		}
	
		xQueueSend(xAvgQueue, &t, portMAX_DELAY);
		// taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		// sendToUART(longToChar(taskWaterMark, aux, 10));
		// sendToUART("\n");
		
		vTaskDelay(xDelay);
		
	}
}

/**
 * @brief Uses sensor values and applies a low pass filter.
 * Every value is the averageg of the latest N samples.
 * Additionaly shows the N, that can be changed via UART.
 * @param pvParameters 
 */
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

	// UBaseType_t taskWaterMark;
	// char aux[16];
	
	for(;;){

		int acum = 0;

		xQueueReceive(xAvgQueue, &rcv, portMAX_DELAY);
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

		// taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		// sendToUART(longToChar(taskWaterMark, aux, 10));
		// sendToUART("\n");

		vTaskDelay(xDelay);
		
	}

}
/**
 * @brief Shows the sampling that come from the sensor
 * 
 * @param pvParameters 
 */
static void vDisplayTask(void *pvParameters){

    char *message; // sample average
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
    unsigned char pucImage[128] = {0};
    uint8_t value_height;

	UBaseType_t taskWaterMark;
	char aux[16];

    for( ;; ){
       
        xQueueReceive(xPrintQueue, &message, portMAX_DELAY);
        
        value_height = (intptr_t)message/8;
		

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

		// taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		// sendToUART(longToChar(taskWaterMark, aux, 10));
		// sendToUART("\n");
    }
}

/**
 * @brief Get statistics from each task and sends it to UART
 * 
 * @param pvParameters 
 */
static void vTopTask( void *pvParameters ) {
	
	TickType_t xLastExecutionTime;
	char writeUART[150];
	
	//Caulculo de watermark
	UBaseType_t taskWaterMark;
	char aux[16];

	xLastExecutionTime = xTaskGetTickCount();

	for( ;; ) {
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainTOP_DELAY );

		getTasksStats(writeUART);

		UARTIntDisable( UART0_BASE, UART_INT_TX );
		{
			pcNextChar = writeUART;

			/* Send the first character. */
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}

			pcNextChar++;
		}
		UARTIntEnable(UART0_BASE, UART_INT_TX);	

		//Calculo de watermark
		//taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		//sendToUART(longToChar(taskWaterMark, aux, 10));
		//sendToUART("\n");
	}
}

/**
 * @brief Fills writeUART buffer with stats as: Task name, CPU use, percentage of the total run time of the task.
 * 
 * @param writeUART 
 */
void getTasksStats( char *writeUART ){

	TaskStatus_t *pxTaskStatusArray;
	volatile UBaseType_t uxArraySize, x;
	uint32_t ulTotalRunTime, ulStatsAsPercentage;
	char buffer[10];

	*writeUART = 0x00;

	uxArraySize = uxTaskGetNumberOfTasks();

	pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

	if( pxTaskStatusArray != NULL ) {
		// Generate raw status information about each task.
		uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

		// For percentage calculations.
		ulTotalRunTime /= 100UL;

		// Avoid divide by zero errors.
		if( ulTotalRunTime > 0 ) {

			
			strcat(writeUART,"Task\t\tRuntime\t\tRuntime [%]\n");
			strcat(writeUART,"----------------------------------------------\n");
	
			for( x = 0; x < uxArraySize; x++ ) {
				// What percentage of the total run time has the task used?
				// This will always be rounded down to the nearest integer.
				// ulTotalRunTimeDiv100 has already been divided by 100.
				ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

				if( ulStatsAsPercentage > 0UL ) {
					strcat(writeUART, pxTaskStatusArray[ x ].pcTaskName);
					strcat(writeUART,"\t\t");
					strcat(writeUART, longToChar(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
					strcat(writeUART,"\t\t");
					strcat(writeUART,longToChar(ulStatsAsPercentage, buffer, 10));
					strcat(writeUART,"%\r\n");
				}
				else {
				 	// If the percentage is zero here then the task has
				 	// consumed less than 1% of the total run time.
					strcat(writeUART, pxTaskStatusArray[ x ].pcTaskName);
					strcat(writeUART,"\t\t");
					strcat(writeUART, longToChar(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
					strcat(writeUART,"\t\t");
					strcat(writeUART,"1%");
					strcat(writeUART,"\r\n");
				}
				
				writeUART += strlen( ( char * ) writeUART );
			}
			strcat(writeUART, "\n\n");
			writeUART += strlen( ( char * ) writeUART );
		}
		// The array is no longer needed, free the memory it consumes.
		vPortFree( pxTaskStatusArray );
	}
}

/**
 * @brief Converts unisgned long to char and saves it into ptr argument
 * 
 * @param value unsigned long value
 * @param ptr saves the char result here
 * @param base 
 * @return char* 
 */
char* longToChar(unsigned long value, char *ptr, int base) {

	unsigned long t = 0, res = 0;
	unsigned long tmp = value;
	int count = 0;

	if (NULL == ptr) {
	return NULL;
	}

	if (tmp == 0) {
	count++;
	}

	while(tmp > 0) {
	tmp = tmp/base;
	count++;
	}

	ptr += count;

	*ptr = '\0';

	do {

	res = value - base * (t = value / base);
	if (res < 10) {
		* -- ptr = '0' + res;
	}
	else if ((res >= 10) && (res < 16)) {
		* --ptr = 'A' - 10 + res;
	}

	} while ((value = t) != 0);

	return(ptr);
}

/**
 * @brief converts integer into char
 * 
 * @param val 
 * @return char* 
 */
static char* intToChar(int val){

    static char buf[32] = {0};
    int i = 30;
	int base = 10;

    for(; val && i ; --i, val /= base)
        buf[i] = "0123456789abcdef"[val % base];

    return &buf[i+1];

}

/**
 * @brief sends a string of data to UART
 * 
 * @param data 
 */
static void sendToUART(char* data){
	UARTIntDisable(UART0_BASE, UART_INT_TX);
	{
		pcNextChar = data;

		/* Send the first character. */
		if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF)){
			HWREG(UART0_BASE + UART_O_DR) = *pcNextChar;
		}

		pcNextChar++;
	}

	UARTIntEnable(UART0_BASE, UART_INT_TX);
}

/**
 * @brief interrupt service rutine of UARTrx and UARTtx
 * 
 */
void vUART_ISR(void) {

	unsigned long ulStatus;
	char* rxChar;
	int rxNum;

	// What caused the interrupt. 
	ulStatus = UARTIntStatus(UART0_BASE, pdTRUE );
	
	//Clear the interrupt.
	UARTIntClear(UART0_BASE, ulStatus );

	//Was a Tx interrupt pending? 
	if(ulStatus & UART_INT_TX){
		//Send the next character in the string.  We are not using the FIFO. 
		if( *pcNextChar != 0 ){

			if(!(HWREG(UART0_BASE + UART_O_FR) & UART_FR_TXFF)){

				HWREG(UART0_BASE + UART_O_DR) = *pcNextChar;
			}
			pcNextChar++;
		}
	}

	if(ulStatus & UART_INT_RX){

		rxNum = UARTCharGet(UART0_BASE) - 48;

		xQueueSend(xUARTQueue, &rxNum, portMAX_DELAY);
		
	}
}