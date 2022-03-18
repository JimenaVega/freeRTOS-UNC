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

#include <string.h>

#define configSENSOR_STACK_SIZE     ( ( unsigned short ) (40) )  //watermark = 32
#define configFILTER_STACK_SIZE     ( ( unsigned short ) (60 )  // watermark = 12
#define configDISPLAY_STACK_SIZE     ( ( unsigned short ) (86) ) //en un comienzo dio 0
//#define configCMD_STACK_SIZE     ( ( unsigned short ) (42) )
#define configTOP_STACK_SIZE     ( ( unsigned short ) (100) ) //en un comienzo dio 0
//mine
//#include <math.h>

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

// /* Demo board specifics. */
// #define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )
#define mainTOP_DELAY				( ( TickType_t ) 2000 / portTICK_PERIOD_MS )   


//#define N 10
#define STEP 8
#define INIT_N 10

//--------------CPU load------------------------
static volatile unsigned int uxPercentLoadCPU = (unsigned int) 0U; // Last CPU load calculated
static volatile unsigned int uxIdleTickCount  = (unsigned int) 0U; // How many ticks were in idle task
static volatile unsigned int uxCPULoadCount   = (unsigned int) 0U; // For 0 to configTICKRATE_HZ we will count idle tasks

//----------------------------
/* String that is transmitted on the UART. */
static volatile char *pcNextChar;



//Funciones
static void prvSetupHardware( void );
static char* intToChar(int val);
char* ultostr(unsigned long value, char *ptr, int base);
void vTaskGetStats( char *pcWriteBuffer );
void printStatistics(char* taskName, UBaseType_t waterMark);
static void sendToUART(char* data);
//Task
static void vDisplayTask( void *pvParameter );
static void vWaterMark(void *pvParameters);
static void vFilterTask(void *pvParameters);
static void vTempSensorTask(void *p);
static void vTopTask( void *pvParameters );


static void testTask(void *p);
void getBaseGraph(unsigned char *pucImage, int avg);

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


//Mide entre 18 y 3 grados celsius
static void vTempSensorTask(void *p){

	int temp = 0;
	int t = 1;
	const TickType_t xDelay = 10; //100 Hz
	int delta = 1;
	// UBaseType_t taskWaterMark;
	// char aux[16];

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

		

		// taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		// sendToUART(ultostr(taskWaterMark, aux, 10));
		// sendToUART("\n");
		
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
		// sendToUART(ultostr(taskWaterMark, aux, 10));
		// sendToUART("\n");

		vTaskDelay(xDelay);
		
	}

}

static void vDisplayTask(void *pvParameters){

    char *message;
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
    unsigned char pucImage[128] = {0};
    uint8_t value_height;

	UBaseType_t taskWaterMark;
	char aux[16];

    for( ;; ){
       
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

		// taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		// sendToUART(ultostr(taskWaterMark, aux, 10));
		// sendToUART("\n");
    }
}


static void vTopTask( void *pvParameters )
{
	
	TickType_t xLastExecutionTime;
	char pcWriteBuffer[150];


	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
	works correctly. */
	xLastExecutionTime = xTaskGetTickCount();
	UBaseType_t taskWaterMark;
	char aux[16];

	for( ;; )
	{
		/* Perform this check every mainCHECK_DELAY milliseconds. */
		vTaskDelayUntil( &xLastExecutionTime, mainTOP_DELAY );

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

		taskWaterMark = uxTaskGetStackHighWaterMark(NULL);
		sendToUART(ultostr(taskWaterMark, aux, 10));
		//sendToUART("\n");
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
				strcat(pcWriteBuffer,"Task\t\tAbs Time\t%Time\n");
				strcat(pcWriteBuffer,"-------------------------------------\n");
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
						strcat(pcWriteBuffer,"\t\t");
						strcat(pcWriteBuffer, ultostr(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
						strcat(pcWriteBuffer,"\t\t");
						strcat(pcWriteBuffer,ultostr(ulStatsAsPercentage, buffer, 10));
						strcat(pcWriteBuffer,"%\r\n");
					}
					else
					{
					// 	// If the percentage is zero here then the task has
					// 	// consumed less than 1% of the total run time.
						strcat(pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName);
						strcat(pcWriteBuffer,"\t\t");
						strcat(pcWriteBuffer, ultostr(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
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
			vPortFree( pxTaskStatusArray );
		}
}

char *ultostr(unsigned long value, char *ptr, int base)
{
  unsigned long t = 0, res = 0;
  unsigned long tmp = value;
  int count = 0;

  if (NULL == ptr)
  {
    return NULL;
  }

  if (tmp == 0)
  {
    count++;
  }

  while(tmp > 0)
  {
    tmp = tmp/base;
    count++;
  }

  ptr += count;

  *ptr = '\0';

  do
  {
    res = value - base * (t = value / base);
    if (res < 10)
    {
      * -- ptr = '0' + res;
    }
    else if ((res >= 10) && (res < 16))
    {
        * --ptr = 'A' - 10 + res;
    }
  } while ((value = t) != 0);

  return(ptr);
}



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

		// rxChar = intToChar(rxNum);
		
		// OSRAMClear();
		// OSRAMStringDraw(rxChar , 0, 0);

		xQueueSend(xUARTQueue, &rxNum, portMAX_DELAY);
		
	}
}


//--------------------------------------------------
/*
 * FreeRTOS Kernel V10.3.0
 * Copyright (C) 2020 Amazon.com, Inc. or its affiliates.  All Rights Reserved.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy of
 * this software and associated documentation files (the "Software"), to deal in
 * the Software without restriction, including without limitation the rights to
 * use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of
 * the Software, and to permit persons to whom the Software is furnished to do so,
 * subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 * http://www.FreeRTOS.org
 * http://aws.amazon.com/freertos
 *
 * 1 tab == 4 spaces!
 */





// /* Demo app includes. */
// #include "integer.h"
// #include "PollQ.h"
// #include "semtest.h"
// #include "BlockQ.h"

// /* Delay between cycles of the 'check' task. 100 ms */
// #define mainSENSOR_DELAY						( ( TickType_t ) 10 / portTICK_PERIOD_MS )
// #define mainDISPLAY_DELAY						( ( TickType_t ) 200 / portTICK_PERIOD_MS )
// #define mainTOP_DELAY						( ( TickType_t ) 2000 / portTICK_PERIOD_MS )   

// /* UART configuration - note this does not use the FIFO so is not very
// efficient. */
// #define mainFIFO_SET				( 0x10 )
// /* The baud rate used by the UART comms tasks/co-routine. */
// #define mainBAUD_RATE				( 57600 )

// /* Demo task priorities. */
// #define mainQUEUE_POLL_PRIORITY		( tskIDLE_PRIORITY + 2 )
// #define mainCHECK_TASK_PRIORITY		( tskIDLE_PRIORITY + 3 )
// #define mainSEM_TEST_PRIORITY		( tskIDLE_PRIORITY + 1 )
// #define mainBLOCK_Q_PRIORITY		( tskIDLE_PRIORITY + 2 )
// #define mainCOMMS_RX_TASK_PRIORITY	( tskIDLE_PRIORITY + 1 )


// /* Misc. */
// #define mainQUEUE_SIZE				( 3 )
// #define mainRX_QUEUE_LEN			( 5 )
// #define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
// #define mainNO_DELAY				( ( TickType_t ) 0 )


// /* The time the Comms Rx task should wait to receive a character.  This should
// be slightly longer than the time between transmissions.  If we do not receive
// a character after this time then there must be an error in the transmission or
// the timing of the transmission. */

// #define max_N 10
// #define min_N 1
// /*
//  * Configure the processor and peripherals for this demo.
//  */
// static void prvSetupHardware( void );

// /*
//  * The 'check' task, as described at the top of this file.
//  */
// static void vSensorTask( void *pvParameters );


// /*
//  * The task that controls access to the LCD.
//  */
// static void vDisplayTask( void *pvParameter );

// static void vTopTask( void *pvParameters );

// static void vFilterTask( void *pvParameter );

// /*
//  * The task that receives the characters from UART 0.
//  */
// static void vCommsRxTask( void * pvParameters );

// void vTaskGetStats( char *pcWriteBuffer );
// char *ultostr(unsigned long value, char *ptr, int base);


// /* The queue used to send values to the Filter task */
// QueueHandle_t xSensorQueue;

// /* The queue used to transmit characters from the interrupt to the Comms Rx
// task. */
// static QueueHandle_t xCommsQueue;

// int avg_value;
// int filter_value;
// static const int n_values[] = {1,2,3,4,5,6,7,8,9,10};
// int values[max_N] = {0,0,0,0,0,0,0,0,0,0};

// /* The next character to transmit. */
// static volatile char *pcNextChar;


// static const char display_values[][2] = {
// 					{0x00, 0x80},
// 					{0x00, 0x40},
// 					{0x00, 0x20},
// 					{0x00, 0x10},
// 					{0x00, 0x08},
// 					{0x00, 0x04},
// 					{0x00, 0x02},
// 					{0x00, 0x01},
// 					{0x80, 0x00},
// 					{0x40, 0x00},
// 					{0x20, 0x00},
// 					{0x10, 0x00},
// 					{0x08, 0x00},
// 					{0x04, 0x00},
// 					{0x02, 0x00},
// 					{0x01, 0x00},
//     };

// /*-----------------------------------------------------------*/

// int main( void )
// {
// 	/* Create the queue used to communicate between the UART ISR and the Comms
// 	Rx task. */
// 	xCommsQueue = xQueueCreate( mainRX_QUEUE_LEN, sizeof( char ) );
	
// 	/* Configure the clocks, UART and GPIO. */
// 	prvSetupHardware();

// 	/* Create the queue used to pass message to vFilterTask. */
// 	xSensorQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( int ) );

// 	/* Start the tasks defined within the file. */
// 	xTaskCreate( vSensorTask, "Sensor", configSENSOR_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
// 	xTaskCreate( vFilterTask, "Filter", configFILTER_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
// 	xTaskCreate( vDisplayTask, "Display", configDISPLAY_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );
// 	xTaskCreate( vCommsRxTask, "cmd", configCMD_STACK_SIZE, NULL, mainCOMMS_RX_TASK_PRIORITY, NULL );
// 	xTaskCreate( vTopTask, "Top", configTOP_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY - 1, NULL );


// 	filter_value = 9;
	
// 	/* Start the scheduler. */
// 	vTaskStartScheduler();

// 	/* Will only get here if there was insufficient heap to start the
// 	scheduler. */

// 	return 0;
// }
// /*-----------------------------------------------------------*/

// static void vSensorTask( void *pvParameters )
// {
// 	TickType_t xLastExecutionTime;
// 	int value = 15;
// 	int mode = 1;

// 	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
// 	works correctly. */
// 	xLastExecutionTime = xTaskGetTickCount();
	
// 	for( ;; )
// 	{
// 		/* Perform this check every mainCHECK_DELAY milliseconds. */
// 		vTaskDelayUntil( &xLastExecutionTime, mainSENSOR_DELAY );
		
// 		if(++value == 31) value = 15;

// 		xQueueSend( xSensorQueue, &value, portMAX_DELAY );
// 	}
// }
// /*-----------------------------------------------------------*/

// static void prvSetupHardware( void )
// {

// 	/* Setup the PLL. */
// 	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );
	
// 	/* Enable the UART.  */
// 	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

// 	/* Configure the UART for 8-N-1 operation. */
// 	UARTConfigSet( UART0_BASE, mainBAUD_RATE, UART_CONFIG_WLEN_8 | UART_CONFIG_PAR_NONE | UART_CONFIG_STOP_ONE );

// 	/* We don't want to use the fifo.  This is for test purposes to generate
// 	as many interrupts as possible. */
// 	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

// 	/* Enable Rx interrupts. */
// 	HWREG( UART0_BASE + UART_O_IM ) |= ( UART_INT_TX | UART_INT_RX );
// 	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
// 	IntEnable( INT_UART0 );


// 	/* Initialise the LCD> */
//     OSRAMInit( false );
//     OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
// 	OSRAMStringDraw("LM3S811 demo", 16, 1);
// }

// void vUART_ISR(void)
// {
// 	unsigned long ulStatus;
// 	char cRxedChar;
// 	portBASE_TYPE xHigherPriorityTaskWoken = pdFALSE;
// 	/* What caused the interrupt. */
// 	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

// 	/* Clear the interrupt. */
// 	UARTIntClear( UART0_BASE, ulStatus );

// 	/* Was a Rx interrupt pending? */
// 	if( ulStatus & UART_INT_RX )
// 	{
// 		if( ( HWREG(UART0_BASE + UART_O_FR ) & UART_FR_RXFF ) )
// 		{
// 			/* Get the char from the buffer and post it onto the queue of
// 			Rxed chars.  Posting the character should wake the task that is 
// 			blocked on the queue waiting for characters. */
// 			cRxedChar = ( char ) HWREG( UART0_BASE + UART_O_DR );
// 			xQueueSendFromISR( xCommsQueue, &cRxedChar, &xHigherPriorityTaskWoken );
// 		}		
// 	}

// 		/* Was a Tx interrupt pending? */
// 	if( ulStatus & UART_INT_TX )
// 	{
// 		/* Send the next character in the string.  We are not using the FIFO. */
// 		if( *pcNextChar != 0 )
// 		{
// 			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
// 			{
// 				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
// 			}
// 			pcNextChar++;
// 		}
// 	}

// 	/* If a task was woken by the character being received then we force
// 	a context switch to occur in case the task is of higher priority than
// 	the currently executing task (i.e. the task that this interrupt 
// 	interrupted.) */
// 	portEND_SWITCHING_ISR( xHigherPriorityTaskWoken );
// }
// /*-----------------------------------------------------------*/

// static void vFilterTask( void *pvParameters )
// {
// 	int value, aux, aux1, i;
// 	int index = 0;

// 	for( ;; )
// 	{
// 		/* Wait for a message to arrive. */
// 		xQueueReceive( xSensorQueue, &value, portMAX_DELAY );
// 		aux = values[0];
// 		values[0] = value;
// 		for(i = 1; i < max_N; i++)
// 		{
// 			aux1 = values[i];
// 			values[i] = aux;
// 			aux = aux1;
// 		}
// 		aux = 0;
// 		index = 0;
// 		for(i = 0; i < n_values[filter_value]; i++)
// 		{
// 			if(values[i] != 0)
// 			{
// 				aux += values[i];
// 				index++;
// 			}
// 		}

// 		avg_value = aux / index;
// 	}
// }

// /*-----------------------------------------------------------*/

// static void vDisplayTask( void *pvParameters )
// {
// 	TickType_t xLastExecutionTime;
// 	int value = 15;
// 	int mode = 1;
// 	static unsigned long ulX = 0;
// 	static unsigned long ulY = 0;

// 	OSRAMClear();
// 	OSRAMStringDraw("30", 85, 0);
// 	OSRAMStringDraw("15", 85, 1);
	
// 	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
// 	works correctly. */
// 	xLastExecutionTime = xTaskGetTickCount();

// 	for( ;; )
// 	{
// 		/* Perform this check every mainCHECK_DELAY milliseconds. */
// 		vTaskDelayUntil( &xLastExecutionTime, mainDISPLAY_DELAY );
		
// 		OSRAMImageDraw (display_values[avg_value - 15],ulX,0,1,2);
// 		if(++ulX == 85) 
// 		{
// 			ulX = 10;
// 			OSRAMClear();
// 			OSRAMStringDraw("30", 85, 0);
// 			OSRAMStringDraw("15", 85, 1);
// 		}
// 	}
// }
// /*-----------------------------------------------------------*/

// static void vCommsRxTask( void * pvParameters )
// {
// 	char rec_uart;
// 	for( ;; )
// 	{
// 		/* Wait for a character to be received. */
// 		xQueueReceive( xCommsQueue,&rec_uart, portMAX_DELAY );
		
// 		//from 1 to :(10) only
// 		if(rec_uart >= (min_N + 48) && rec_uart <= (max_N + 48))
// 		{
// 			filter_value = rec_uart - 49;
// 		}
// 	}
// }
// /*-----------------------------------------------------------*/

// static void vTopTask( void *pvParameters )
// {
	
// 	TickType_t xLastExecutionTime;
// 	char pcWriteBuffer[150];


// 	/* Initialise xLastExecutionTime so the first call to vTaskDelayUntil()
// 	works correctly. */
// 	xLastExecutionTime = xTaskGetTickCount();

// 	for( ;; )
// 	{
// 		/* Perform this check every mainCHECK_DELAY milliseconds. */
// 		vTaskDelayUntil( &xLastExecutionTime, mainTOP_DELAY );

// 		vTaskGetStats(pcWriteBuffer);

// 		UARTIntDisable( UART0_BASE, UART_INT_TX );
// 		{
// 			pcNextChar = pcWriteBuffer;

// 			/* Send the first character. */
// 			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
// 			{
// 				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
// 			}

// 			pcNextChar++;
// 		}
// 		UARTIntEnable(UART0_BASE, UART_INT_TX);	
// 	}
// }

// /*-----------------------------------------------------------*/

// void vTaskGetStats( char *pcWriteBuffer )
// {
// 	TaskStatus_t *pxTaskStatusArray;
// 	volatile UBaseType_t uxArraySize, x;
// 	uint32_t ulTotalRunTime, ulStatsAsPercentage;
// 	char buffer[10];

// 		// Make sure the write buffer does not contain a string.
// 		*pcWriteBuffer = 0x00;

// 		// Take a snapshot of the number of tasks in case it changes while this
// 		// function is executing.
// 		uxArraySize = uxTaskGetNumberOfTasks();

// 		// Allocate a TaskStatus_t structure for each task.  An array could be
// 		// allocated statically at compile time.
// 		pxTaskStatusArray = pvPortMalloc( uxArraySize * sizeof( TaskStatus_t ) );

// 		if( pxTaskStatusArray != NULL )
// 		{
// 			// Generate raw status information about each task.
// 			uxArraySize = uxTaskGetSystemState( pxTaskStatusArray, uxArraySize, &ulTotalRunTime );

// 			// For percentage calculations.
// 			ulTotalRunTime /= 100UL;

// 			// Avoid divide by zero errors.
// 			if( ulTotalRunTime > 0 )
// 			{
// 				strcat(pcWriteBuffer,"Task\t\tAbs Time\t%Time\n");
// 				strcat(pcWriteBuffer,"-------------------------------------\n");
// 				// For each populated position in the pxTaskStatusArray array,
// 				// format the raw data as human readable ASCII data
// 				for( x = 0; x < uxArraySize; x++ )
// 				{
// 					// What percentage of the total run time has the task used?
// 					// This will always be rounded down to the nearest integer.
// 					// ulTotalRunTimeDiv100 has already been divided by 100.
// 					ulStatsAsPercentage = pxTaskStatusArray[ x ].ulRunTimeCounter / ulTotalRunTime;

// 					if( ulStatsAsPercentage > 0UL )
// 					{
// 						strcat(pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName);
// 						strcat(pcWriteBuffer,"\t\t");
// 						strcat(pcWriteBuffer, ultostr(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
// 						strcat(pcWriteBuffer,"\t\t");
// 						strcat(pcWriteBuffer,ultostr(ulStatsAsPercentage, buffer, 10));
// 						strcat(pcWriteBuffer,"%\r\n");
// 					}
// 					else
// 					{
// 					// 	// If the percentage is zero here then the task has
// 					// 	// consumed less than 1% of the total run time.
// 						strcat(pcWriteBuffer, pxTaskStatusArray[ x ].pcTaskName);
// 						strcat(pcWriteBuffer,"\t\t");
// 						strcat(pcWriteBuffer, ultostr(pxTaskStatusArray[ x ].ulRunTimeCounter, buffer, 10));
// 						strcat(pcWriteBuffer,"\t\t");
// 						strcat(pcWriteBuffer,"<1%");
// 						strcat(pcWriteBuffer,"\r\n");
// 					}
					
// 					pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
// 				}
// 				strcat(pcWriteBuffer, "\n\n");
// 				pcWriteBuffer += strlen( ( char * ) pcWriteBuffer );
// 			}

// 			// The array is no longer needed, free the memory it consumes.
// 			vPortFree( pxTaskStatusArray );
// 		}
// }

// char *ultostr(unsigned long value, char *ptr, int base)
// {
//   unsigned long t = 0, res = 0;
//   unsigned long tmp = value;
//   int count = 0;

//   if (NULL == ptr)
//   {
//     return NULL;
//   }

//   if (tmp == 0)
//   {
//     count++;
//   }

//   while(tmp > 0)
//   {
//     tmp = tmp/base;
//     count++;
//   }

//   ptr += count;

//   *ptr = '\0';

//   do
//   {
//     res = value - base * (t = value / base);
//     if (res < 10)
//     {
//       * -- ptr = '0' + res;
//     }
//     else if ((res >= 10) && (res < 16))
//     {
//         * --ptr = 'A' - 10 + res;
//     }
//   } while ((value = t) != 0);

//   return(ptr);
// }