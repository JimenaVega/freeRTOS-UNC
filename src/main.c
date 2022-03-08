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

/* Demo board specifics. */
#define mainPUSH_BUTTON             GPIO_PIN_4

/* Misc. */
#define mainQUEUE_SIZE				( 3 )
#define mainDEBOUNCE_DELAY			( ( TickType_t ) 150 / portTICK_PERIOD_MS )
#define mainNO_DELAY				( ( TickType_t ) 0 )

#define N 10
#define STEP 8

/* String that is transmitted on the UART. */
static char *cMessage = "Task woken by button interrupt! --- ";
static volatile char *pcNextChar;
int idk = 1;

//Funciones
static void prvSetupHardware( void );
static void testTask(void *p);
static void generateTemperatureNum(void *p);
static char* intToChar(int val);
static void averageTask(void *pvParameters);

void getBaseGraph(unsigned char *pucImage, int avg);
static void vPrintTask( void *pvParameter );

/* The queue used to send strings to the print task for display on the LCD. */
QueueHandle_t xAvgQueue;
QueueHandle_t xUARTQueue;
QueueHandle_t xPrintQueue;

/*-----------------------------------------------------------*/

int main( void )
{
	/* Configure the clocks, UART and GPIO. */
	prvSetupHardware();

	/* Create the queue used to pass message to vPrintTask. */
	xAvgQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ));;
	xPrintQueue = xQueueCreate( mainQUEUE_SIZE, sizeof( char * ));;
	/* Start the tasks defined within the file. */
	xTaskCreate(averageTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	xTaskCreate(generateTemperatureNum, "test", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL);
	xTaskCreate(vPrintTask, "Print", configMINIMAL_STACK_SIZE, NULL, mainCHECK_TASK_PRIORITY, NULL );
	/* Start the scheduler. */
	vTaskStartScheduler();

	/* Will only get here if there was insufficient heap to start the
	scheduler. */

	return 0;
}



static void prvSetupHardware( void ){

	/* Setup the PLL. */
	SysCtlClockSet( SYSCTL_SYSDIV_10 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_6MHZ );

	/* Enable the UART.  */
	SysCtlPeripheralEnable(SYSCTL_PERIPH_UART0);

		/* Configure the UART for 8-N-1 operation. */
	UARTConfigSet( UART0_BASE, mainBAUD_RATE,
					   UART_CONFIG_WLEN_8 
					 | UART_CONFIG_PAR_NONE 
					 | UART_CONFIG_STOP_ONE );

	/* We don't want to use the fifo.  This is for test purposes to generate
	as many interrupts as possible. */
	HWREG( UART0_BASE + UART_O_LCR_H ) &= ~mainFIFO_SET;

	/* Enable Tx interrupts. */
	HWREG( UART0_BASE + UART_O_IM ) |= UART_INT_TX;
	IntPrioritySet( INT_UART0, configKERNEL_INTERRUPT_PRIORITY );
	IntEnable( INT_UART0 );

	/* Initialise the LCD> */
    OSRAMInit( false );
    // OSRAMStringDraw("www.FreeRTOS.org", 0, 0);
	// OSRAMStringDraw("LM3S811 demo", 16, 1);
}


//Mide entre 18 y 3 grados celsius
static void generateTemperatureNum(void *p){

	int temp = 0;
	int t = 1;
	const TickType_t xDelay = 10; //100 Hz
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

static void averageTask(void *pvParameters){

	int lastSamples[N] = {0};
	char* output = "";
	const TickType_t xDelay = 100; //100ms
	unsigned char *pucImage;
	uint8_t y_axis=0;

	int rcv = 0;
	int avg = 0;

	for(;;){

		int acum = 0;

		xQueueReceive(xAvgQueue, &rcv, portMAX_DELAY );
		
		/* shifting array elements */
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
		// //Write the message to the LCD.
		//OSRAMClear();
		//OSRAMStringDraw( output ,  0x3f, 0x01);
		
		
	}

}

static void vPrintTask(void *pvParameters){
    char *pcMessage;
    unsigned portBASE_TYPE uxLine = 0, uxRow = 0;
    unsigned char pucImage[128] = {0};
    uint8_t value_height;

    for( ;; )
    {
        /* Wait for a message to arrive. */
        xQueueReceive(xPrintQueue, &pcMessage, portMAX_DELAY );

        /* Write the message to the LCD. */
        uxRow++;
        uxLine++;
        OSRAMClear();
        // OSRAMStringDraw( pcMessage, 0/*uxLine & 0x3f*/, 0/*uxRow & 0x01*/);
        
        value_height = (uint8_t)pcMessage/8;

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



void vUART_ISR(void) {

	unsigned long ulStatus;

	/* What caused the interrupt. */
	ulStatus = UARTIntStatus( UART0_BASE, pdTRUE );

	/* Clear the interrupt. */
	UARTIntClear( UART0_BASE, ulStatus );

	/* Was a Tx interrupt pending? */
	if( ulStatus & UART_INT_TX )
	{
		/* Send the next character in the string.  We are not using the FIFO. */
		if( *pcNextChar != 0 )
		{
			if( !( HWREG( UART0_BASE + UART_O_FR ) & UART_FR_TXFF ) )
			{
				HWREG( UART0_BASE + UART_O_DR ) = *pcNextChar;
			}
			pcNextChar++;
		}
	}
}


