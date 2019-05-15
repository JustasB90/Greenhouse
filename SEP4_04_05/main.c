/*
 * SEP4_04_05.c
 *
 * Created: 04/05/2019 15.06.52
 * Author : Justas
 */ 
#include <ATMEGA_FreeRTOS.h>
#include <timers.h>
#include <task.h>
#include <queue.h>
#include <avr/io.h>
#include <stdio.h>
#include <stdio_driver.h>
#include <avr/interrupt.h>
#include <util/delay.h>
#include <hih8120.h> 
#include <mh_z19.h>
#include <avr/sfr_defs.h>
#include <stdint.h>
#include <semphr.h>
#include "mpu_wrappers.h"
#include <lora_driver.h>
#include <iled.h>
#include <stddef.h>


#define LED_TASK_PRIORITY 7
#define LORA_appEUI "f6d3bd78bce55453"
#define LORA_appKEY "80be447b508913441700e22d96ec386b"

// driver init methods
void initHIH8210(void);
void initLora(void);
//  temperature and humidity task
void temp_hum_co2_task( void *pvParameters );
// lora handler task
void lora_handler_task( void *pvParameters );

// task handles

TaskHandle_t temp_hum_co2_task_handle;
TaskHandle_t lora_task_handle;

// task handlers

static lora_payload_t _uplink_payload;

// semaphore handle
SemaphoreHandle_t xPrintfSemaphore;
SemaphoreHandle_t xCo2Semaphore;

//queues
QueueHandle_t	temp_q;
QueueHandle_t	hum_q;
QueueHandle_t	co2_q;	

static char _out_buf[100];

static uint16_t global_co2=0;
static void _lora_setup(void)
{
	e_LoRa_return_code_t rc;
	//led_slow_blink(led_ST2); // OPTIONAL: Led the green led blink slowly while we are setting up LoRa
	
	// Factory reset the transceiver
	printf("FactoryReset >%s<\n", lora_driver_map_return_code_to_text(lora_driver_rn2483_factory_reset()));
	
	// Configure to EU868 LoRaWAN standards
	printf("Configure to EU868 >%s<\n", lora_driver_map_return_code_to_text(lora_driver_configure_to_eu868()));

	// Get the transceivers HW EUI
	rc = lora_driver_get_rn2483_hweui(_out_buf);
	printf("Get HWEUI >%s<: %s\n",lora_driver_map_return_code_to_text(rc), _out_buf);

	// Set the HWEUI as DevEUI in the LoRaWAN software stack in the transceiver
	printf("Set DevEUI: %s >%s<\n", _out_buf, lora_driver_map_return_code_to_text(lora_driver_set_device_identifier(_out_buf)));

	// Set Over The Air Activation parameters to be ready to join the LoRaWAN
	printf("Set OTAA Identity appEUI:%s appKEY:%s devEUI:%s >%s<\n", LORA_appEUI, LORA_appKEY, _out_buf, lora_driver_map_return_code_to_text(lora_driver_set_otaa_identity(LORA_appEUI,LORA_appKEY,_out_buf)));

	// Save all the MAC settings in the transceiver
	printf("Save mac >%s<\n",lora_driver_map_return_code_to_text(lora_driver_save_mac()));

	// Enable Adaptive Data Rate
	printf("Set Adaptive Data Rate: ON >%s<\n", lora_driver_map_return_code_to_text(lora_driver_set_adaptive_data_rate(LoRa_ON)));
	
	// Join the LoRaWAN
	uint8_t maxJoinTriesLeft = 5;
	do {
		rc = lora_driver_join(LoRa_OTAA);
		printf("Join Network TriesLeft:%d >%s<\n", maxJoinTriesLeft, lora_driver_map_return_code_to_text(rc));

		if ( rc != LoRa_ACCEPTED)
		{
			// Make the red led pulse to tell something went wrong
			//led_long_puls(led_ST1); // OPTIONAL
			// Wait 5 sec and lets try again
			vTaskDelay(pdMS_TO_TICKS(5000UL));
		}
		else
		{
			break;
		}
	} while (--maxJoinTriesLeft);

	if (rc == LoRa_ACCEPTED)
	{
		// Connected to LoRaWAN :-)
		// Make the green led steady
		//led_led_on(led_ST2); // OPTIONAL
	}
	else
	{
		// Something went wrong
		// Turn off the green led
		//led_led_off(led_ST2); // OPTIONAL
		// Make the red led blink fast to tell something went wrong
		//led_fast_blink(led_ST1); // OPTIONAL

		// Lets stay here
		while (1)
		{
			taskYIELD();
		}
	}
}

void create_tasks_and_semaphores(void)
{
	// Semaphores are useful to stop a Task proceeding, where it should be paused to wait,
	// because it is sharing a resource, such as the Serial port.
	// Semaphores should only be used whilst the scheduler is running, but we can set it up here.
	
	
	if ( xPrintfSemaphore == NULL )  // Check to confirm that the Semaphore has not already been created.
	{
		xPrintfSemaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore.
		if ( ( xPrintfSemaphore ) != NULL )
		{
			xSemaphoreGive( ( xPrintfSemaphore ) );  // Make the mutex available for use, by initially "Giving" the Semaphore.
		}
	}
	if ( xCo2Semaphore == NULL )  // Check to confirm that the Semaphore has not already been created.
	{
		xCo2Semaphore = xSemaphoreCreateMutex();  // Create a mutex semaphore.
		if ( ( xCo2Semaphore ) != NULL )
		{
			xSemaphoreGive( ( xCo2Semaphore ) );  // Make the mutex available for use, by initially "Giving" the Semaphore.
		}
	}
			
	//Create queues
	temp_q = xQueueCreate(1, sizeof(uint16_t));	
	hum_q = xQueueCreate(1, sizeof(uint16_t));	
	co2_q = xQueueCreate(1, sizeof(uint16_t));
	
	//Create tasks	
	xTaskCreate(
	temp_hum_co2_task
	,	(const portCHAR *)"Temperature, humidity, co2"
	,	configMINIMAL_STACK_SIZE+300 
	,	NULL
	,	2
	,	&temp_hum_co2_task_handle);
	
	xTaskCreate(
	lora_handler_task
	,  (const portCHAR *)"LRHand"  // A name just for humans
	,  configMINIMAL_STACK_SIZE+300  // This stack size can be checked & adjusted by reading the Stack Highwater
	,  NULL
	,  2  // Priority, with 3 (configMAX_PRIORITIES - 1) being the highest, and 0 being the lowest.
	,  &lora_task_handle);
	
	
}

void temp_hum_co2_task( void *pvParameters )
{
	#if (configUSE_APPLICATION_TASK_TAG == 1)
	// Set task no to be used for tracing with R2R-Network
	vTaskSetApplicationTaskTag( NULL, ( void * ) 1 );
	#endif
		
	uint16_t temperature =0;
	uint16_t humidity=0;		
		
	for(;;)
	{		
		
		if (HIH8120_OK != hih8120Wakeup()){
			//printf("wake up failed\n");
		}				
		vTaskDelay(100);		
		if (HIH8120_OK != hih8120Meassure()){
			//printf("data polling failed\n");
		}			
		vTaskDelay(100);		
				
		//get measurements
		
		temperature=hih8120GetTemperature_x10();
		humidity=hih8120GetHumidityPercent_x10();
		xSemaphoreTake(xPrintfSemaphore,portMAX_DELAY);		
		printf("PROD:Temp: %d\n", temperature);
		printf("PROD:Hum: %d\n", humidity);		
		xSemaphoreGive(xPrintfSemaphore);		
		
		//send humidity and temperature to the queue.		
		xQueueSend(temp_q, //queue handle
		(void*) &temperature,	//pointer to the temp
		0);						
				
		xQueueSend(hum_q,	//queue handle
		(void*) &humidity,	//pointer to the humidity
		0);		
		//get co2 and wait for co2 semaphore
		getCo2();
		xSemaphoreTake(xCo2Semaphore,portMAX_DELAY);
		
		xQueueSend(co2_q,
		(void*) &global_co2,
		0);
		xSemaphoreGive(xCo2Semaphore);
		
		vTaskDelay(7000);
		/*xSemaphoreTake(xPrintfSemaphore,portMAX_DELAY);
		UBaseType_t spaces = uxQueueSpacesAvailable(temp_hum_co2_q);
		printf("PROD:Q SP: %d\n", spaces);		
		xSemaphoreGive(xPrintfSemaphore);	*/
		
	}
	
}


void lora_handler_task( void *pvParameters )
{
	
	lora_driver_reset_rn2483(1);
	vTaskDelay(2);
	lora_driver_reset_rn2483(0);
	vTaskDelay(150);
	
	e_LoRa_return_code_t rc;
		
	lora_driver_flush_buffers(); // get rid of first version string from module after reset!
	_lora_setup();
		
	uint16_t co2_fromQueue=0;
	uint16_t temp_fromQueue=0;
	uint16_t hum_fromQueue=0;
			
	for(;;){
				
		BaseType_t em1 =xQueueReceive(temp_q, // queue handle
									  &temp_fromQueue, // address of temperature placeholder
									  portMAX_DELAY);			  // time out if the queue is empty
											
		BaseType_t em2 =xQueueReceive(hum_q, // queue handle
									  &hum_fromQueue, // address of humidity placeholder
									  portMAX_DELAY);			  // time out if the queue is empty
		
		BaseType_t em3 =xQueueReceive(co2_q,
									  &co2_fromQueue,
									  portMAX_DELAY);	
		/* PRINTING RESULTS AND AVAILABLE SPACES IN THE QUEUE AFTER RECEIVING */
		xSemaphoreTake(xPrintfSemaphore,portMAX_DELAY);										  
		printf("READER:TEMPERATURE FR Q: %d\n", temp_fromQueue);									  	
		printf("READER:HUMIDITY FR Q: %d\n", hum_fromQueue);
		printf("READER:CO2 FR Q: %d\n", co2_fromQueue);						
		xSemaphoreGive(xPrintfSemaphore);
		
		_uplink_payload.len =6;
		_uplink_payload.port_no =1;
		_uplink_payload.bytes[0] = temp_fromQueue >> 8;
		_uplink_payload.bytes[1] = temp_fromQueue & 0xFF;
		_uplink_payload.bytes[2] = hum_fromQueue >> 8;
		_uplink_payload.bytes[3] = hum_fromQueue & 0xFF;
		_uplink_payload.bytes[4] = co2_fromQueue >> 8;
		_uplink_payload.bytes[5] = co2_fromQueue & 0xFF;
			
		//send the payload
		xSemaphoreTake(xPrintfSemaphore,portMAX_DELAY);			
		printf("Upload Message >%s<\n", lora_driver_map_return_code_to_text(lora_driver_sent_upload_message(true, &_uplink_payload)));
		xSemaphoreGive(xPrintfSemaphore);
					
		}	
}

void my_co2_call_back(uint16_t ppm){
		
	printf("CO2 ppm is: %d\n",ppm); //check the value
	xSemaphoreTake(xCo2Semaphore,portMAX_DELAY);
	global_co2=ppm;
	xSemaphoreGive(xCo2Semaphore);
		
}

void getCo2(){
	uint16_t ppm=0;
	mh_z19_return_code_t rc;	
	rc = mh_z19_take_meassuring();	 
}

int main(void)
{
	mh_z19_create(ser_USART3, my_co2_call_back);
	stdioCreate(0);
	sei();		
	initHIH8210();
	initLora();	
	puts("Program Started");
	
	create_tasks_and_semaphores();				
	vTaskStartScheduler();
			
	while (1)
	{				
	}
}

void initHIH8210(void){
	if (HIH8120_OK == hih8120Create())
	{
		//DRIVER CREATED, CHECK OUTPUT
		printf("hih8210Create return code is: %d  We need 0\n", hih8120Create());
	}
}

void initLora(void){
	hal_create(LED_TASK_PRIORITY);
	lora_driver_create(ser_USART1);
}


	
