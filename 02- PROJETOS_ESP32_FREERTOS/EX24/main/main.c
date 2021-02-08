/* --------------------------------------------------------------------------
  Autor: Prof° Fernando Simplicio;
  Hardware: NodeMCU ESP32
  Espressif SDK-IDF: v3.2
  Curso: Formação em Internet das Coisas (IoT) com ESP32
  Link: https://www.microgenios.com.br/formacao-iot-esp32/
 *  --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
   
/*
 * Lib C
 */
#include <stdio.h>
#include <stdint.h>  
#include <string.h>

/*
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/*
 * Drivers
 */
#include "driver/gpio.h"

/*
 * logs
 */
#include "esp_log.h"

/*
 * Definições Gerais
 */
#define DEBUG 1

#define LED_BUILDING	2    
#define GPIO_OUTPUT_PIN_SEL  (1ULL<<LED_BUILDING)

#define BUTTON_1	17 
#define GPIO_INPUT_PIN_SEL  (1ULL<<BUTTON_1)

/*
 * Variável Global. 
 */
static const char * TAG = "Main: ";
static TaskHandle_t taskHandle;

/**
 * Protótipos
 */
void Task_Led( void *pvParameter ); 


/**
 * Função de callback chamada por uma ISR.
 */
static void IRAM_ATTR gpio_isr_handler( void * pvParameter)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    if( BUTTON_1 == (uint32_t) pvParameter )
	{
		vTaskNotifyGiveFromISR( taskHandle, &xHigherPriorityTaskWoken );
        
        if( xHigherPriorityTaskWoken == pdTRUE )
        {
            portYIELD_FROM_ISR();
        }
	
	}   
		
}
 
void Task_Led( void * pvParameter )
{
    uint32_t cnt = 0;
    uint32_t nNotify = 0;

    if( DEBUG )
        ESP_LOGI( TAG, "task_led running.\n"); 

	/**
	 * Led Building
	 */
	gpio_config_t io_conf1 = {
		.intr_type = GPIO_PIN_INTR_DISABLE,
		.mode = GPIO_MODE_OUTPUT,
		.pin_bit_mask = GPIO_OUTPUT_PIN_SEL
	};	
	gpio_config(&io_conf1);  
	
	/**
	 * Button (por interrupção externa)
	 */
	gpio_config_t io_conf2 = {
		.intr_type = GPIO_INTR_NEGEDGE,
		.mode = GPIO_MODE_INPUT,
		.pin_bit_mask = GPIO_INPUT_PIN_SEL,
		.pull_up_en = GPIO_PULLUP_ENABLE,
		.pull_down_en = GPIO_PULLDOWN_DISABLE
	};
    gpio_config(&io_conf2); 

    gpio_install_isr_service(0);
    gpio_isr_handler_add( BUTTON_1, gpio_isr_handler, (void*) BUTTON_1 ); 
	
    
    for(;;) 
    {	

        /**
         * Aguarda por um evento de notificação gerado a cada interrupção
         * externa em GPIO_17;
         * pdTRUE indica que após a notificação acontecer, deverá zer zerada
         * a notificação interna;
         */
        nNotify = ulTaskNotifyTake( pdTRUE, portMAX_DELAY );
	    if( nNotify > 0 )
        {
            if( DEBUG )
                ESP_LOGI( TAG, "Notify = %d, LED BUILDING = %d\n", nNotify, cnt % 2 ); 

            gpio_set_level( LED_BUILDING, cnt++ % 2 );
        }
        
		vTaskDelay( 300/portTICK_PERIOD_MS );
    }
}

/**
 * Inicio do Programa;
 */
void app_main( void )
{	

    if( xTaskCreate( Task_Led, "Task_Led", 2048, NULL, 2, &taskHandle )!= pdTRUE )
	{
		if( DEBUG )
			ESP_LOGI( TAG, "error - nao foi possivel alocar Task_Led.\n" );	
		return;		
	}
}





