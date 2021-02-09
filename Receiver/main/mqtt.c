/* --------------------------------------------------------------------------
  Autor: Prof° Fernando Simplicio;
  Hardware: NodeMCU ESP32
  Espressif SDK-IDF: v3.2 ou v3.3
  Curso: Formação em Internet das Coisas (IoT) com ESP32
  Link: https://www.microgenios.com.br/formacao-iot-esp32/
 *  --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
   
/**
 * C library
 */
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdarg.h>

/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"
#include "freertos/semphr.h"

/**
 * MQTT 
 */
#include "mqtt.h"

/**
 * Carrega as configurações padrão;
 */
#include "sys_config.h"

/**
 * Logs;
 */
#include "esp_log.h"

/**
 * Lib MQTT
 */
#include "mqtt_client.h"

/**
 * LoRa API;
 */
#include "lora.h"

/**
 * cJSON
 */
#include "cJSON.h"

#include "nvs_storage.h"
#include "esp_wifi.h"
#include "lib_ssd1306.h"
/**
 * Variáveis
 */
static const char *TAG = "mqtt: ";
static EventGroupHandle_t mqtt_event_group;
esp_mqtt_client_handle_t client;
esp_mqtt_client_handle_t client_mqtt;
extern SemaphoreHandle_t xSemaphore;
extern unsigned int potencia_transmissao;
/**
 * Variáveis externas; 
 */
const static int CONNECTED_BIT = BIT0;
void mqtt_stop( void );


typedef struct _ret
{
    char address[30];
    char sf[30];
    char pt[30];
} Json_Data;

int processJson( char * payload, Json_Data * res )
{
    /**
     * payload = {"node":"1","sf":"7","pt":"14"}
     */
    int error = 0; 
    cJSON *json = cJSON_Parse( payload );

    if( json != NULL )
    {
        cJSON *address = cJSON_GetObjectItem( json, "node" ); 
        if( address != NULL )
        {
            if(cJSON_IsString(address) && (address->valuestring != NULL))
            {
                /**
                 * Se chegou aqui é porque o json é válido, porém é preciso verificar
                 * o campo sf do json;
                 */
                if( DEBUG )
                    ESP_LOGI( TAG, "Node: %s.\n", address->valuestring ); 
                
                cJSON *sf = cJSON_GetObjectItem( json, "sf" ); 
                if( sf != NULL )
                {
                    if( cJSON_IsString( sf ) && ( sf->valuestring != NULL ) )
                    {
                        /**
                         * Se chegou aqui é porque o json é válido. Tanto o address
                         * quanto sf são válidos, portanto podem ser resgatados;
                         */
                        if( DEBUG )
                            ESP_LOGI( TAG, "sf: %s.\n", sf->valuestring ); 

                        memcpy( res->address, address->valuestring, strlen(address->valuestring)+1 ); 
                        memcpy( res->sf, sf->valuestring, strlen(sf->valuestring)+1 );


                        /**
                         * O campo value é opcional no Json;
                         */
                        cJSON *power_transmitter = cJSON_GetObjectItem( json, "pt" ); 
                        if( power_transmitter != NULL )
                        {
                            if( cJSON_IsString( power_transmitter ) && ( power_transmitter->valuestring != NULL ) )
                            {
                                if( DEBUG )
                                    ESP_LOGI( TAG, "power_transmitter: %s.\n", power_transmitter->valuestring ); 

                                memcpy( res->pt, power_transmitter->valuestring, strlen(power_transmitter->valuestring)+1 );
                            }
                        }
                        
                    }            
                } else {
                    if( DEBUG )
                        ESP_LOGI( TAG, "error in cJSON" );
                     error = -1;        
                }
                
            }            
        } else 
        {
            if( DEBUG )
                ESP_LOGI( TAG, "error in cJSON" );
            error = -1;         
        }
           
        cJSON_Delete( json );           
    } 

    return error; 
}

/**
 * Função responsável pelo tratamento das mensagem recebidas via MQTT; 
 */
esp_err_t mqtt_read( esp_mqtt_client_handle_t client, esp_mqtt_event_handle_t mqtt_msg )
{
    Json_Data res = { {0}, {0}, {0} };
    /*
      Reserva
    */
    int total = 1*512;  
    char *data = pvPortMalloc( total ); //reserva 512 bytes do heap (RAM);
    if( data == NULL ) 
    {
        if( DEBUG )
            ESP_LOGI(TAG, "pvPortMalloc Error\r\n");
       return ESP_FAIL;
    }

    /*
       Comando "TOOGLE" enviado pelo cliente para um non-root. 
    */
    if( mqtt_msg->data_len > 0 )
    {
        snprintf( data, total, "%.*s", mqtt_msg->data_len, mqtt_msg->data  );
        if( processJson( (char*)data , &res ) == 0 )
        {
            if( DEBUG ) 
            {
               ESP_LOGI( TAG, "%s", res.address );
               ESP_LOGI( TAG, "%s", res.sf ); 
               ESP_LOGI( TAG, "%s", res.pt );
            } 
           /**
            * Envia mensagem MQTT para o node da rede LoRa;
            */
           //lora_data_send( res.address, res.sf, res.value );
			if( nvs_str_save( "spread_f", res.sf) == ESP_OK )
			{
				if( nvs_str_save( "power_t", res.pt) == ESP_OK )
				{
					if( DEBUG ) {
						ESP_LOGI(TAG, "spread_f= %s; power_t= %s\n", res.sf, res.pt );
					}
					
					
					xSemaphoreTake( xSemaphore, portMAX_DELAY );
						lora_set_spreading_factor( atoi( res.sf ) );
						lora_set_tx_power( atoi( res.pt ) );
						potencia_transmissao = atoi( res.pt );
					xSemaphoreGive(xSemaphore);
					//mqtt_stop();
					//ESP_ERROR_CHECK(esp_wifi_stop() );
                	//ESP_ERROR_CHECK(esp_wifi_deinit() );
					
                	//esp_restart(); 
					sprintf( data,"MQTT Receptor: spread_f= %s; power_t= %s\n", res.sf, res.pt );
					if( mqtt_publish_data( data, strlen(data) ) == ESP_OK )
					{
						if( DEBUG ) {
							ESP_LOGI(TAG, "%s", data );
						}
					}
				}  
			}       
           

        }

    } 

    /*
       Libera o buffer data utilizado no armazenamento dos bytes recebidos que foi criado de maneira dinâmica;
    */
    vPortFree( data ); 

    return ESP_OK; 
}

/**
 * Função de callback do stack MQTT;
 */
esp_err_t mqtt_event_handler( esp_mqtt_event_handle_t event )
{
    client = event->client;
    
    switch ( event->event_id ) 
    {
        case MQTT_EVENT_BEFORE_CONNECT: 
            if( DEBUG )
                ESP_LOGI(TAG, "MQTT_EVENT_BEFORE_CONNECT");          
            break;
          
        case MQTT_EVENT_CONNECTED:
            if( DEBUG )
                ESP_LOGI( TAG, "MQTT_EVENT_CONNECTED" );

            /**
             * Assina um único tópico no broker MQTT com QoS 0;
             */
            esp_mqtt_client_subscribe( client, TOPIC_SUBSCRIBE, 0 );

            /**
             * Sinaliza o app que estamos conectados ao broker MQTT; 
             */
            xEventGroupSetBits( mqtt_event_group, CONNECTED_BIT );
            ssd1306_out8( 1, 0, "Online  ", WHITE );
            break;

        case MQTT_EVENT_DISCONNECTED:
            if(DEBUG)
                ESP_LOGI(TAG, "MQTT_EVENT_DISCONNECTED");

            /**
             * Caso, por algum motivo, o broker MQTT venha a desconectar o cliente
             * Ex: rede offline, servidor offline, usuário não mais autenticado...
             * Sinaliza a app que não estamos mais conectados ao broker;
             */
            xEventGroupClearBits( mqtt_event_group, CONNECTED_BIT );
			ssd1306_out8( 1, 0, "Offline", WHITE );
            break;

        case MQTT_EVENT_SUBSCRIBED:   
            if( DEBUG )
                ESP_LOGI( TAG, "MQTT_EVENT_SUBSCRIBED, msg_id=%d", event->msg_id );
            break;

        case MQTT_EVENT_UNSUBSCRIBED:           
            if( DEBUG )
                ESP_LOGI( TAG, "MQTT_EVENT_UNSUBSCRIBED, msg_id=%d", event->msg_id );
            break;

        case MQTT_EVENT_PUBLISHED:            
            if( DEBUG )
                ESP_LOGI( TAG, "MQTT_EVENT_PUBLISHED, msg_id=%d", event->msg_id );
            break;

        case MQTT_EVENT_DATA:

            /**
             * Se este case for chamado significa que alguma mensagem foi recebida pelo ESP32
             * ou seja, alguma mensagem foi publicada no tópico 'TOPIC_SUBSCRIBE';
             */
            if( DEBUG )
            {
                ESP_LOGI( TAG, "MQTT_EVENT_DATA" ); 
                /**
                 * Imprime o nome do tópico no qual a mensagem foi publicada.
                 */
                ESP_LOGI( TAG, "Topico = %.*s\r\n", event->topic_len, event->topic );
                /**
                 * Imprime o conteúdo da mensagem recebida via MQTT;
                 */
                ESP_LOGI( TAG, "Data = %.*s\r\n", event->data_len, event->data );               
            }  

            /**
             * Já que a mensagem foi recebida, vamos processar os dados!
             * Os dados serão processados na folha app.c;
             */
             if( mqtt_read( client, event ) == ESP_OK ) {
                if( DEBUG )
                    ESP_LOGI( TAG, "Ok, mensagem processada com sucesso" );                 
             } else {
                if( DEBUG )
                    ESP_LOGI( TAG, "Falha no processamento da mensagem." ); 
             }

            break;

        case MQTT_EVENT_ERROR:
            if( DEBUG )
                ESP_LOGI( TAG, "MQTT_EVENT_ERROR" );
            break;
    }

    return ESP_OK;
}



esp_err_t mqtt_publish_data( char * data, int data_size )
{
    if( xEventGroupWaitBits( mqtt_event_group, CONNECTED_BIT, false, true, 0 ) == pdTRUE )
    {
        if( esp_mqtt_client_publish( client, TOPIC_PUBLISH, (char*)data , data_size, 0, 0 ) == 0 )
        {
            if( DEBUG )
                ESP_LOGI( TAG, "Mensagem publicada com sucesso!\r\n" );
            return ESP_OK;
        }  
    }
    return ESP_FAIL;
}

/**
 * Caso seja desejado encerrar o serviço MQTT use esta função;
 */
void mqtt_stop( void )
{
    if( DEBUG )
        ESP_LOGI( TAG, "mqtt_stop()" );
    
   esp_mqtt_client_stop( client_mqtt );
   esp_mqtt_client_destroy( client_mqtt );
}


/**
 * Configuração do stack MQTT; 
 */
void mqtt_start( void )
{
    if( DEBUG )
        ESP_LOGI( TAG, "chamado mqtt_start()" );
    
    mqtt_event_group = xEventGroupCreate();

    /**
     * Sem SSL/TLS
     */
    const esp_mqtt_client_config_t mqtt_cfg = {
        .uri = "mqtt://" MQTT_URL ":" MQTT_PORTA,
        .event_handle = mqtt_event_handler,
        .username = MQTT_USERNAME,
        .password = MQTT_PASSWORD,
    };

    /**
     * Carrega configuração do descritor e inicializa stack MQTT;
     */
    client_mqtt = esp_mqtt_client_init( &mqtt_cfg );
    esp_mqtt_client_start( client_mqtt );
}