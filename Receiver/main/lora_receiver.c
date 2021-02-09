/* --------------------------------------------------------------------------
  Autor: Prof° Fernando Simplicio;
  Hardware: kit Heltec LoRa Oled ESP32 ;
  Espressif SDK-IDF: v3.2 e v3.3
  Curso: Formação em Internet das Coisas (IoT) com ESP32
  Link: https://www.microgenios.com.br/formacao-iot-esp32/
 *  --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distancia_rx_txributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/*
#define CMD_READ_COUNT 0
#define CMD_READ_ADC   1
#define CMD_SET_GPIO   2
#define CMD_PRINT_OLED 3
#define CMD_READ_RSSI  4

 * comandos:
 * Escreve no display Oled o valor 545 do node 1 e 
 * {"node":"1","command":"3","value":"545"}
 * retorno: {"node":"1","command":"3","ack":"ok"}
 * 
 * Realiza a leitura do RSSI e SNR do node 1;
 * {"node":"1","command":"4"}
 * retorno: {"node":"1","rssi":"-47","snr":"9.75"}
 */

/**
 * Lib Standar C;
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

/**
 * FreeRTOS
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "freertos/event_groups.h"

/**
 * Drivers
 */
#include "nvs_flash.h"

/**
 * Lib LoRa;
 * Localizado em Componentes > lora;
 */
#include "lora.h"

/**
 * Log
 */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"

/**
 * Drivers
 */
#include "driver/gpio.h"

/**
 * Lora CRC;
 */
#include "lora_crc.h"

/**
 * Lib Display SSD1306 Oled;
 */
#include "lib_heltec.h"

/**
 * Configurações de Rede;
 */
#include "sys_config.h"

/**
 * LWIP
 */
#include "lwip/sockets.h"
#include "lwip/dns.h"
#include "lwip/netdb.h"

/**
 * WiFi Callback
 */
#include "esp_event_loop.h"

/**
 * WiFi
 */
#include "esp_wifi.h"

/**
 * lib nvs
 */
#include "nvs_storage.h"

/**
 * MQTT User;
 */
#include "mqtt.h"

/**
 * cJSON
 */
#include "cJSON.h"


#define MAX(x,y) ( \
    { __auto_type __x = (x); __auto_type __y = (y); \
      __x > __y ? __x : __y; })
	  

#define LORA_FREQUENCY 			915e6
#define SPREADING_FACTOR 		7
#define BANDWIDTH_KHZ 			125
#define POWER_TRANSMITTER       20
#define CODING_RATE 			5 	/* 4/5 */
#define PREAMBLE_LENGTH_BYTES 	8 //Preamble Length = 12 symbols (programmed register PreambleLength=8)
#define EXPLICIT_HEADER 		1
/*
*   @param size full packet size. For LoRaWAN this includes the LoRaWAN MAC
*   header (about 9 bytes when no MAC commands are included), the application
*   payload, and the MIC (4 bytes).
*/
#define OVERHEAD_MAC_BYTES  	9+4  
#define PAYLOAD_LENGTH_BYTES 	10
#define SLAVE_NODE_ADDRESS 1


/**
 * Endereçamento dos dispositivos da rede LoRa;
 * O MASTER sempre será o inicializa a comunicação com os SLAVES;
 * O MASTER possui endereço 0, enquanto os SLAVES são enumerados de 1 a 1000;
 */
#define MASTER_NODE_ADDRESS 0
#define LORA_RECEIVER_TIMEOUT_MS 5000

/**
 * Comandos;
 */
#define CMD_READ_COUNT 0
#define CMD_READ_ADC   1
#define CMD_SET_GPIO   2
#define CMD_PRINT_OLED 3
#define CMD_READ_RSSI  4

/**
 * Variáveis;
 */
const char * TAG = "main ";
const static int CONNECTED_BIT = BIT0;
static EventGroupHandle_t wifi_event_group;
esp_mqtt_client_handle_t client;
static int spreading_factor;
SemaphoreHandle_t xSemaphore;
unsigned int potencia_transmissao = 0; 

char * distance[15] = {"10", "50","100", "150","200", "250","300", "350","400", "450","500", "550","600", "650", "700"};
	
/**
 * Protótipos
 */
static esp_err_t wifi_event_handler( void *ctx, system_event_t *event );
static void wifi_init_sta( void );
void task_button ( void *pvParameter );

/**
 * Função de callback do WiFi;
 */
static esp_err_t wifi_event_handler( void *ctx, system_event_t *event )
{
    switch( event->event_id ) 
    {
        case SYSTEM_EVENT_STA_START:
            esp_wifi_connect();
            break;

        case SYSTEM_EVENT_STA_GOT_IP:
            xEventGroupSetBits( wifi_event_group, CONNECTED_BIT );
            break;

        case SYSTEM_EVENT_STA_DISCONNECTED:
            esp_wifi_connect();
            xEventGroupClearBits( wifi_event_group, CONNECTED_BIT );
            break;

        default:
            break;
    }
    return ESP_OK;
}

/**
 * Inicializa a rede WiFi em modo Station;
 */
static void wifi_init_sta( void )
{
    tcpip_adapter_init();

#if IP_FIXO
    /**
     * O ESP32 ROOT da rede Mesh é aquele que receber o endereço IP do Roteador; 
     * Deseja trabalhar com o endereço IP fixo na rede? Ou seja, deseja configurar
     * o ROOT com IP Statico? 
     */
    ESP_ERROR_CHECK(tcpip_adapter_dhcpc_stop(TCPIP_ADAPTER_IF_STA));
    tcpip_adapter_ip_info_t sta_ip;
    sta_ip.ip.addr = ipaddr_addr( IP_ADDRESS );
    sta_ip.gw.addr = ipaddr_addr( GATEWAY_ADDRESS );
    sta_ip.netmask.addr = ipaddr_addr( NETMASK_ADDRESS );
    tcpip_adapter_set_ip_info(WIFI_IF_STA, &sta_ip);
#endif

    ESP_ERROR_CHECK( esp_event_loop_init( wifi_event_handler, NULL ) );

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = WIFI_SSID,
            .password = WIFI_PASSWORD,
        },
    };
    ESP_ERROR_CHECK( esp_wifi_set_mode( WIFI_MODE_STA ) );
    ESP_ERROR_CHECK( esp_wifi_set_config( ESP_IF_WIFI_STA, &wifi_config ) );
    ESP_ERROR_CHECK( esp_wifi_start() );

    if( DEBUG )
    {
      ESP_LOGI(TAG, "start the WIFI SSID:[%s] password:[%s]", WIFI_SSID, "******");
      ESP_LOGI(TAG, "Waiting for wifi");    
    }

}

static int processJson( char * payload, char * packet_number, char * distance );

/**
 * Função responsável pela conversão do pacote MQTT JSON;
 */
static int processJson( char * payload, char * packet_number, char * distance )
{
    /**
     * payload = {"node":"1","command":"4"}
     */
    int error = 0; 
    cJSON *json = cJSON_Parse( payload );

    if( json != NULL )
    {
        cJSON *address = cJSON_GetObjectItem( json, "p" ); 
        if( address != NULL )
        {
            if(cJSON_IsString(address) && (address->valuestring != NULL))
            {
                /**
                 * Se chegou aqui é porque o json é válido, porém é preciso verificar
                 * o campo command do json;
                 */
                //if( DEBUG )
                    //ESP_LOGI( TAG, "p: %s.\n", address->valuestring ); 
                
                cJSON *command = cJSON_GetObjectItem( json, "d" ); 
                if( command != NULL )
                {
                    if( cJSON_IsString( command ) && ( command->valuestring != NULL ) )
                    {
                        /**
                         * Se chegou aqui é porque o json é válido. Tanto o address
                         * quanto command são válidos, portanto podem ser resgatados;
                         */
                        //if( DEBUG )
                            //ESP_LOGI( TAG, "d: %s.\n", command->valuestring ); 

                        memcpy( packet_number, address->valuestring, strlen(address->valuestring)+1 ); 
                        memcpy( distance, command->valuestring, strlen(command->valuestring)+1 );


                       
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
 * Task responsável pela recepção LoRa;
 */
static void task_rx( void *pvParameter )
{
   int x = 0;
   char buf[150];
   char str[20];
   uint8_t protocol[255];
   int cnt_1 = 0;
   int rssi = -1;
   float snr = -1;
   unsigned int n_pk_mqtt = 0;
   int num;
   int distancia_rx_tx = -1;
   int flag = 0;
   
   char packet_number[20];
   char distancia[20];
   
   for( ;; ) 
   {
	   
		xQueueReceive( xQueue_LoRa, &cnt_1, portMAX_DELAY );

		/**
		* Algum byte foi recebido?
		* Realiza a leitura dos registradores de status do LoRa com o 
		* objetivo de verificar se algum byte recebido foi armazenado
		* na FIFO do rádio;
		*/
		while( lora_received() ) 
		{
			/**
			* Sim, existe bytes na FIFO do rádio LoRa, portanto precisamos ler
			* esses bytes; A variável buf armazenará os bytes recebidos pelo LoRa;
			* x -> armazena a quantidade de bytes que foram populados em buf;
			*/
			x = lora_receive_packet( protocol, sizeof(protocol) );

			/**
			* Protocolo;
			* <id_node_sender><id_node_receiver><command><payload_size><payload><crc>
			*/
			if( x >= 6 && protocol[0] == MASTER_NODE_ADDRESS && protocol[1] == SLAVE_NODE_ADDRESS )
			{
				/**
				* Verifica CRC;
				*/
				USHORT usCRC = usLORACRC16( protocol, 4 + protocol[3] );
				UCHAR ucLow =  (UCHAR)(usCRC & 0xFF);
				UCHAR ucHigh = (UCHAR)((usCRC >> 8) & 0xFF);
				
				if( ucLow == protocol[4 + protocol[3]] && ucHigh == protocol[5 + protocol[3]] )
				{
				  //if( DEBUG )
					  //ESP_LOGI( TAG, "CRC OK!" );

				  /**
				   * Verifica qual o comando recebido;
				   */
				  switch( protocol[2] )
				  {
					  /**
					   * Comando para impressão no display Oled;
					   */
					  case CMD_PRINT_OLED:
							
							//(char*)&protocol[4];
							if( processJson( (char*)&protocol[4] , packet_number, distancia  ) == 0 )
							{
								ESP_LOGI( TAG, "packet_number: %s; distance: %s", packet_number, distancia );
								flag = 1;
							}

							break;
				  }
				}

			}
			  
	
			
			/*
			qtd_bytes_recebidos = lora_receive_packet( protocol, sizeof(protocol) );
			
			if( qtd_bytes_recebidos > 2 )
			{
				if(protocol[0] == '[')
				{
					for(int i = 1; i < 20; ++i)
					{
						if(protocol[i] == ']')
						{
							protocol[i] = 0; 
							num = atoi( (char*)&protocol[1] );
							ESP_LOGI( TAG, "NUM:%s", &protocol[1] );
							flag = 1;
							
							i++;
							if(protocol[i] == '[')
							{
								for(int z = 1; z < 5; ++z)
								{
									if(protocol[i+z] == ']')
									{
										protocol[i+z] = 0;
										distancia_rx_tx = atoi( (char*)&protocol[i+1] );
									}
								}
							}
									
							break;
						}
					}
				}
			}

            */
			num = atoi( packet_number );
			distancia_rx_tx = atoi( distancia );
			
			rssi = lora_packet_rssi();
			snprintf( buf, sizeof(buf), "RSSI:%d dbm", rssi );
			//ssd1306_out8( 3, 0, buf, WHITE );

			snr = lora_packet_snr();
			snprintf( buf, sizeof(buf), "SNR:%.2f", snr );
			//ssd1306_out8( 4, 0, buf, WHITE );

			if( DEBUG )
				ESP_LOGI( TAG, "RSSI:%d dbm SNR:%.2f", rssi, snr );

			n_pk_mqtt++;
			
			if( flag == 1 )
			{
				snprintf( buf, sizeof(buf), "{\"node\":\"%d\",\"n_pk_mqtt\":\"%d\",\"n_pk_rcv\":\"%d\",\"rssi\":\"%d\",\"snr\":\"%2f\",\"sf\":\"%d\",\"m\":\"%s\",\"pt\":\"%d\"}", SLAVE_NODE_ADDRESS, n_pk_mqtt, num, rssi, snr, spreading_factor,  distance[distancia_rx_tx], potencia_transmissao );
				flag = 0;
				
				xSemaphoreTake( xSemaphore, portMAX_DELAY );
					sprintf( str, "%d         ", num);
					ssd1306_out8( 5, 0, str, WHITE );
					
					sprintf( str, "%d         ", num);
					ssd1306_out8( 3, 5, distance[distancia_rx_tx], WHITE );
				xSemaphoreGive(xSemaphore);
	  
			} else
			{
				snprintf( buf, sizeof(buf), "{\"node\":\"%d\",\"n_pk_mqtt\":\"%d\",\"n_pk_rcv\":\"error\",\"rssi\":\"%d\",\"snr\":\"%2f\",\"sf\":\"%d\",\"m\":\"%s\",\"pt\":\"%d\"}", SLAVE_NODE_ADDRESS, n_pk_mqtt, rssi, snr, spreading_factor,  distance[distancia_rx_tx], potencia_transmissao );				
			}

			if( mqtt_publish_data( buf, strlen(buf) ) == ESP_OK )
			{
			    if( DEBUG )
				    ESP_LOGI( TAG, "mqtt published %s", buf );
			}
						  
		}

		/**
		* Delay entre cada leitura dos registradores de status do LoRa;
		*/
		vTaskDelay( 10/portTICK_PERIOD_MS  );
   }
}

/**
 * task responsável em ler o nível lógico de button
 * e por enviar o valor de count ao servidor via socket tcp;
 */
void task_button ( void *pvParameter )
{
    int aux=0;
	
	char str[20];

	if( DEBUG )
		ESP_LOGI(TAG, "task_button run...\r\n");

    /**
     * GPIO (Button) configurada como entrada;
     */
	gpio_pad_select_gpio( BUTTON );	
    gpio_set_direction( BUTTON, GPIO_MODE_INPUT );
	gpio_set_pull_mode( BUTTON, GPIO_PULLUP_ONLY ); 
    
    for(;;) 
	{
	
		/**
		 * Botão pressionado?
		 */
		if( gpio_get_level( BUTTON ) == 0 && aux == 0 )
		{ 
			/**
			* Aguarda 80ms devido ao bounce;
			*/
			vTaskDelay( 80/portTICK_PERIOD_MS );	

			if( gpio_get_level( BUTTON ) == 0 && aux == 0 ) 
			{
				spreading_factor++;
				if( spreading_factor > 12 ) spreading_factor = 7;
				
				sprintf( str, "%d", spreading_factor );
				/**
				* Envia mensagem MQTT para o node da rede LoRa;
				*/
				if( nvs_str_save( "spread_f", str ) == ESP_OK )
				{
					{
						if( DEBUG ) {
							ESP_LOGI(TAG, "spread_f= %s", str );
						}
						xSemaphoreTake( xSemaphore, portMAX_DELAY );
							lora_set_spreading_factor( spreading_factor );
							
							ssd1306_out8( 3, 0, "SF        ", WHITE );
							ssd1306_out8( 3, 2, str, WHITE );
						xSemaphoreGive(xSemaphore);
						
					}  
				}       
			

				aux = 1; 
			}
		}
		/**
		 * Botão solto?
		 */
		if( gpio_get_level( BUTTON ) == 1 && aux == 1 )
		{   
		    /**
			* Aguarda 80ms devido ao bounce;
			*/
		    vTaskDelay( 80/portTICK_PERIOD_MS );	

			if( gpio_get_level( BUTTON ) == 1 && aux == 1 )
			{
				aux = 0;
			}
		}	

		/**
		 * Em algum momento é preciso dar oportunidade para as demais task de menor prioridade
		 * do programa serem executadas. 
		 */
		vTaskDelay( 10/portTICK_PERIOD_MS );	
    }
}



/**
 * Inicio do app;
 */
void app_main( void )
{
	char buf[50];
    /*
      Inicialização da memória não volátil para armazenamento de dados (Non-volatile storage (NVS)).
      **Necessário para realização da calibração do PHY. 
    */
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
      ESP_ERROR_CHECK(nvs_flash_erase());
      ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

	xSemaphore = xSemaphoreCreateBinary();
	xSemaphoreGive(xSemaphore);
	
    /*
       Event Group do FreeRTOS. 
       Só podemos enviar ou ler alguma informação TCP quando a rede WiFi estiver configurada, ou seja, 
       somente após o aceite de conexão e a liberação do IP pelo roteador da rede.
    */
    wifi_event_group = xEventGroupCreate();

   /**
    * Inicializa display Oled 128x64 SSD1306;
    * As configurações de pinagens do Oled são encontradas
    * em "lib_heltec.h";
    */
    ssd1306_start();
    ssd1306_out8( 0, 0, "Receiver", WHITE );


    /*
      Configura a rede WiFi.
    */
    wifi_init_sta();

    /**
     * Aguarda a conexão WiFi do ESP32 com o roteador;
     */
    xEventGroupWaitBits( wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY );

    /**
     * Inicializa e configura a rede;
     */
    mqtt_start();


    /**
    * Inicializa LoRa utilizando as configurações
    * definidas via menuconfig -> componentes -> lora
    */
    lora_init();

    /**
    * A frequência licenciada no Brasil é a 915Mhz; 
    * Verifique a frequência do seu dispositivo LoRa; 
    * conforme: 433E6 para Asia; 866E6 para Europa e 915E6 para EUA;
    */
    lora_set_frequency( LORA_FREQUENCY );
	
    //spreading factor
	if( nvs_read_key( "spread_f", buf, sizeof(buf) ) == ESP_OK ){
        if( DEBUG ) {
            ESP_LOGI(TAG, "spread_f: %s\n", buf );
        }
		lora_set_spreading_factor( atoi( buf ) );
		spreading_factor = atoi( buf );
	} else {
		lora_set_spreading_factor( SPREADING_FACTOR );
		spreading_factor = SPREADING_FACTOR;
		sprintf( buf, "%d", SPREADING_FACTOR );
		if( nvs_str_save( "spread_f", buf) == ESP_OK )
		{
			if( DEBUG ) {
               ESP_LOGI(TAG, "spread_f (SALVO): %s\n", buf );
            }
		}
	}

	ssd1306_out8( 3, 0, "SF        ", WHITE );
	ssd1306_out8( 3, 2, buf, WHITE );
	
    lora_set_bandwidth( BANDWIDTH_KHZ  * 1000 );
	lora_set_coding_rate( CODING_RATE );
	lora_set_preamble_length( PREAMBLE_LENGTH_BYTES );
	
	//power transmitter
	if( nvs_read_key( "power_t", buf, sizeof(buf) ) == ESP_OK ){
        if( DEBUG ) {
            ESP_LOGI(TAG, "power_t: %s\n", buf );
        }
		lora_set_tx_power( atoi( buf ) );
		potencia_transmissao = atoi( buf );
	} else {
		
		lora_set_tx_power( POWER_TRANSMITTER );
		sprintf( buf, "%d", POWER_TRANSMITTER );
		if( nvs_str_save( "power_t", buf) == ESP_OK )
		{
			if( DEBUG ) {
               ESP_LOGI(TAG, "power_t (SALVO): %s\n", buf );
            }
		}
	}	
	
	lora_set_sync_word(0x34);

    /**
    * Deseja habilitar o CRC no Payload da mensagem?
    */
    lora_enable_crc();

    /**
    * Habilita a recepção LoRa via Interrupção Externa;
    */
    lora_enable_irq();

    /**
    * Cria a task de recepção LoRa;
    */
    if( xTaskCreate( task_rx, "task_rx", 1024*5, NULL, 5, NULL ) != pdTRUE )
    {
        if( DEBUG )
           ESP_LOGI( TAG, "error - Nao foi possivel alocar task_rx.\r\n" );  
        return;   
    }
		/*
	   Task responsável em ler e enviar valores via Socket TCP Client. 
	*/
	if( xTaskCreate( task_button, "task_button", 4098, NULL, 5, NULL ) != pdTRUE )
	{
		if( DEBUG )
			ESP_LOGI( TAG, "error - nao foi possivel alocar task_button.\n" );	
		return;		
	}
}