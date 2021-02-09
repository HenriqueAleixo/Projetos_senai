/* --------------------------------------------------------------------------
  Autor: Prof° Fernando Simplicio;
  Hardware: kit Heltec LoRa Oled ESP32 ;
  Espressif SDK-IDF: v3.2 e v3.3
  Curso: Formação em Internet das Coisas (IoT) com ESP32
  Link: https://www.microgenios.com.br/formacao-iot-esp32/
 *  --------------------------------------------------------------------------

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

/**
 * Lib Standar C;
 */
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include <math.h>

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

#include "driver/gpio.h"
/**
 * MQTT User;
 */
#include "mqtt.h"

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
#define MASTER_NODE_ADDRESS 0

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

/**
 * Protótipos
 */
static esp_err_t wifi_event_handler( void *ctx, system_event_t *event );
static void wifi_init_sta( void );
void task_button ( void *pvParameter );
static void sendt( char * message );
SemaphoreHandle_t xSemaphore;

static int spreading_factor;
static int distancia_rx_tx = 0;

char * distance[15] = {"10", "50","100", "150","200", "250","300", "350","400", "450","500", "550","600", "650", "700"};
	
	


typedef struct _datarate {
	int sf;
	const int size;
}  DataRate_t;

static DataRate_t lorawan_dr[] = 
{
	{ 7,  235},
	{ 8,  135},
	{ 9,  66},
	{ 10,  24},
	{ 11,  64},
	{ 12,  64}
};
  
float calculate_air_time( 	unsigned int sf, 
							unsigned int bw, 
							unsigned int explicitHeader, 
							unsigned int codingRate, 
							unsigned int preambleLength, 
							unsigned int size ) 
{
	float tSym;
	float tPreamble;
	unsigned int h;
	unsigned int de;
	unsigned int cr;
	float payloadSymbNb;
	float tPayload;
	float air_time;
	//#define LOWDR_OPTIMIZE 			0 	/* 0 -> SF7 ~SF10; 1-> SF11 e SF12 */
	unsigned int lowDrOptimize;
	
	if( sf >= 7 || sf <= 10 ) lowDrOptimize = 0; else lowDrOptimize = 1;
	tSym = (pow(2, sf) / (bw * 1000) )* 1000;
	tPreamble = (preambleLength + 4.25f) * tSym;
	h = explicitHeader ? 0: 1; 
	de = ((lowDrOptimize == 1 && bw == 125 && sf >= 11 ) || lowDrOptimize == 1 ) ? 1 : 0;	
	cr = codingRate-4;
	payloadSymbNb = 8 + MAX(ceil((float)( (8 * size)-(4 * sf)+ 28 + 16 - (20 * h)) / (4 * (sf - (2 * de)))) * (cr + 4), 0);   
	tPayload = payloadSymbNb * tSym;
    air_time = (tPreamble + tPayload);
	return air_time;
}


/**
 * Task responsável pela transmissão Tx via LoRa;
 */
static void sendt( char * message )
{
   uint8_t protocol[100];
  /**
   * Protocolo;
   * <id_node_sender><id_node_receiver><command><payload_size><payload><crc>
   */
    protocol[0] = MASTER_NODE_ADDRESS; 
    protocol[1] = SLAVE_NODE_ADDRESS;        
    protocol[2] = CMD_PRINT_OLED;       
    protocol[3] = strlen(message) + 1;                   
    
    strcpy( (char*)&protocol[4], message );
    
    /**
     * Calcula o CRC do pacote;
     */
    USHORT usCRC = usLORACRC16( protocol, 4 + protocol[3] );
    protocol[4 + protocol[3]] = (UCHAR)(usCRC & 0xFF); 
    protocol[5 + protocol[3]] = (UCHAR)((usCRC >> 8) & 0xFF);

    /**
     * Transmite protocol via LoRa;
     */
    lora_send_packet( protocol, 6 + protocol[3] );
	if( DEBUG )
		ESP_LOGI( TAG, "Bytes Enviados: %d;", 6 + protocol[3] );
}

/**
 * Task responsável pela transmissão Tx via LoRa;
 */
void task_tx( void *pvParameter )
{
  
   int packet_number = 0;
   char message[255];
   unsigned int airtime;
   
   for( ;; ) 
   {
		snprintf( message, sizeof(message),"{\"p\":\"%3d\",\"d\":\"%d\"}", ++packet_number, distancia_rx_tx);
		sendt( message ); 

		if( DEBUG )
			ESP_LOGI( TAG, "%.*s", strlen(message), message );

		airtime = calculate_air_time(SPREADING_FACTOR,BANDWIDTH_KHZ,EXPLICIT_HEADER,CODING_RATE,PREAMBLE_LENGTH_BYTES,lorawan_dr[ SPREADING_FACTOR -7].size
		); 
		airtime = (unsigned int)((airtime/10.0)*1000.0);
		
		if( DEBUG )
			ESP_LOGI( TAG, "Airtime=%d ms", airtime );

		xSemaphoreTake( xSemaphore, portMAX_DELAY );
		
			sprintf( message, "%d", packet_number);
			ssd1306_out8( 5, 0, message, WHITE );

		xSemaphoreGive(xSemaphore);
		/**
		* Delay;
		*/
		vTaskDelay(  100/portTICK_PERIOD_MS  );
   }

   /**
    * Esta linha não deveria ser executada...
    */
   vTaskDelete( NULL );
}



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

/**
 * task responsável em ler o nível lógico de button
 * e por enviar o valor de count ao servidor via socket tcp;
 */
void task_button ( void *pvParameter )
{
    int aux=0;
	
	char str[20];
	char str_distance[20];

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
				if( spreading_factor > 12 )
				{ 
					spreading_factor = 7; 
					
					distancia_rx_tx++; 
					if( distancia_rx_tx >= 15 ) 
					{
						distancia_rx_tx = 0;
					}
					
					
				}
				
				sprintf( str, "%d", spreading_factor );
				sprintf( str_distance,"%d", distancia_rx_tx );
				/**
				* Envia mensagem MQTT para o node da rede LoRa;
				*/
				if( nvs_str_save( "spread_f", str ) == ESP_OK )
				{
					if( nvs_str_save( "disrx_tx", str_distance ) == ESP_OK )
					{
						if( DEBUG ) {
							ESP_LOGI(TAG, "spread_f= %s", str );
						}
						if( DEBUG ) {
							ESP_LOGI(TAG, "disrx_tx= %s", str_distance );
						}						
						
						xSemaphoreTake( xSemaphore, portMAX_DELAY );
						
							lora_set_spreading_factor( spreading_factor );
							
							ssd1306_out8( 3, 0, "SF           ", WHITE );
							ssd1306_out8( 3, 2, str, WHITE );	
							ssd1306_out8( 3, 5, distance[distancia_rx_tx], WHITE );
						
						xSemaphoreGive(xSemaphore);
						
					}  
				}       
			
                vTaskDelay( 80/portTICK_PERIOD_MS );
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
				vTaskDelay( 80/portTICK_PERIOD_MS );
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
    /**
    * Imprime usando fonte8x16;
    * Sintaxe: ssd1306_out8( linha, coluna, ftring , fonte_color );
    */
    ssd1306_out8( 0, 0, "Transmitter", WHITE );
    //ssd1306_out8( 2, 0, "Node Add:", WHITE );
    //ssd1306_chr8( 2, 9, SLAVE_NODE_ADDRESS + '0', WHITE );

    /*
      Configura a rede WiFi.
    */
//    wifi_init_sta();

    /**
     * Aguarda a conexão WiFi do ESP32 com o roteador;
     */
//    xEventGroupWaitBits( wifi_event_group, CONNECTED_BIT, false, true, portMAX_DELAY );

    /**
     * Inicializa e configura a rede;
     */
//    mqtt_start();
	
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

	ssd1306_out8( 3, 0, "SF         ", WHITE );
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

	//distancia_rx_tx
	if( nvs_read_key( "disrx_tx", buf, sizeof(buf) ) == ESP_OK ){
        if( DEBUG ) {
            ESP_LOGI(TAG, "distancia_rx_tx: %s\n", buf );
        }
		distancia_rx_tx = atoi( buf );
	} else {
		
		distancia_rx_tx = 0;
		sprintf( buf, "%d", distancia_rx_tx );
		if( nvs_str_save( "disrx_tx", buf) == ESP_OK )
		{
			if( DEBUG ) {
               ESP_LOGI(TAG, "disrx_tx (SALVO): %s\n", buf );
            }
		}
	}	

	ssd1306_out8( 3, 5, distance[distancia_rx_tx], WHITE );
						
    /**
    * Cria a task de transmissao LoRa;
    */
    if( xTaskCreate( task_tx, "task_tx", 1024*5, NULL, 5, NULL ) != pdTRUE )
    {
        if( DEBUG )
           ESP_LOGI( TAG, "error - Nao foi possivel alocar task_tx.\r\n" );  
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