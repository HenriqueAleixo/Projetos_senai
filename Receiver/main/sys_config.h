#ifndef _SYSCONFIG__H
#define _SYSCONFIG__H

/**
 * Definições das GPIOs
 */

#define LED_BUILDING         ( 25 ) 
#define GPIO_OUTPUT_PIN_SEL  ( 1ULL<<LED_BUILDING )

#define BUTTON               ( 0 )
#define GPIO_INPUT_PIN_SEL   ( 1ULL<<BUTTON )

/**
 * Configurãção da rede WiFi do USUÁRIO;
 * Informe o SSID e PASSWORD da rede WiFi de sua casa ou empresa aqui:
 */
#define WIFI_SSID       "FSimplicio_2.4G" //"AndroidAP16F1"//"Fefe"
#define WIFI_PASSWORD   "fsimpliciokzz5" //"ifat1347"//"555554qr"

/**
 * Configura~ão do MQTT;
 */
#define MQTT_URL 		"10.1.1.2" //"mqtt.geniot.io"
#define MQTT_PORTA 		"1883"
#define MQTT_USERNAME 	"admin"
#define MQTT_PASSWORD 	"admin"

#define TOPIC_SUBSCRIBE "sub_redemesh" //{"node":"1","sf":"3","pt":"14"}
#define TOPIC_PUBLISH 	"pub_redemesh"

/**
 * Configuração de Rede;
 */
#define IP_FIXO 0
#define IP_ADDRESS 		"192.168.0.80"
#define GATEWAY_ADDRESS "192.168.0.1"
#define NETMASK_ADDRESS "255.255.255.0"

/**
 * Definições Gerais;
 */
#define TRUE  1
#define FALSE 0
#define DR_REG_RNG_BASE   0x3ff75144
 

/**
 * Debug?
 */
#define DEBUG 1
 

#endif 