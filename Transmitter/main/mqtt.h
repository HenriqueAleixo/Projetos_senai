#ifndef __MQTT_H_
#define __MQTT_H_

/**
 * Esp Api;
 */
#include "esp_system.h"

/**
 * Lib MQTT
 */
#include "mqtt_client.h"

/**
 * Rede mesh;
 */
#include "lora.h"
 
/**
 * Protótipos; 
 */
void mqtt_start( void );
void mqtt_stop( void );
esp_err_t mqtt_event_handler( esp_mqtt_event_handle_t event );
esp_err_t mqtt_read( esp_mqtt_client_handle_t client, esp_mqtt_event_handle_t mqtt_msg );
esp_err_t mqtt_publish_data( char * data, int data_size );

#endif 