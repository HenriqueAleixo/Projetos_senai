#ifndef NVS_STORAGE__
#define NVS_STORAGE__

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
 * Log
 */
#include "esp_system.h"
#include "esp_err.h"
#include "esp_log.h"


/**
 * Configurações de Rede;
 */
#include "sys_config.h"


esp_err_t nvs_read_key( char * keystr1, char * ssid, size_t ssid_len );
esp_err_t nvs_str_save( char * key, char * value );

#endif