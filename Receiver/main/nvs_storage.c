#include "nvs_storage.h"

static const char *TAG = "nvs: ";

esp_err_t nvs_read_key( char * keystr1, char * ssid, size_t ssid_len )
{
    nvs_handle my_handle;
	esp_err_t err = nvs_open( "storage", NVS_READWRITE, &my_handle );
	if ( err != ESP_OK ) {					
			if( DEBUG )
				ESP_LOGI(TAG, "Error (%d) opening NVS handle!\n", err );			
	} else {
		/**
		 * Leitura do SSID salvo na nvs
		 */
		err = nvs_get_str(my_handle, keystr1, ssid, &ssid_len);
		if( err != ESP_OK ) {							
			if( err == ESP_ERR_NVS_NOT_FOUND ) {
				if( DEBUG )
					ESP_LOGI(TAG, "\nKey %s not found.\n", keystr1 );
			} 							
		} else {
			if( DEBUG )
				ESP_LOGI(TAG, "\nkey is %s\n", ssid );
		}
		

		nvs_close(my_handle);		
	}

	return err;
	
}

esp_err_t nvs_str_save(char * key, char * value)
{
	nvs_handle my_handle;
	esp_err_t err = nvs_open( "storage", NVS_READWRITE, &my_handle );
	if ( err != ESP_OK ) {					
			if( DEBUG )
				ESP_LOGI(TAG, "Error (%d) opening NVS handle!\n", err );			
	} else {
	    err = nvs_set_str(my_handle, key, value );
        if(err != ESP_OK) {
            if( DEBUG )
                ESP_LOGI(TAG, "\nError in %s : (%04X)\n", key, err);			
        }
        /* Salva em nvs */
		err = nvs_commit(my_handle);
		if(err != ESP_OK) {
			if( DEBUG )
				printf("\nError in commit! (%04X)\n", err);
		}
									
		nvs_close(my_handle);		
	}

	return err;
}
