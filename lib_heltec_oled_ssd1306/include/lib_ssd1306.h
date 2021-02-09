#ifndef __ssd1306_H__
#define __ssd1306_H__

/**
 * @defgroup   LIB_SSD1306 library ssd 1306
 *
 * @brief      This file implements library ssd 1306.
 *
 * @author     Fernando Simplicio
 * @date       2020
 * www.microgenios.com.br
 * 
 * Biblioteca Adaptada por Fernando Simplicio para o ESP32 com SDK-IDF 4.0; 
 * para o display SSD1306 compatíveis com Kit ESP32 LoRa Oled Heltec;
 * 
 * Código baseado nos projetos:
 * https://github.com/jfreax/jal/blob/master/src/devices/oled/ssd1306.c
 * https://github.com/pakozm/TinyGames/blob/master/Games/libraries/ssd1306/ssd1306.cpp
 * http://microcontrolandos.blogspot.com/2014/12/pic-ssd1306.html
 * 
 * https://bitbucket.org/tinusaur/ssd1306xled/src/d467de1d216d/ssd1306xled/?at=default
 * https://bitbucket.org/tinusaur/ssd1306xled/src/d467de1d216d/ssd1306xled/ssd1306xled.c?at=default
 */
 
/**
 * Protótipos de função;
 */
void ssd1306_config( uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, uint8_t i2c_port_number, uint8_t reset_pin );
void ssd1306_init( void );
void ssd1306_cmd( uint8_t wData );
void ssd1306_data( uint8_t wData );
void ssd1306_chr( char character, uint8_t color );
void ssd1306_chr8( uint8_t row, uint8_t col, char wChar, uint8_t color );
void ssd1306_chr16( uint8_t row, uint8_t col, char wChar, uint8_t color );
void ssd1306_out8( uint8_t row, uint8_t col, char *texto, uint8_t color );
void ssd1306_out16( uint8_t row, uint8_t col, char *texto, uint8_t color );
void ssd1306_set_cursor( uint8_t row, uint8_t col );
void ssd1306_set_contrast( uint8_t value );
void ssd1306_reset( uint8_t reset_pin );
void ssd1306_screen_vertically( void );
void ssd1306_image( const char *bmp, uint8_t color );
void ssd1306_fill( uint8_t color );

#define ssd1306_clear() ssd1306_fill( 0 )

#define WHITE 1
#define BLACK 0


#endif