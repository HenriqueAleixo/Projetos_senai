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
 * Lib Standar C;
 */
#include <stdio.h>
#include <string.h>

/**
 * FreeRTOs;
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/**
 * Drivers ESP32;
 */
#include "driver/gpio.h"
#include "driver/i2c.h"

/**
 * Logs;
 */
#include "esp_err.h"
#include "esp_log.h"

/**
 * Oled protótipos;
 */
#include "lib_ssd1306.h"

/**
 * Inclui fonts;
 */
#include "lib_ssd1306_fonts.h"

/**
 * Debugger?
 */
#define DEBUG 0


/**
 * Definições;
 */
#define ACK_CHECK_EN 0x1

/* Fundamental Command */
#define SSD1306_SETCONTRAST 0x81
#define SSD1306_ENTIREDISPLAYON_RESUME 0xA4
#define SSD1306_ENTIREDISPLAYON 0xA5
#define SSD1306_NORMALDISPLAY 0xA6
#define SSD1306_INVERTDISPLAY 0xA7
#define SSD1306_DISPLAYOFF 0xAE
#define SSD1306_DISPLAYON 0xAF

/* Scrolling Command */
#define SSD1306_RIGHT_HORIZONTAL_SCROLL 0x26
#define SSD1306_LEFT_HORIZONTAL_SCROLL 0x27
#define SSD1306_VERTICAL_AND_RIGHT_HORIZONTAL_SCROLL 0x29
#define SSD1306_VERTICAL_AND_LEFT_HORIZONTAL_SCROLL 0x2A
#define SSD1306_DEACTIVATE_SCROLL 0x2E
#define SSD1306_ACTIVATE_SCROLL 0x2F
#define SSD1306_SET_VERTICAL_SCROLL_AREA 0xA3

/* Addressing Setting Command */
#define SSD1306_SETLOWCOLUMN 0x00
#define SSD1306_SETHIGHCOLUMN 0x10
#define SSD1306_MEMORYMODE 0x20
#define SSD1306_COLUMNADDR 0x21
#define SSD1306_PAGEADDR   0x22
#define SSD1306_PAGESTARTADDR 0xB0

/* Hardware Configuration (Panel resolution and layout related) Command */
#define SSD1306_SETSTARTLINE 0x40
#define SSD1306_SEGREMAP 0xA0
#define SSD1306_SETMULTIPLEXRATIO 0xA8
#define SSD1306_COMSCANINC 0xC0
#define SSD1306_COMSCANDEC 0xC8
#define SSD1306_SETDISPLAYOFFSET 0xD3
#define SSD1306_SETCOMPINS 0xDA

/* Timing and Driving Scheme Setting Command  */
#define SSD1306_SETDISPLAYCLOCKDIV 0xD5
#define SSD1306_SETPRECHARGE 0xD9
#define SSD1306_SETVCOMDETECT 0xDB
#define SSD1306_NOP 0xE3

#define SSD1306_CHARGEPUMP 0x8D
#define SSD1306_EXTERNALVCC 0x1
#define SSD1306_SWITCHCAPVCC 0x2

#define SSD1306_CL_WHITE 0
#define SSD1306_CL_BLACK 1

// Control byte
#define OLED_CONTROL_BYTE_CMD_SINGLE    0x80
#define OLED_CONTROL_BYTE_CMD_STREAM    0x00
#define OLED_CONTROL_BYTE_DATA_STREAM   0x40

/**
 * Protótipos;
 */
static void i2c_master_init( uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, int8_t i2c_port_number );

/**
 * Variáveis Globais;
 */
static uint8_t i2c_number = 0; 
static const char * TAG = "OLED: ";
/**
 * Fonts;
 */


/**
 * Função responsável pelos envios dos COMANDOS
 * ao display Oled;
 */
void ssd1306_cmd( uint8_t wData )
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start( cmd );
    // SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
    i2c_master_write_byte( cmd, 0x78, ACK_CHECK_EN );
    i2c_master_write_byte( cmd, 0X00, ACK_CHECK_EN );
    i2c_master_write_byte( cmd, (uint8_t)wData, ACK_CHECK_EN );
    i2c_master_stop( cmd );
    ret = i2c_master_cmd_begin( i2c_number, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete( cmd );

    /**
     * Caso você tenha problemas em utilizar esta biblioteca
     * habilite o modo de debug para verificar se realmente os 
     * pacotes estão sendo gravados no display Oled via I2C;
     */
    if( DEBUG )
    {
        if( ret == ESP_OK ) {
               ESP_LOGI( TAG, "Write OK" );
        } else 
        if( ret == ESP_ERR_TIMEOUT ) {
               ESP_LOGW( TAG, "Bus is busy" );
        } else {
               ESP_LOGW( TAG, "Write Failed" );
        }
    }
}

/**
 * Função responsável pelos envios dos DADOS
 * ao display Oled;
 */
void ssd1306_data( uint8_t wData )
{
    esp_err_t ret;
    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    i2c_master_start( cmd );
    // SLA (0x3C) + WRITE_MODE (0x00) =  0x78 (0b01111000)
    i2c_master_write_byte( cmd, 0x78, ACK_CHECK_EN );
    i2c_master_write_byte( cmd, 0b1100000, ACK_CHECK_EN );
    i2c_master_write_byte( cmd, (uint8_t)wData, ACK_CHECK_EN );
    i2c_master_stop( cmd );
    ret = i2c_master_cmd_begin( i2c_number, cmd, 1000 / portTICK_RATE_MS );
    i2c_cmd_link_delete( cmd );
    
    /**
     * Caso você tenha problemas em utilizar esta biblioteca
     * habilite o modo de debug para verificar se realmente os 
     * pacotes estão sendo gravados no display Oled via I2C;
     */
    if( DEBUG )
    {
        if( ret == ESP_OK ) {
               ESP_LOGI( TAG, "Write OK" );
        } else 
        if( ret == ESP_ERR_TIMEOUT ) {
               ESP_LOGW( TAG, "Bus is busy" );
        } else {
               ESP_LOGW( TAG, "Write Failed" );
        }
    }
}

/**
 * Função de inicialização do display Oled; 
 * 
void ssd1306_init(void)
{
    DDRB |= (1 << SSD1306_SDA); // Set port as output
    DDRB |= (1 << SSD1306_SCL); // Set port as output

    ssd1306_send_command(0xAE); // display off
    ssd1306_send_command(0x00); // Set Memory Addressing Mode
    ssd1306_send_command(0x10); // 00,Horizontal Addressing Mode;01,Vertical Addressing Mode;10,Page Addressing Mode (RESET);11,Invalid
    ssd1306_send_command(0x40); // Set Page Start Address for Page Addressing Mode,0-7
    ssd1306_send_command(0x81); // Set COM Output Scan Direction
    ssd1306_send_command(0xCF); // ---set low column address
    ssd1306_send_command(0xA1); // ---set high column address
    ssd1306_send_command(0xC8); // --set start line address
    ssd1306_send_command(0xA6); // --set contrast control register
    ssd1306_send_command(0xA8);
    ssd1306_send_command(0x3F); // --set segment re-map 0 to 127
    ssd1306_send_command(0xD3); // --set normal display
    ssd1306_send_command(0x00); // --set multiplex ratio(1 to 64)
    ssd1306_send_command(0xD5); // 
    ssd1306_send_command(0x80); // 0xa4,Output follows RAM content;0xa5,Output ignores RAM content
    ssd1306_send_command(0xD9); // -set display offset
    ssd1306_send_command(0xF1); // -not offset
    ssd1306_send_command(0xDA); // --set display clock divide ratio/oscillator frequency
    ssd1306_send_command(0x12); // --set divide ratio
    ssd1306_send_command(0xDB); // --set pre-charge period
    ssd1306_send_command(0x40); // 
    ssd1306_send_command(0x20); // --set com pins hardware configuration
    ssd1306_send_command(0x02);
    ssd1306_send_command(0x8D); // --set vcomh
    ssd1306_send_command(0x14); // 0x20,0.77xVcc
    ssd1306_send_command(0xA4); // --set DC-DC enable
    ssd1306_send_command(0xA6); // 
    ssd1306_send_command(0xAF); // --turn on oled panel 
}
 */
void ssd1306_init( void )
{
    ssd1306_cmd( SSD1306_DISPLAYOFF );

    ssd1306_cmd( SSD1306_SETDISPLAYCLOCKDIV ); 
    ssd1306_cmd( 0xF0 );

    ssd1306_cmd( SSD1306_SETMULTIPLEXRATIO );
    ssd1306_cmd( 0x3F );  

    ssd1306_cmd( SSD1306_SETDISPLAYOFFSET );
    ssd1306_cmd( 0x00 );

    ssd1306_cmd( SSD1306_SETSTARTLINE );

    ssd1306_cmd( SSD1306_CHARGEPUMP );
    ssd1306_cmd( 0x14 );

    ssd1306_cmd( SSD1306_MEMORYMODE );
    ssd1306_cmd( 0 ); 

    ssd1306_cmd( SSD1306_SEGREMAP  );
    ssd1306_cmd( SSD1306_COMSCANINC );
    ssd1306_cmd( SSD1306_SETCOMPINS );
    ssd1306_cmd( 0x12 );

    ssd1306_cmd( SSD1306_SETCONTRAST );
    ssd1306_cmd( 0XC7 ); 

    ssd1306_cmd( SSD1306_SETPRECHARGE );
    ssd1306_cmd( 0xF1 );

    ssd1306_cmd( SSD1306_SETVCOMDETECT );
    ssd1306_cmd( 0x40 );

    ssd1306_cmd( SSD1306_ENTIREDISPLAYON_RESUME );

    ssd1306_cmd( SSD1306_NORMALDISPLAY );
    ssd1306_cmd( 0x2E );

    ssd1306_cmd( SSD1306_PAGEADDR );
    ssd1306_cmd( 0 );
    ssd1306_cmd( 7 );

    ssd1306_cmd( SSD1306_DISPLAYON );
}

/* Define a posição do cursor */
void ssd1306_set_cursor( uint8_t row, uint8_t col )
{
    ssd1306_cmd( SSD1306_PAGESTARTADDR | row ); //Define a posicao da linha
    ssd1306_cmd( ( SSD1306_SETHIGHCOLUMN | ( col >> 4 ) ) ); //Define o nibble MSB da posição da coluna
    ssd1306_cmd( ( 0x0f & col ) ); //Difine o nibble LSB da posição da coluna
}

/* Define o constraste do display */
void ssd1306_set_contrast( uint8_t value )
{
    ssd1306_cmd( SSD1306_SETCONTRAST ); //Envia o comando para alterar o contraste
    ssd1306_cmd( value ); //Define o novo valor do constraste
}

/* Escreve um caracter 8x8 na posição atual do cursor */
void ssd1306_chr( char wChar, uint8_t color )
{
    uint8_t i;
    wChar = wChar - 0x20;
    color = color ? 0x00 : 0xFF;
    for( i = 0; i < 8; i++ )
        ssd1306_data( ssd1306_font8x8[(uint8_t)wChar][i] ^ color );
}

/* Escreve um caracter 8x8 na posição especificada */
void ssd1306_chr8( uint8_t row, uint8_t col, char wChar, uint8_t color )
{
    ssd1306_set_cursor( row, col << 3 );
    ssd1306_chr( wChar, color );
}

/* Escreve um caracter 8x16 na posição especificada */
void ssd1306_chr16( uint8_t row, uint8_t col, char wChar, uint8_t color )
{
    uint8_t i;
    col <<= 3;
    ssd1306_set_cursor( row++, col );
    wChar = wChar - 0x20;
    color = color ? 0x00 : 0xFF;
    for( i = 0; i < 8; i++ )
        ssd1306_data( ssd1306_font8x16[(uint8_t)wChar][i] ^ color );
    ssd1306_set_cursor( row, col ); 
    for( i = 8; i < 16; i++ )
        ssd1306_data( ssd1306_font8x16[(uint8_t)wChar][i] ^ color );
}

/* Escreve um texto com caracteres 8x8 na posição especificada */
void ssd1306_out8( uint8_t row, uint8_t col, char *texto, uint8_t color )
{
    ssd1306_set_cursor( row, col << 3 );
    while( *texto )
        ssd1306_chr( *texto++, color );
}

/* Escreve um texto com caracteres 8x16 na posição especificada */
void ssd1306_out16( uint8_t row, uint8_t col, char *texto, uint8_t color )
{
    while( *texto )
        ssd1306_chr16( row, col++, *texto++, color );
}

/* Desenha uma imagem */
void ssd1306_image( const char *bmp, uint8_t color )
{
    i2c_cmd_handle_t cmd;
    uint8_t x, y;
    color = color ? 0x00 : 0xFF;
    for( y = 0; y < 8; y++ )
    {
        ssd1306_set_cursor( y, 0 );
        cmd = i2c_cmd_link_create();
        i2c_master_start( cmd );
        i2c_master_write_byte( cmd, 0x78, ACK_CHECK_EN );
        i2c_master_write_byte( cmd, 0b1100000, ACK_CHECK_EN );
        for( x = 0; x < 128; x++ )
            i2c_master_write_byte( cmd, (uint8_t)*bmp++ ^ color, ACK_CHECK_EN );
        i2c_master_stop( cmd);
        i2c_master_cmd_begin( i2c_number, cmd, 1000 / portTICK_RATE_MS );
        i2c_cmd_link_delete( cmd );
    } 
}

/* Preeche o display com o byte */
void ssd1306_fill( uint8_t color )
{
    i2c_cmd_handle_t cmd;
    uint8_t x, y;
    for( y = 0; y < 8; y++ )
    {
        ssd1306_set_cursor( y, 0 );
        cmd = i2c_cmd_link_create();
        i2c_master_start( cmd );
        i2c_master_write_byte( cmd, 0x78, ACK_CHECK_EN );
        i2c_master_write_byte( cmd, 0b1100000, ACK_CHECK_EN );
        for( x = 0; x < 128; x++ )
           i2c_master_write_byte( cmd, (uint8_t)color, ACK_CHECK_EN );
        i2c_master_stop( cmd );
        i2c_master_cmd_begin( i2c_number, cmd, 1000 / portTICK_RATE_MS );
        i2c_cmd_link_delete( cmd );
    }
}


static void i2c_master_init( uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, int8_t i2c_port_number )
{
    /**
     * Portas I2C suportada são a 0, 1 e 2
     */
    if( i2c_port_number < 0) i2c_port_number = 0; 
      else
        if( i2c_port_number > 2 ) i2c_port_number = 2; 

    i2c_number = i2c_port_number; 

    i2c_config_t i2c_config = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = i2c_sda_pin,
        .scl_io_num = i2c_scl_pin,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = 700000
    };
    i2c_param_config( i2c_number, &i2c_config );
    i2c_driver_install( i2c_number, I2C_MODE_MASTER, 0, 0, 0 );
}

void ssd1306_reset( uint8_t reset_pin )
{
    /**
     * Configura o pino de reset do display Oled;
     */
    gpio_pad_select_gpio( reset_pin ); 
    gpio_set_direction( reset_pin, GPIO_MODE_OUTPUT );

    /**
     * Reseta o display Oled;
     */
    gpio_set_level( reset_pin , 0 ); 
    vTaskDelay( 50 / portTICK_RATE_MS );
    gpio_set_level( reset_pin , 1 ); 
    vTaskDelay( 50 / portTICK_RATE_MS );
}

void ssd1306_screen_vertically( void ) 
{
    ssd1306_cmd( SSD1306_SEGREMAP | 0x01 );
    ssd1306_cmd( SSD1306_COMSCANDEC );
}

void ssd1306_config( uint8_t i2c_sda_pin, uint8_t i2c_scl_pin, uint8_t i2c_port_number, uint8_t reset_pin )
{   
    /**
     * Configura o barramento I2C;
     */
    i2c_master_init( i2c_sda_pin, i2c_scl_pin, i2c_port_number );

    /**
     * Reseta o display Oled antes de inicializá-lo;
     */
    ssd1306_reset( reset_pin );

    /**
     * Inicializa o display Oled;
     */
    ssd1306_init();

    /**
     * Display na vertical;
     */
    ssd1306_screen_vertically();

    /**
     * Limpa o display;
     */
    ssd1306_fill( 0 );
}
