/**
 * @defgroup   LoRa
 *
 * @brief      Biblioteca LoRa para ESP32 baseada no SX127x;
 *
 * @author     Fernando Simplicio
 * @date       2020
 * www.microgenios.com.br
 * Curso Online: https://www.microgenios.com.br/formacao-iot-esp32/
 * Biblioteca Adaptada por Fernando Simplicio para o ESP32 com SDK-IDF 4.0; 
 * para LoRa SX127x integrado ao Kit ESP32 LoRa Oled Heltec;
 * 
 */
 
/**
 * Standard C Lib;
 */
#include <stdio.h>
#include <string.h>

/**
 * FreeRTOs;
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * ESP Driver;
 */
#include "esp_system.h"
#include "driver/spi_master.h"
#include "soc/gpio_struct.h"
#include "driver/gpio.h"

/**
 * Logs 
 */
#include "esp_log.h"

/**
 * Debug?
 */
#define DEBUG 0

/*
 * Register definitions
 */
#define REG_FIFO                       0x00
#define REG_OP_MODE                    0x01
#define REG_FRF_MSB                    0x06
#define REG_FRF_MID                    0x07
#define REG_FRF_LSB                    0x08
#define REG_PA_CONFIG                  0x09
#define REG_LNA                        0x0c
#define REG_OCP                        0x0b
#define REG_FIFO_ADDR_PTR              0x0d
#define REG_FIFO_TX_BASE_ADDR          0x0e
#define REG_FIFO_RX_BASE_ADDR          0x0f
#define REG_FIFO_RX_CURRENT_ADDR       0x10
#define REG_IRQ_FLAGS                  0x12
#define REG_RX_NB_BYTES                0x13
#define REG_PKT_SNR_VALUE              0x19
#define REG_PKT_RSSI_VALUE             0x1a
#define REG_MODEM_CONFIG_1             0x1d
#define REG_MODEM_CONFIG_2             0x1e
#define REG_PREAMBLE_MSB               0x20
#define REG_PREAMBLE_LSB               0x21
#define REG_PAYLOAD_LENGTH             0x22
#define REG_MODEM_CONFIG_3             0x26
#define REG_RSSI_WIDEBAND              0x2c
#define REG_DETECTION_OPTIMIZE         0x31
#define REG_DETECTION_THRESHOLD        0x37
#define REG_SYNC_WORD                  0x39
#define REG_DIO_MAPPING_1              0x40
#define REG_VERSION                    0x42

/*
 * Transceiver modes
 */
#define MODE_LONG_RANGE_MODE           0x80
#define MODE_SLEEP                     0x00
#define MODE_STDBY                     0x01
#define MODE_TX                        0x03
#define MODE_RX_CONTINUOUS             0x05
#define MODE_RX_SINGLE                 0x06

/*
 * PA configuration
 */
#define PA_BOOST                       0x80
#define REG_PA_DAC                     0x4d

/*
 * IRQ masks
 */
#define IRQ_TX_DONE_MASK               0x08
#define IRQ_PAYLOAD_CRC_ERROR_MASK     0x20
#define IRQ_RX_DONE_MASK               0x40

#define PA_OUTPUT_RFO_PIN              0
#define PA_OUTPUT_PA_BOOST_PIN         1

#define TIMEOUT_RESET                  100

/**
 * Tamanho máximo do pacote;
 */
#define MAX_PKT_LENGTH                 255

/**
 * Variáveis Globais;
 */
static const char * TAG = "API_LORA: ";
static spi_device_handle_t __spi;
static int __implicit;

/**
 * Define o tamanho do pacote quando em modo implicit;
 */
static int __implicit_size = 100;
static long __frequency;
QueueHandle_t xQueue_LoRa;

/**
 * Write a value to a register.
 * @param reg Register index.
 * @param val Value to write.
 */
void 
lora_write_reg(int reg, int val)
{
   uint8_t out[2] = { 0x80 | reg, val };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in  
   };

   gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_CS_GPIO, 1);
}

/**
 * Read the current value of a register.
 * @param reg Register index.
 * @return Value of the register.
 */
int
lora_read_reg(int reg)
{
   uint8_t out[2] = { reg, 0xff };
   uint8_t in[2];

   spi_transaction_t t = {
      .flags = 0,
      .length = 8 * sizeof(out),
      .tx_buffer = out,
      .rx_buffer = in
   };

   gpio_set_level(CONFIG_CS_GPIO, 0);
   spi_device_transmit(__spi, &t);
   gpio_set_level(CONFIG_CS_GPIO, 1);
   return in[1];
}

/**
 * Perform physical reset on the Lora chip
 */
void 
lora_reset(void)
{
   gpio_set_level(CONFIG_RST_GPIO, 0);
   vTaskDelay(pdMS_TO_TICKS(1));
   gpio_set_level(CONFIG_RST_GPIO, 1);
   vTaskDelay(pdMS_TO_TICKS(10));
}

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
void 
lora_explicit_header_mode(void)
{
   __implicit = 0;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) & 0xfe);
}

/**
 * Configure implicit header mode.
 * All packets will have a predefined size.
 * @param size Size of the packets.
 */
void 
lora_implicit_header_mode(int size)
{
   __implicit = 1;
   __implicit_size = size;
   lora_write_reg(REG_MODEM_CONFIG_1, lora_read_reg(REG_MODEM_CONFIG_1) | 0x01);
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
}

/**
 * Sets the radio transceiver in idle mode.
 * Must be used to change registers and access the FIFO.
 */
void 
lora_idle(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_STDBY);
}

/**
 * Sets the radio transceiver in sleep mode.
 * Low power consumption and FIFO is lost.
 */
void 
lora_sleep(void)
{ 
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_SLEEP);
}

/**
 * Sets the radio transceiver in receive mode.
 * Incoming packets will be received.
 */
void 
lora_receive(void)
{
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_RX_CONTINUOUS);
}


void lora_set_ocp( unsigned char mA )
{
  unsigned char ocpTrim = 27;

  if (mA <= 120) {
    ocpTrim = (mA - 45) / 5;
  } else if (mA <=240) {
    ocpTrim = (mA + 30) / 10;
  }

   lora_write_reg(REG_OCP, 0x20 | (0x1F & ocpTrim));
}

/**
 * Configure power level for transmission
 * @param level 2-17, from least to most power
 */
void 
lora_set_tx_power(int level)
{
   if (level > 17)
   {
        if (level > 20) {
            level = 20;
        }
      
      // subtract 3 from level, so 18 - 20 maps to 15 - 17
      level -= 3;
      
      // High Power +20 dBm Operation (Semtech SX1276/77/78/79 5.4.3.)
      lora_write_reg(REG_PA_DAC, 0x87);
      lora_set_ocp(140); 
      
   } else 
   {
      if (level < 2) {
        level = 2;
      }
      //Default value PA_HF/LF or +17dBm
      lora_write_reg(REG_PA_DAC, 0x84);
      lora_set_ocp(100);
    }
    
   lora_write_reg(REG_PA_CONFIG, PA_BOOST | (level - 2));
}

/**
 * Set carrier frequency.
 * @param frequency Frequency in Hz
 */
void 
lora_set_frequency(long frequency)
{
   __frequency = frequency;

   uint64_t frf = ((uint64_t)frequency << 19) / 32000000;

   lora_write_reg(REG_FRF_MSB, (uint8_t)(frf >> 16));
   lora_write_reg(REG_FRF_MID, (uint8_t)(frf >> 8));
   lora_write_reg(REG_FRF_LSB, (uint8_t)(frf >> 0));
}

/**
 * Set spreading factor.
 * @param sf 6-12, Spreading factor to use.
 */
void 
lora_set_spreading_factor(int sf)
{
   if (sf < 6) sf = 6;
   else if (sf > 12) sf = 12;

   if (sf == 6) {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc5);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0c);
   } else {
      lora_write_reg(REG_DETECTION_OPTIMIZE, 0xc3);
      lora_write_reg(REG_DETECTION_THRESHOLD, 0x0a);
   }

   lora_write_reg(REG_MODEM_CONFIG_2, (lora_read_reg(REG_MODEM_CONFIG_2) & 0x0f) | ((sf << 4) & 0xf0));
}

/**
 * Set bandwidth (bit rate)
 * @param sbw Bandwidth in Hz (up to 500000)
 */
void 
lora_set_bandwidth(long sbw)
{
   int bw;

   if (sbw <= 7.8E3) bw = 0;
   else if (sbw <= 10.4E3) bw = 1;
   else if (sbw <= 15.6E3) bw = 2;
   else if (sbw <= 20.8E3) bw = 3;
   else if (sbw <= 31.25E3) bw = 4;
   else if (sbw <= 41.7E3) bw = 5;
   else if (sbw <= 62.5E3) bw = 6;
   else if (sbw <= 125E3) bw = 7;
   else if (sbw <= 250E3) bw = 8;
   else bw = 9;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0x0f) | (bw << 4));
}

/**
 * Set coding rate 
 * @param denominator 5-8, Denominator for the coding rate 4/x
 */ 
void 
lora_set_coding_rate(int denominator)
{
   if (denominator < 5) denominator = 5;
   else if (denominator > 8) denominator = 8;

   int cr = denominator - 4;
   lora_write_reg(REG_MODEM_CONFIG_1, (lora_read_reg(REG_MODEM_CONFIG_1) & 0xf1) | (cr << 1));
}

/**
 * Set the size of preamble.
 * @param length Preamble length in symbols.
 */
void 
lora_set_preamble_length(long length)
{
   lora_write_reg(REG_PREAMBLE_MSB, (uint8_t)(length >> 8));
   lora_write_reg(REG_PREAMBLE_LSB, (uint8_t)(length >> 0));
}

/**
 * Change radio sync word.
 * @param sw New sync word to use.
 */
void 
lora_set_sync_word(int sw)
{
   lora_write_reg(REG_SYNC_WORD, sw);
}

/**
 * Enable appending/verifying packet CRC.
 */
void 
lora_enable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) | 0x04);
}

/**
 * Disable appending/verifying packet CRC.
 */
void 
lora_disable_crc(void)
{
   lora_write_reg(REG_MODEM_CONFIG_2, lora_read_reg(REG_MODEM_CONFIG_2) & 0xfb);
}

/**
 * Perform hardware initialization.
 */
int 
lora_init(void)
{
   esp_err_t ret;

   /*
    * Configure CPU hardware to communicate with the radio chip
    */
   gpio_pad_select_gpio(CONFIG_RST_GPIO);
   gpio_set_direction(CONFIG_RST_GPIO, GPIO_MODE_OUTPUT);
   gpio_pad_select_gpio(CONFIG_CS_GPIO);
   gpio_set_direction(CONFIG_CS_GPIO, GPIO_MODE_OUTPUT);

   spi_bus_config_t bus = {
      .miso_io_num = CONFIG_MISO_GPIO,
      .mosi_io_num = CONFIG_MOSI_GPIO,
      .sclk_io_num = CONFIG_SCK_GPIO,
      .quadwp_io_num = -1,
      .quadhd_io_num = -1,
      .max_transfer_sz = 0
   };
           
   ret = spi_bus_initialize(VSPI_HOST, &bus, 0);
   assert(ret == ESP_OK);

   spi_device_interface_config_t dev = {
      .clock_speed_hz = 9000000,
      .mode = 0,
      .spics_io_num = -1,
      .queue_size = 1,
      .flags = 0,
      .pre_cb = NULL
   };
   ret = spi_bus_add_device(VSPI_HOST, &dev, &__spi);
   assert(ret == ESP_OK);

   /*
    * Perform hardware reset.
    */
   lora_reset();

   /*
    * Check version.
    */
   uint8_t version;
   uint8_t i = 0;
   while(i++ < TIMEOUT_RESET) {
      version = lora_read_reg(REG_VERSION);
      if(version == 0x12) break;
      vTaskDelay(2);
   }
   assert(i <= TIMEOUT_RESET + 1); // at the end of the loop above, the max value i can reach is TIMEOUT_RESET + 1

   /*
    * Default configuration.
    */
   lora_sleep(); //a função correta é lora_sleep(). Não é possível usar aqui lora_idle();
   lora_write_reg(REG_FIFO_RX_BASE_ADDR, 0);
   lora_write_reg(REG_FIFO_TX_BASE_ADDR, 0);
   lora_write_reg(REG_LNA, lora_read_reg(REG_LNA) | 0x03);
   lora_write_reg(REG_MODEM_CONFIG_3, 0x04);
   lora_set_tx_power(14);

   lora_set_spreading_factor(7);
   lora_set_bandwidth(125E3);
   lora_set_sync_word(0x34);

/**
 * Configure explicit header mode.
 * Packet size will be included in the frame.
 */
   lora_explicit_header_mode();
   lora_idle();

/**
 * modo de recepção habilitado;
 */   
   lora_receive();
   
   return 1;
}


/**
 * Send a packet.
 * @param buf Data to be sent
 * @param size Size of data.
 */
void 
lora_send_packet(uint8_t *buf, int size)
{
   /*
    * Transfer data to radio.
    */
   lora_idle();
   /**
    * Reseta o endereço da FIFO e Payload Length;
    */
   lora_write_reg(REG_FIFO_ADDR_PTR, 0);
   lora_write_reg(REG_PAYLOAD_LENGTH, 0);

   /**
    * Verifica o tamanho do pacote a ser transmitido;
    */
   if ( size > MAX_PKT_LENGTH ) size = MAX_PKT_LENGTH;

   /**
    * Escreve o valor a ser transmitido na FIFO do Rádio LoRa;
    */
   for(int i=0; i<size; i++) 
      lora_write_reg(REG_FIFO, *buf++);
   
   /**
    * Registra o tamanho do Payload a ser transmitido;
    */
   lora_write_reg(REG_PAYLOAD_LENGTH, size);
   

    if( DEBUG )
      ESP_LOGI( TAG, "REG_PAYLOAD_LENGH = %d", lora_read_reg(REG_PAYLOAD_LENGTH) );

   /*
    * Start transmission and wait for conclusion.
    */
  /**
   * Coloca o Rádio LoRa em modo de transmissão e inicia a transmissão;
   */
   lora_write_reg(REG_OP_MODE, MODE_LONG_RANGE_MODE | MODE_TX);
   /**
    * Aguarda o término da transmissão;
    */
   while((lora_read_reg(REG_IRQ_FLAGS) & IRQ_TX_DONE_MASK) == 0)
      vTaskDelay(2); 

    /**
     * Apaga os flags de IRQ's do rádio;
     */
   lora_write_reg(REG_IRQ_FLAGS, IRQ_TX_DONE_MASK);
   
   /**
    *  Após o envio, deixamos o rádio LoRa configurado em modo de recepção;
    */
    lora_receive();
}

/**
 * Read a received packet.
 * @param buf Buffer for the data.
 * @param size Available size in buffer (bytes).
 * @return Number of bytes received (zero if no packet available).
 */
int 
lora_receive_packet(uint8_t *buf, int size)
{
   int len = 0;

   /**
    * Apaga os flags de IRQs;
    */
   int irqFlags = lora_read_reg(REG_IRQ_FLAGS);
   
   if (__implicit) lora_implicit_header_mode(__implicit_size);
   else lora_explicit_header_mode();
  
   lora_write_reg(REG_IRQ_FLAGS, irqFlags);

   
   if( (irqFlags & IRQ_RX_DONE_MASK) &&  (irqFlags & IRQ_PAYLOAD_CRC_ERROR_MASK)  == 0 ) 
   {
       /*
        * Find packet size.
        */
       if (__implicit) len = lora_read_reg(REG_PAYLOAD_LENGTH);
       else len = lora_read_reg(REG_RX_NB_BYTES);

        if( DEBUG )
          ESP_LOGI( TAG, "REG_PAYLOAD_LENGH = %d IRQ = %d", len, irqFlags);

       /**
        * set FIFO address to current RX address
        */
       lora_write_reg(REG_FIFO_ADDR_PTR, lora_read_reg(REG_FIFO_RX_CURRENT_ADDR));

       /*
        * Transfer data from radio.
        */
       lora_idle();

       if(len > size) len = size;
       for(int i=0; i<len; i++) 
          *buf++ = lora_read_reg(REG_FIFO);
      
   } 
   
   lora_receive();

   return len;
}

/**
 * Returns non-zero if there is data to read (packet received).
 */
int
lora_received(void)
{
   if(lora_read_reg(REG_IRQ_FLAGS) & IRQ_RX_DONE_MASK) return 1;
   return 0;
}

/**
 * Return last packet's RSSI.
 */
int 
lora_packet_rssi(void)
{
   return (lora_read_reg(REG_PKT_RSSI_VALUE) - (__frequency < 868E6 ? 164 : 157));
}

/**
 * Return last packet's SNR (signal to noise ratio).
 */
float 
lora_packet_snr(void)
{
   return ((int8_t)lora_read_reg(REG_PKT_SNR_VALUE)) * 0.25;
}

/**
 * Shutdown hardware.
 */
void 
lora_close(void)
{
   lora_sleep();
   spi_bus_remove_device(__spi);
}

void 
lora_dump_registers(void)
{
   int i;
   printf("00 01 02 03 04 05 06 07 08 09 0A 0B 0C 0D 0E 0F\n");
   for(i=0; i<0x40; i++) {
      printf("%02X ", lora_read_reg(i));
      if((i & 0x0f) == 0x0f) printf("\n");
   }
   printf("\n");
}


static void IRAM_ATTR gpio_isr_handler( void * pvParameter )
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    static int cnt_1 = 0;
    xQueueSendFromISR( xQueue_LoRa, &cnt_1, &xHigherPriorityTaskWoken );
    if( xHigherPriorityTaskWoken == pdTRUE )
    {
        portYIELD_FROM_ISR();
    }

#ifdef CONFIG_DEBUG_LED_BUILDING_GPIO
    gpio_set_level( CONFIG_DEBUG_LED_BUILDING_GPIO, cnt_1 % 2 );
#endif
    cnt_1++;
}

void
lora_enable_irq( void )
{

    if( ( xQueue_LoRa = xQueueCreate( 5,  sizeof( uint32_t ) ) ) == NULL )
    {
      if( DEBUG )
          ESP_LOGI( TAG, "error - nao foi possivel alocar xQueue_LoRa.\n" );
      return;
    } 

#ifdef CONFIG_DEBUG_LED_BUILDING_GPIO
    gpio_pad_select_gpio( CONFIG_DEBUG_LED_BUILDING_GPIO ); 
    gpio_set_direction( CONFIG_DEBUG_LED_BUILDING_GPIO , GPIO_MODE_OUTPUT );
    gpio_set_level( CONFIG_DEBUG_LED_BUILDING_GPIO , 0 );
#endif

    gpio_config_t io_conf;   
    io_conf.intr_type = GPIO_INTR_POSEDGE; 
    io_conf.mode = GPIO_MODE_INPUT;  
    io_conf.pin_bit_mask = ((1ULL<<CONFIG_IRQ_GPIO)); 
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE; 
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; 
    gpio_config(&io_conf);  
  
    gpio_install_isr_service(0);

    gpio_isr_handler_add( CONFIG_IRQ_GPIO, gpio_isr_handler, NULL ); 

}

void
lora_disable_irq( void )
{
    gpio_intr_disable( CONFIG_IRQ_GPIO );
    vQueueDelete( xQueue_LoRa );
}

