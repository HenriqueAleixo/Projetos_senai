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
 
#ifndef __LORA_H__
#define __LORA_H__

/**
 * FreeRTOs;
 */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"

/**
 * Estrutura dos Pacotes de dados da LoRa; 
 * 
* <PREAMBLE><SYNC_WORD> {<HEADER><CRC>}<PAYLOAD><PAYLOAD_CRC>
                                |
                                V
* <PREAMBLE><SYNC_WORD> {<LENGTH BYTE><ADDRESS BYTE><CRC>}<PAYLOAD><PAYLOAD_CRC>
 * 	                                  
 */                      


/**
 * Função responsável pelo reset do chip LoRa SX1276;
 * A qualquer momento o ESP32 pode resetar o chip SX1276;
 */
void lora_reset(void);
/**
 * O modo explicito é o padrão do LoRa SX1276. 
 * Neste modo, o campo <HEADER> é habilitado e carrega as informações referente:
 *  - A quantidade de bytes contido no payload;
 *  - A taxa de código de correção de erro (4/8);
 *  - Se será utilizado o CRC (16 bits) no final do Payload;
 */
void lora_explicit_header_mode(void);

/**
 * O modo implicito é utilizado quando o payload, coding rate e CRC são conhecidos ou
 * pré definidos e tem a vantagem de reduzir o tempo de transmissão pois o campo 
 * <HEADER> é removido do pacote a ser transmitido. Neste modo, o tamanho do payload, 
 * da taxa de codificação de erro e a presença do CRC devem ser manualmente configurados
 * em ambos os end-devices LoRa.
 */
void lora_implicit_header_mode(int size);

/**
 * É possível ler qualquer registrador interno do SX1276 via SPI. Entretanto, para escrever
 * nos registradores o SX1276 precisa estar no modo Sleep e Standby; 
 * A função Idle coloca o SX1276 em modo Standby, no qual deve ser utilizado para alterar
 * registradores e acessar a FIFO.
 */
void lora_idle(void);

/**
 * Função responsável por colocar o SX1276 em modo de baixo consumo de energia elétrica.
 * Estando neste modo, os dados armazenados na FIFO do SX1276 são perdidos.
 */
void lora_sleep(void); 

/**
 * Função responsável em colocar o SX1276 em modo de recepção. 
 * Observação: Lembre-se que o LoRa é half-duplex, ou seja, durante a recepção de um dado
 * o dispositivo não pode transmitir, e vice e versa.
 */
void lora_receive(void);

/**
 * Função para configuração da potência de transmissão do LoRa;
 * Atribuida entre 2 a 20 (dbm); 
 */
void lora_set_tx_power(int level);

/**
 * Função responsável pela configuração da frequência de operação do LoRa;
 * Podendo ser 433E6 para Asia; 868E6 para Europa e 915E6 para EUA;
 */
void lora_set_frequency(long frequency);

/**
 * A LoRa implementa uma tecnica de modulação por espalhamento espectral de chirps (Chirp
 * Spreading Spectrum – CSS), que varia a frequencia sem mudar a fase entre
 * símbolos adjacentes, fazendo com que o sinal resultante seja resistente 
 * a interferencias por ruído ou sinais com frequencias próximas.
 * O valor de SF pode variar 7 a 12. 
 */
void lora_set_spreading_factor(int sf);

/**
 * Define a largura de banda da LoRa, no qual utiliza três valores programáveis: 
 * 125kHz, 250kHz ou 500kHz;
 */
void lora_set_bandwidth(long sbw);

/**
 * Define a taxa de código da LoRa (Code Rate CR); //5 - 8
 */
void lora_set_coding_rate(int denominator);

/**
 * O preamble é utilizado para sincronizar o receptor ao dado; 
 * Seu comprimento é de 6 a 65535;
 */
void lora_set_preamble_length(long length);

/**
 * Também chamado de Pattern recognition. Sync word do pacote transmitido é comparadado
 * com o pré-configurado sync word do receptor. Caso iguais o pacote é processado pelo receptor
 * caso contrário, o pacote é descartado.
 */
void lora_set_sync_word(int sw); //0X34

/**
 * Habilita o Payload CRC quando operando em modo Explicito; 
 */
void lora_enable_crc(void);

/**
 * Desabilita o Payload CRC quando operando em modo Explicito; 
 */
void lora_disable_crc(void);

/**
 * Inicializa LoRa;
 */
int lora_init(void);

/**
 * Função responsavel pelo envio de um pacote de dados.
 *
 * @param      buf   The buffer
 * @param[in]  size  The size
 */
void lora_send_packet(uint8_t *buf, int size);

/**
 * Função responsavel pelo recebimento de um pacote de dados.
 *
 * @param      buf   The buffer
 * @param[in]  size  The size
 */
int lora_receive_packet(uint8_t *buf, int size);

/**
 * Função que verifica se algum pacote foi recebido;
 */
int lora_received(void);

/**
 * Retorna o valor do RSSI no pacote recebido pelo receptor;
 */
int lora_packet_rssi(void);

/**
 * Retorna o valor do SNR no pacote recebido pelo receptor;
 */
float lora_packet_snr(void);

/**
 * Função que deleta o objeto SPI;
 */
void lora_close(void);

/**
 * Função que imprime em hexadecimal o conteúdo dos registradores do SX1276 para debug;
 */
void lora_dump_registers(void);

/**
 * Habilita a interrupção externa DIO0. Assim, quando um pacote de dados for recebido no
 * LoRa, ocorrerá uma borda de subida neste pino, provocando assim uma interrupção. 
 * Logo após, o microcontrolador pode ler a FIFO e resgatar os bytes recebidos no SX1276;
 */
void lora_enable_irq( void );

/**
 * Desabilita a interrupção externa em DIO0;
 */
void lora_disable_irq( void );

/**
 * Função para parametrização do circuito OcpTrim do SX1276. Este circuito faz a proteção
 * contra sobrecargas de corrente em condições adversas de carga de RF.
 * Isso tem o benefício adicional de proteger produtos químicos de bateria com corrente de 
 * pico limitada capacidade e minimizando o pior consumo de PA no cálculo da duração da bateria.
 */
void lora_set_ocp( unsigned char mA );

/**
 * Queue responsável em sinalizar para a task de recepção serial que um determinado pacote
 * está armazenado na FIFO da LoRa.
 */
extern QueueHandle_t xQueue_LoRa;

#endif
