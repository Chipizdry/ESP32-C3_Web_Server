
#ifndef MAIN_UART_H_
#define MAIN_UART_H_

#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "driver/uart.h"
#include "freertos/semphr.h"
#include "driver/gpio.h"
#include <stdint.h>
#include "esp_netif.h"
// Определите пины для UART
#define TXD_PIN (GPIO_NUM_10)  // Передача данных (TX)
#define RXD_PIN (GPIO_NUM_9)  // Приём данных (RX)
#define RTS_PIN (UART_PIN_NO_CHANGE)  // Управление передачей данных (опционально)

#define UART_PORT_NUM      UART_NUM_1
#define UART_BAUD_RATE     57600
#define UART_BUF_SIZE      (1024)
#define RX_BUFFER_SIZE 32
#define HEADER_1 0x55
#define HEADER_2 0xAA

// Определяем командные коды
#define COMMAND_REGULAR_REQUEST 0x01  // Команда для регулярного запроса данных
#define COMMAND_FRONTEND 0x02         // Команда от фронтэнда
#define CMD_IP_CONFIG     0x14  // Команда для передачи IP данных
//#define TAG "UART"
#define QUEUE_SIZE 10

// Структура для передачи данных через очередь
typedef struct {
    uint8_t cmd_type;
    uint8_t command;
    uint8_t payload[10];  // Полезная нагрузка, максимальная длина 10 байт
    uint8_t payload_len;
} uart_command_t;

extern QueueHandle_t uart_command_queue;
extern SemaphoreHandle_t uart_mutex;

// Функции для работы с UART
void init_uart(void);
void process_received_data(uint8_t *data, int len);
uint16_t calcCRC16(uint8_t *buffer, uint8_t u8length);
uint16_t form_tuya_request(uint8_t cmd_type, uint8_t command, uint8_t *payload, uint16_t payload_len, uint8_t *request);
static void IRAM_ATTR uart_isr(void* arg);
void periodic_request_task(void *pvParameters);
void uart_command_task(void *pvParameters);
void uart_event_task(void *pvParameters);
void send_ip_config_command();
#endif /* MAIN_UART_H_ */
