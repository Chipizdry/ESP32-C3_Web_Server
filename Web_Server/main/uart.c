
#include "uart.h"
#include "esp_log.h"
#include <string.h>

static QueueHandle_t uart_queue; 

QueueHandle_t uart_command_queue = NULL;

static uint8_t rx_buffer[RX_BUFFER_SIZE]; // Приватный буфер

static const char* TAG = "UART";
void init_uart() {
    const uart_port_t uart_num = UART_NUM_1;

    // Настройки UART
    uart_config_t uart_config = {
        .baud_rate = UART_BAUD_RATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    uart_param_config(uart_num, &uart_config);
    uart_set_pin(uart_num, TXD_PIN, RXD_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);

    // Инициализация UART с использованием DMA
  
    esp_err_t uart_err =  uart_driver_install(UART_NUM_1,UART_BUF_SIZE * 2, UART_BUF_SIZE* 2, 100, &uart_queue, ESP_INTR_FLAG_IRAM);
    if (uart_err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install UART driver: %s", esp_err_to_name(uart_err));
    }

     // Настройка паттерна для детектирования
    uart_enable_pattern_det_baud_intr(UART_NUM_1, '\n', 1, 9, 0, 0);  // Паттерн '\n'
    uart_pattern_queue_reset(UART_NUM_1, 20);

}


uint16_t calcCRC16(uint8_t *buffer, uint8_t u8length) {
	unsigned int temp, temp2, flag;
	temp = 0xFFFF;
	for (unsigned char i = 0; i < u8length; i++) {
		temp = temp ^ buffer[i];
		for (unsigned char j = 1; j <= 8; j++) {
			flag = temp & 0x0001;
			temp >>= 1;
			if (flag)
				temp ^= 0xA001;
		}
	}
	// Reverse byte order.
	temp2 = temp >> 8;
	temp = (temp << 8) | temp2;
	temp &= 0xFFFF;
	// the returned value is already swapped
	// crcLo byte is first & crcHi byte is last
	return temp;

}

static void IRAM_ATTR uart_isr(void* arg) {
    // Код, который будет выполняться в ISR
}

// Периодическая отправка регулярных запросов
void periodic_request_task(void *pvParameters) {
    uart_command_t cmd;
    while (1) {
        cmd.cmd_type = 0;
        cmd.command = COMMAND_REGULAR_REQUEST;
        cmd.payload_len = 0;

        // Отправка регулярного запроса в очередь команд
        if (xQueueSend(uart_command_queue, &cmd, 0) != pdPASS) {
            ESP_LOGW(TAG, "Failed to send regular request");
        }
        vTaskDelay(pdMS_TO_TICKS(1000));  // Периодическое ожидание 1 секунду
    }
}


// Функция для формирования запроса
uint16_t form_tuya_request(uint8_t cmd_type, uint8_t command, uint8_t *payload, uint16_t payload_len, uint8_t *request) {
    uint16_t pos = 0;

    // 1. Заголовок (2 байта)
    request[pos++] = HEADER_1;
    request[pos++] = HEADER_2;

    // 2. Тип пакета (1 байт), 0x00 для запроса
                    //         0x01 для команд
                    //         0x02 для файлов прошивки
              
    request[pos++] =cmd_type;

    // 3. Идентификатор команды (1 байт)
    request[pos++] = command;

    // 4. Длина данных (2 байта) — длина полезной нагрузки
    request[pos++] = (payload_len >> 8) & 0xFF;  // Старший байт длины
    request[pos++] = payload_len & 0xFF;         // Младший байт длины

    // 5. Полезная нагрузка (N байт)
    if (payload != NULL && payload_len > 0) {
        memcpy(&request[pos], payload, payload_len);
        pos += payload_len;
    }

   // 6. CRC (2 байта) — вычисляем CRC для всех предыдущих данных
    uint16_t crc = calcCRC16(request, pos);
    request[pos++] = crc & 0xFF;         // Младший байт CRC
    request[pos++] = (crc >> 8) & 0xFF;  // Старший байт CRC

    return pos;  // Возвращаем длину сформированного запроса
}



// Задача для отправки команд через UART
void uart_command_task(void *pvParameters) {
    uart_command_t cmd;
    uint8_t request[64];
    uint16_t request_len;

    while (1) {
        // Ожидаем запрос из очереди
        if (xQueueReceive(uart_command_queue, &cmd, portMAX_DELAY) == pdPASS) {
            // Формируем запрос
            request_len = form_tuya_request(cmd.cmd_type, cmd.command, cmd.payload, cmd.payload_len, request);

            // Логируем запрос
            ESP_LOGI(TAG, "Sending command %d via UART, request length: %d", cmd.command, request_len);
            ESP_LOG_BUFFER_HEX(TAG, request, request_len);


           size_t buffered_size;
				uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
				ESP_LOGI(TAG, "Buffered data size: %d", buffered_size);
				
				if (buffered_size > 256) {
				    ESP_LOGW(TAG, "Buffer overflow: %d bytes buffered", buffered_size);
				    uart_flush_input(UART_NUM_1);  // Сброс буфера
				}

            // Используем мьютекс для доступа к UART
            if (xSemaphoreTake(uart_mutex, pdMS_TO_TICKS(50))) {
                int res = uart_write_bytes(UART_NUM_1, (const char *)request, request_len);
                if (res < 0) {
                    ESP_LOGE(TAG, "Failed to write bytes to UART");
                }
                xSemaphoreGive(uart_mutex);  // Освобождаем доступ к UART
                  uart_flush_input(UART_NUM_1); 
            } else {
                ESP_LOGE(TAG, "Failed to take UART mutex");
            }
        }
    }
}



// Задача для обработки данных из UART
 void uart_event_task(void *pvParameters) {
    uart_event_t event;
    size_t buffered_size;
    uint8_t* data = (uint8_t*) malloc(UART_BUF_SIZE);
    
    while (1) {
        // Ждём, когда придёт событие из UART
        if (xQueueReceive(uart_queue, (void*)&event, (TickType_t)portMAX_DELAY)) {
            bzero(data,UART_BUF_SIZE);

            switch (event.type) {
                // Данные пришли
                case UART_DATA:
                    uart_get_buffered_data_len(UART_NUM_1, &buffered_size);
                    int len = uart_read_bytes(UART_NUM_1, data, event.size, pdMS_TO_TICKS(100));
                    ESP_LOGI(TAG, "Received data of length: %d", len);
                  
                    process_received_data(data,len); 
                    break;

                // Ошибка фрейма
                case UART_FRAME_ERR:
                    ESP_LOGE(TAG, "UART Frame Error");
                    break;

                // Ошибка паритета
                case UART_PARITY_ERR:
                    ESP_LOGE(TAG, "UART Parity Error");
                    break;

                // Ошибка буфера
                case UART_BUFFER_FULL:
                    ESP_LOGW(TAG, "Ring buffer full");
                    uart_flush_input(UART_NUM_1);  // Очищаем входной буфер
                    xQueueReset(uart_queue);     // Сбрасываем очередь
                    break;

                // Таймаут
                case UART_FIFO_OVF:
                    ESP_LOGW(TAG, "UART FIFO Overflow");
                    uart_flush_input(UART_NUM_1);  // Очищаем входной буфер
                    xQueueReset(uart_queue);     // Сбрасываем очередь
                    break;

                // Событие по умолчанию
                default:
                    ESP_LOGI(TAG, "UART event type: %d", event.type);
                    break;
            }
        }
    }
    free(data);
    vTaskDelete(NULL);
}


 // Функция обработки принятых данных
void process_received_data(uint8_t *data, int len) {
  
      ESP_LOG_BUFFER_HEX(TAG, data, len); 
     if (len < 6) {
        ESP_LOGW(TAG, "Packet too short");
        return;
    }
    
     // Проверяем заголовок
    if (data[0] != HEADER_1 || data[1] != HEADER_2) {
        ESP_LOGW(TAG, "Invalid header");
        return;
    }
    
      // Проверяем CRC
    uint16_t crc_received = (data[len - 1] << 8) | data[len - 2];
    uint16_t crc_calculated = calcCRC16(data,len - 2);
    ESP_LOGI(TAG, "Received CRC: %d", crc_received);
    ESP_LOGI(TAG, "Calculated CRC: %d", crc_calculated); 
    if (crc_received != crc_calculated) {
        ESP_LOGE(TAG, "CRC mismatch");
        return;
    }
      // Извлекаем длину полезной нагрузки
    uint16_t payload_len = (data[4] << 8) | data[5];
    uint8_t *payload = &data[6];  // Указатель на начало полезной нагрузки
  // Извлекаем команду
     uint8_t cmd_receive = data[3];
// Обработка команды
    switch (cmd_receive) {
		
		 case 0:
		 
		      ESP_LOGI(TAG, "Life pulse");
		      
            break;
        case 4:  // Параметры батареи
            if (payload_len >= 6) {
                uint16_t battery_voltage = (payload[0] << 8) | payload[1];
                uint16_t battery_current = (payload[2] << 8) | payload[3];
                uint16_t battery_temperature = (payload[4] << 8) | payload[5];

                ESP_LOGI(TAG, "Battery Voltage: %d", battery_voltage);
                ESP_LOGI(TAG, "Battery Current: %d", battery_current);
                ESP_LOGI(TAG, "Battery Temperature: %d", battery_temperature);

                // Записываем значения в соответствующие переменные
                // Например:
                // g_battery_voltage = battery_voltage;
                // g_battery_current = battery_current;
                // g_battery_temperature = battery_temperature;
            }
            break;

        case 6:  // Параметры нагрузки
            if (payload_len >= 6) {
                uint16_t load_voltage = (payload[0] << 8) | payload[1];
                uint16_t load_current = (payload[2] << 8) | payload[3];
                uint16_t load_power = (payload[4] << 8) | payload[5];

                ESP_LOGI(TAG, "Load Voltage: %d", load_voltage);
                ESP_LOGI(TAG, "Load Current: %d", load_current);
                ESP_LOGI(TAG, "Load Power: %d", load_power);

                // Записываем значения в соответствующие переменные
            }
            break;

        case 8:  // Параметры входного напряжения сети 220-380В
            if (payload_len >= 4) {
                uint16_t grid_voltage = (payload[0] << 8) | payload[1];
                uint16_t grid_frequency = (payload[2] << 8) | payload[3];

                ESP_LOGI(TAG, "Grid Voltage: %d", grid_voltage);
                ESP_LOGI(TAG, "Grid Frequency: %d", grid_frequency);

                // Записываем значения в соответствующие переменные
            }
            break;

        case 10:  // Параметры накопительного маховика
            if (payload_len >= 10) {
                uint16_t flywheel_speed = (payload[0] << 8) | payload[1];
                uint16_t flywheel_current = (payload[2] << 8) | payload[3];
                uint16_t flywheel_pwm = (payload[4] << 8) | payload[5];
                uint16_t active_windings = (payload[6] << 8) | payload[7];
                uint16_t flywheel_power = (payload[8] << 8) | payload[9];

                ESP_LOGI(TAG, "Flywheel Speed: %d", flywheel_speed);
                ESP_LOGI(TAG, "Flywheel Current: %d", flywheel_current);
                ESP_LOGI(TAG, "Flywheel PWM: %d", flywheel_pwm);
                ESP_LOGI(TAG, "Active Windings: %d", active_windings);
                ESP_LOGI(TAG, "Flywheel Power: %d", flywheel_power);

                // Записываем значения в соответствующие переменные
            }
            break;

        case 12:  // Параметры солнечных панелей
            if (payload_len >= 6) {
                uint16_t solar_voltage = (payload[0] << 8) | payload[1];
                uint16_t solar_current = (payload[2] << 8) | payload[3];
                uint16_t solar_power = (payload[4] << 8) | payload[5];

                ESP_LOGI(TAG, "Solar Panel Voltage: %d", solar_voltage);
                ESP_LOGI(TAG, "Solar Panel Current: %d", solar_current);
                ESP_LOGI(TAG, "Solar Panel Power: %d", solar_power);

                // Записываем значения в соответствующие переменные
            }
            break;

        default:
            ESP_LOGW(TAG, "Unknown command: %d", cmd_receive);
            break;
    }
}
