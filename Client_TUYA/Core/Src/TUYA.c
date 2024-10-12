
#include "TUYA.h"
#include "stm32f4xx_hal.h"

// Глобальные буферы для приема и передачи
uint8_t rx_buffer[RX_BUFFER_SIZE];
uint8_t tx_buffer[TX_BUFFER_SIZE];


// Обработчик IDLE линии UART для обработки завершения приема данных
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart) {
    if (__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE)) {
        __HAL_UART_CLEAR_IDLEFLAG(huart);

        // Вычисляем количество принятых байт
        uint16_t received_length = RX_BUFFER_SIZE - __HAL_DMA_GET_COUNTER(huart->hdmarx);

        // Обрабатываем данные
        process_received_data(rx_buffer, received_length);

    }
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

void send_updated_data() {
    uint8_t response[64];
    uint16_t pos = 0;

    // Заголовок
    response[pos] = HEADER_1;
    response[pos++] = HEADER_2;

    // Тип пакета: ответ
    response[pos++] = 0x01;

    // Команда: 0x05, например, для отправки обновленных данных
    response[pos++] = 0x05;

    // Данные — пример значений напряжения, тока и других параметров
    float voltage = 230.5;
    float current = 5.2;
    float temperature = 24.3;

    uint16_t voltage_val = (uint16_t)(voltage * 10);
    uint16_t current_val = (uint16_t)(current * 10);
    uint16_t temp_val = (uint16_t)(temperature * 10);

    response[pos++] = (voltage_val >> 8) & 0xFF;
    response[pos++] = voltage_val & 0xFF;
    response[pos++] = (current_val >> 8) & 0xFF;
    response[pos++] = current_val & 0xFF;
    response[pos++] = (temp_val >> 8) & 0xFF;
    response[pos++] = temp_val & 0xFF;

    // Вычисляем CRC
    uint16_t crc = calcCRC16(response, pos);
    response[pos++] = crc & 0xFF;
    response[pos++] = (crc >> 8) & 0xFF;

    // Отправляем ответ через UART
    HAL_UART_Transmit_DMA(&huart1, response, pos);
}

// Функция для обработки полученных данных
void process_received_data(uint8_t *data, uint16_t length) {
    // Минимальная длина пакета: заголовок (2 байта) + тип пакета (1 байт) + команда (1 байт) + длина данных (2 байта) + CRC (2 байта)
    if (length < 8) {
        // Пакет слишком короткий
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
    	             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    	             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        return;
    }

    // Проверяем заголовок
    if (data[0] != HEADER_1 || data[1] != HEADER_2) {
        // Неверный заголовок
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
    	             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    	             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        return;
    }

    // Извлекаем тип пакета и команду
    uint8_t packet_type = data[2];
    if(packet_type==0){

    	 // Если данные не обновлялись
if (update_data == 0) {
// Отправляем ответ «пульс» (без полезной нагрузки)
		uint8_t heartbeat[6];
		heartbeat[0] = HEADER_1;
		heartbeat[1] = HEADER_2;
		heartbeat[2] = 0x01;  // Тип пакета: ответ
		heartbeat[3] = 0x00;  // Команда для "пульса" или пустого ответа
		// Отправляем ответ через UART
		uint16_t crc = calcCRC16(heartbeat, 4);
		heartbeat[4] = crc & 0xFF;
		heartbeat[5] = (crc >> 8) & 0xFF;

		HAL_UART_Transmit_DMA(&huart1, heartbeat, 6);  // Передаем пакет
}

			// Если данные обновились
    	        else if (update_data == 1) {
    	       // Отправляем новые данные
    	            send_updated_data();
    	            update_data = 0;  // Сбрасываем флаг обновления после отправки
    	            return;
    	           }

    }
    uint8_t command = data[3];
    cmd_in=data[3];
    // Извлекаем длину полезной нагрузки
    uint16_t payload_len = (data[4] << 8) | data[5];

    // Проверяем корректность длины пакета
    if (payload_len + 8 != length) {
        // Неверная длина пакета
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
    	             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    	             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        return;
    }

    // Проверяем CRC
    uint16_t crc_received = (data[length - 2] << 8) | data[length - 1];
    uint16_t crc_calculated = calcCRC16(data, length - 2);
    if (crc_received != crc_calculated) {
        // Неверный CRC
    	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
    	             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
    	             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

        return;
    }

    // Извлекаем полезную нагрузку
    uint8_t *payload = &data[6];

    // Обрабатываем команду
    switch (command) {
        case 0x03: // Параметры батареи
                if (payload_len == 6) {
                uint16_t lowThreshold = (payload[0] << 8) | payload[1];
                uint16_t highThreshold = (payload[2] << 8) | payload[3];
                uint16_t maxCurrentRange = (payload[4] << 8) | payload[5];

                // Преобразуем значения обратно в float
                float low = lowThreshold / 10.0;
                float high = highThreshold / 10.0;
                float maxCurrent = maxCurrentRange / 10.0;

                // Сохраняем в переменные или используем
                // Пример: отображение значений
                printf("Battery settings - Low: %.2f, High: %.2f, Max Current: %.2f\n", low, high, maxCurrent);
               }
            break;

        case 0x04: // Параметры нагрузки
                if (payload_len == 6) {
                uint16_t maxLoad = (payload[0] << 8) | payload[1];
                uint16_t outputVoltage = (payload[2] << 8) | payload[3];
                uint16_t maxCurrentDifference = (payload[4] << 8) | payload[5];

                // Преобразуем значения обратно в float
                float load = maxLoad / 10.0;
                float voltage = outputVoltage / 10.0;
                float currentDiff = maxCurrentDifference / 10.0;

                // Сохраняем в переменные или используем
                // Пример: отображение значений
                printf("Load settings - Max Load: %.2f, Output Voltage: %.2f, Max Current Difference: %.2f\n", load, voltage, currentDiff);
                }
              break;

        default:
            // Обработка других команд
        	HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
        	             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
        	             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

            break;
    }
}

// Колбэк завершения передачи данных по DMA
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if (huart == &huart1) {
        // Запускаем прием данных через DMA
        // Включаем прерывание по IDLE для отслеживания завершения приема данных
        HAL_UARTEx_ReceiveToIdle_DMA(&huart1,rx_buffer, RX_BUFFER_SIZE);
             __HAL_UART_ENABLE_IT(&huart1, UART_IT_IDLE);
             __HAL_DMA_DISABLE_IT(&hdma_usart1_rx, DMA_IT_HT);

    }
}

// Функция отправки данных через DMA
void send_response(uint8_t *data, uint16_t length) {
  // Отправка через DMA
    HAL_UART_Transmit_DMA(&huart1,data, length);
}

