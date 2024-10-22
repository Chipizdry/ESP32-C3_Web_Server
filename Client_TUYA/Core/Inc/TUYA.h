/*
 * TUYA.h
 *
 *  Created on: Oct 8, 2024
 *      Author: chipi
 */

#ifndef INC_TUYA_H_
#define INC_TUYA_H_


#include "stm32f4xx_hal.h"  // Замените на вашу модель микроконтроллера, например stm32f4xx_hal.h
#include "main.h"
#define RX_BUFFER_SIZE 32  // Размер буфера для приема данных
#define TX_BUFFER_SIZE 32  // Размер буфера для передачи данных
#define HEADER_1 0x55
#define HEADER_2 0xAA

// Глобальные переменные для передачи и приема
extern uint8_t rx_buffer[RX_BUFFER_SIZE];
extern uint8_t tx_buffer[TX_BUFFER_SIZE];
extern uint8_t update_data;
extern uint8_t cmd_in;
extern UART_HandleTypeDef huart1;  // Объявляем huart1 как внешнюю переменную
// Инициализация UART с DMA
void UART_DMA_Init(UART_HandleTypeDef *huart, DMA_HandleTypeDef *hdma_tx, DMA_HandleTypeDef *hdma_rx);
extern DMA_HandleTypeDef hdma_usart1_rx;
// Обработка полученных данных
void send_response(uint8_t *payload, uint16_t payload_len, uint8_t command);

// Функция для отправки данных по DMA
//void send_response(uint8_t *data, uint16_t length);

// Прерывание для IDLE линии (используется для определения завершения приема данных)
void HAL_UART_IDLECallback(UART_HandleTypeDef *huart);

uint16_t calcCRC16(uint8_t *buffer, uint8_t u8length);

#endif /* INC_TUYA_H_ */
