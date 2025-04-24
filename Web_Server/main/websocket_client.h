// websocket_client.h
#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include "esp_websocket_client.h"
#include "esp_event.h"
#include "esp_log.h"
#include "freertos/queue.h"

typedef struct {
    esp_websocket_client_handle_t client;
    bool is_connected;
    char uri[256];
    char device_id[32];
    QueueHandle_t out_queue;  // Очередь для отправки сообщений
    QueueHandle_t in_queue;   // Очередь для принятых команд
} websocket_client_t;

// Инициализация WebSocket клиента
void websocket_client_init(websocket_client_t *ws_client, const char *uri, const char *device_id);

// Запуск задачи WebSocket клиента
void websocket_client_start_task(websocket_client_t *ws_client);

// Отправка данных через WebSocket (неблокирующая, через очередь)
void websocket_client_send(websocket_client_t *ws_client, const char *data);


// Получение команды (из очереди)
bool websocket_client_get_command(websocket_client_t *ws_client, char *buffer, size_t buffer_size);

void websocket_client_task(void *pvParameters);

#endif // WEBSOCKET_CLIENT_H