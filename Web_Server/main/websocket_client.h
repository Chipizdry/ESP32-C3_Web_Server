


#ifndef WEBSOCKET_CLIENT_H
#define WEBSOCKET_CLIENT_H

#include "esp_websocket_client.h"
#include "esp_event.h"
#include "esp_log.h"

typedef struct {
    esp_websocket_client_handle_t client;
    bool is_connected;
    char uri[256]; // Для хранения URI WebSocket сервера
} websocket_client_t;

// Инициализация WebSocket клиента
void websocket_client_init(websocket_client_t *ws_client, const char *uri);

// Подключение к WebSocket серверу
void websocket_client_connect(websocket_client_t *ws_client);

// Отправка данных через WebSocket
void websocket_client_send(websocket_client_t *ws_client, const char *data);

// Отключение от сервера
void websocket_client_disconnect(websocket_client_t *ws_client);

// Деинициализация
void websocket_client_cleanup(websocket_client_t *ws_client);

#endif // WEBSOCKET_CLIENT_H