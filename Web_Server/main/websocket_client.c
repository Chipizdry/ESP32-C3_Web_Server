

#include "websocket_client.h"
#include <string.h>

static const char *TAG = "WebSocketClient";

// Обработчик событий WebSocket
static void websocket_event_handler(void *handler_args, esp_event_base_t base, 
                                  int32_t event_id, void *event_data) {
    websocket_client_t *ws_client = (websocket_client_t *)handler_args;
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED:
            ESP_LOGI(TAG, "WebSocket Connected");
            ws_client->is_connected = true;
            break;
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket Disconnected");
            ws_client->is_connected = false;
            break;
            
        case WEBSOCKET_EVENT_DATA:
            ESP_LOGI(TAG, "Received data: %.*s", data->data_len, (char *)data->data_ptr);
            // Здесь можно обрабатывать входящие данные
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket Error");
            ws_client->is_connected = false;
            break;
    }
}

void websocket_client_init(websocket_client_t *ws_client, const char *uri) {
    esp_websocket_client_config_t ws_cfg = {
        .uri = uri,
        .keep_alive_enable = true,
        .keep_alive_idle = 30,
        .keep_alive_interval = 5,
        .keep_alive_count = 3
    };

    ws_client->client = esp_websocket_client_init(&ws_cfg);
    ws_client->is_connected = false;
    strncpy(ws_client->uri, uri, sizeof(ws_client->uri) - 1);
    
    // Регистрация обработчика событий
    esp_websocket_register_events(ws_client->client, WEBSOCKET_EVENT_ANY, 
                                 websocket_event_handler, ws_client);
}

void websocket_client_connect(websocket_client_t *ws_client) {
    if (ws_client->client == NULL) {
        ESP_LOGE(TAG, "WebSocket client not initialized");
        return;
    }
    
    esp_err_t ret = esp_websocket_client_start(ws_client->client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WebSocket client: %s", esp_err_to_name(ret));
    }
}

void websocket_client_send(websocket_client_t *ws_client, const char *data) {
    if (!ws_client->is_connected) {
        ESP_LOGW(TAG, "WebSocket not connected, cannot send data");
        return;
    }
    
    int ret = esp_websocket_client_send_text(ws_client->client, data, strlen(data), portMAX_DELAY);
    if (ret < 0) {
        ESP_LOGE(TAG, "Failed to send WebSocket data");
    }
}

void websocket_client_disconnect(websocket_client_t *ws_client) {
    if (ws_client->client == NULL) {
        return;
    }
    
    esp_websocket_client_stop(ws_client->client);
    ws_client->is_connected = false;
}

void websocket_client_cleanup(websocket_client_t *ws_client) {
    if (ws_client->client == NULL) {
        return;
    }
    
    websocket_client_disconnect(ws_client);
    esp_websocket_client_destroy(ws_client->client);
    ws_client->client = NULL;
}