// websocket_client.c
#include "websocket_client.h"
#include <string.h>
#include "cJSON.h"

static const char *TAG = "WebSocketClient";

// Структура для сообщений
typedef struct {
    char data[512];
} ws_message_t;

// Обработчик событий WebSocket
static void websocket_event_handler(void *handler_args, esp_event_base_t base, 
                                  int32_t event_id, void *event_data) {
    websocket_client_t *ws_client = (websocket_client_t *)handler_args;
    esp_websocket_event_data_t *data = (esp_websocket_event_data_t *)event_data;

    switch (event_id) {
        case WEBSOCKET_EVENT_CONNECTED: {
            ESP_LOGI(TAG, "WebSocket Connected");
            ws_client->is_connected = true;
            
            // Отправляем идентификацию устройства при подключении
            cJSON *root = cJSON_CreateObject();
            cJSON_AddStringToObject(root, "device_id", ws_client->device_id);
            cJSON_AddStringToObject(root, "type", "identify");
            char *json_str = cJSON_PrintUnformatted(root);
            websocket_client_send(ws_client, json_str);
            cJSON_Delete(root);
            free(json_str);
            break;
        }
            
        case WEBSOCKET_EVENT_DISCONNECTED:
            ESP_LOGI(TAG, "WebSocket Disconnected");
            ws_client->is_connected = false;
            break;
            
            case WEBSOCKET_EVENT_DATA: {
                if (data->data_len > 0 && data->data_ptr != NULL) {
                    // Логируем тип фрейма
                    ESP_LOGI(TAG, "WebSocket Frame received: opcode=%d, payload_len=%d, payload (first 20 bytes)='%.*s'",
                             data->op_code, data->data_len,
                             data->data_len > 20 ? 20 : data->data_len, (char *)data->data_ptr);
            
                    switch (data->op_code) {
                        case 0x1: // Текстовое сообщение
                            ESP_LOGI(TAG, "Text frame: %.*s", data->data_len, (char *)data->data_ptr);
                            break;
                        case 0x2: // Бинарное сообщение
                            ESP_LOGI(TAG, "Binary frame (first 20 bytes): %.*s", 
                                     data->data_len > 20 ? 20 : data->data_len, (char *)data->data_ptr);
                            break;
                        case 0x9: // Ping
                            ESP_LOGI(TAG, "Ping frame received");
                            break;
                        case 0xA: // Pong
                            ESP_LOGI(TAG, "Pong frame received");
                            break;
                        case 0x8: // Close
                            ESP_LOGI(TAG, "Close frame received");
                            break;
                        default:
                            ESP_LOGW(TAG, "Unknown frame opcode: %d", data->op_code);
                            break;
                    }
                }
                break;
            }
            break;
            
        case WEBSOCKET_EVENT_ERROR:
            ESP_LOGE(TAG, "WebSocket Error");
            ws_client->is_connected = false;
            break;
    }
}

void websocket_client_init(websocket_client_t *ws_client, const char *uri, const char *device_id) {
    memset(ws_client, 0, sizeof(websocket_client_t));
    
    // Формируем полный URI с device_id
    snprintf(ws_client->uri, sizeof(ws_client->uri), "%s/%s", uri, device_id);
    strncpy(ws_client->device_id, device_id, sizeof(ws_client->device_id) - 1);
    
    // Создаем очереди
    ws_client->out_queue = xQueueCreate(50, sizeof(ws_message_t));
    ws_client->in_queue = xQueueCreate(20, sizeof(ws_message_t));
    
    if (ws_client->out_queue == NULL || ws_client->in_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create queues");
        return;
    }
}

void websocket_client_start_task(websocket_client_t *ws_client) {
    extern const uint8_t _binary_cacert_pem_start[] asm("_binary_cacert_pem_start");
    extern const uint8_t _binary_cacert_pem_end[] asm("_binary_cacert_pem_end");  
    // Конфигурация клиента
    esp_websocket_client_config_t ws_cfg = {
        .uri = ws_client->uri,
        .cert_pem = (const char *) _binary_cacert_pem_start,
        .keep_alive_enable = true,
        .keep_alive_idle = 30,
        .keep_alive_interval = 5,
        .keep_alive_count = 3,
        .reconnect_timeout_ms = 5000,
        .network_timeout_ms = 10000
    };
    

    ws_client->client = esp_websocket_client_init(&ws_cfg);
    if (ws_client->client == NULL) {
        ESP_LOGE(TAG, "Failed to initialize WebSocket client");
        return;
    }
    // Регистрация обработчика событий
    esp_websocket_register_events(ws_client->client, WEBSOCKET_EVENT_ANY, 
                                websocket_event_handler, ws_client);
    
    // Запуск клиента
    esp_err_t ret = esp_websocket_client_start(ws_client->client);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WebSocket client: %s", esp_err_to_name(ret));
    }
}

void websocket_client_send(websocket_client_t *ws_client, const char *data) {
    if (ws_client->out_queue == NULL) return;
    
    ws_message_t msg;
    size_t len = strlen(data) < sizeof(msg.data) - 1 ? strlen(data) : sizeof(msg.data) - 1;
    memcpy(msg.data, data, len);
    msg.data[len] = '\0';
    
    if (xQueueSend(ws_client->out_queue, &msg, pdMS_TO_TICKS(100)) != pdTRUE) {
        ESP_LOGW(TAG, "Outgoing message queue full");
    }
}

bool websocket_client_get_command(websocket_client_t *ws_client, char *buffer, size_t buffer_size) {
    if (ws_client->in_queue == NULL || buffer == NULL || buffer_size == 0) {
        return false;
    }
    
    ws_message_t msg;
    if (xQueueReceive(ws_client->in_queue, &msg, 0) == pdTRUE) {
        size_t len = strlen(msg.data) < buffer_size - 1 ? strlen(msg.data) : buffer_size - 1;
        memcpy(buffer, msg.data, len);
        buffer[len] = '\0';
        return true;
    }
    
    return false;
}

// Задача для работы WebSocket клиента
void websocket_client_task(void *pvParameters) {
    websocket_client_t *ws_client = (websocket_client_t *)pvParameters;
    
    while (1) {
        ESP_LOGD(TAG, "WebSocket task running. Connected: %d", ws_client->is_connected);
        
        if (ws_client->is_connected) {
            // Периодическая отправка данных
            send_periodic_data(ws_client);

           // Проверка и отправка сообщений из очереди
            ws_message_t outgoing_msg;
            while (xQueueReceive(ws_client->out_queue, &outgoing_msg, 0) == pdTRUE) {
                esp_websocket_client_send_text(ws_client->client, outgoing_msg.data, strlen(outgoing_msg.data), portMAX_DELAY);
                ESP_LOGI(TAG, "Sent message from out_queue: %s", outgoing_msg.data);
            }


            // Проверка входящих сообщений
            char command[512];
            if (websocket_client_get_command(ws_client, command, sizeof(command))) {
                ESP_LOGI(TAG, "Received command: %s", command);
                
                // Обработка команды
                // Например, парсинг JSON и выполнение действий
                cJSON *root = cJSON_Parse(command);
                if (root != NULL) {
                    const cJSON *command_type = cJSON_GetObjectItem(root, "command");
                    if (command_type != NULL && cJSON_IsString(command_type)) {
                        if (strcmp(command_type->valuestring, "reset") == 0) {
                            ESP_LOGI(TAG, "Reset command received.");
                            // Выполнить команду сброса устройства
                        }
                    }
                    cJSON_Delete(root);
                }
            }
        }

        if (!ws_client->is_connected) {
            ESP_LOGI(TAG, "Reconnecting WebSocket...");
            vTaskDelay(pdMS_TO_TICKS(10000));
            esp_websocket_client_stop(ws_client->client);
            esp_websocket_client_destroy(ws_client->client);
            websocket_client_start_task(ws_client);
        }

        vTaskDelay(pdMS_TO_TICKS(5000));  // Пауза перед следующей отправкой
    }
}


void send_periodic_data(websocket_client_t *ws_client) {
    cJSON *root = cJSON_CreateObject();
    cJSON_AddStringToObject(root, "device_id", ws_client->device_id);
    cJSON_AddStringToObject(root, "type", "status");
    cJSON_AddNumberToObject(root, "temperature", 25.5);  // Пример данных
    cJSON_AddNumberToObject(root, "humidity", 60);       // Пример данных 

    char *json_str = cJSON_PrintUnformatted(root);
    websocket_client_send(ws_client, json_str);
    ESP_LOGI(TAG, "Sent periodic data: %s", json_str);

    cJSON_Delete(root);
    free(json_str);
}