#include <stdbool.h>
#include <stdint.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_ota_ops.h>
#include <esp_err.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <nvs_flash.h>
#include "esp_partition.h"
#include "esp_littlefs.h"
#include "dirent.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_mac.h"
#include "hal/uart_types.h"
#include "nvs.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "esp_intr_alloc.h"
#include <time.h>
#include "lwip/ip4_addr.h"
#include "soc/gpio_num.h"
#include "uart.h"

// Статический IP и параметры Wi-Fi AP
#define AP_SSID_PREFIX "FlyWheel-"  // Префикс для SSID
#define AP_PASS "12345678"
#define AP_CHANNEL 1
#define AP_MAX_CONN 8
#define AP_IP "192.168.1.1"
#define AP_GW "192.168.1.1"
#define AP_NETMASK "255.255.255.0"
#define STORAGE_NAMESPACE "storage"

#define MAX_CLIENTS 5  // Максимальное количество одновременных подключений

#define BOUNDARY_SUFFIX "\r\n"

#define BOUNDARY_PREFIX "------WebKitFormBoundary"

#define RESET_PIN GPIO_NUM_0  // Выберите нужный пин, например, GPIO 0

// Размер очереди
//#define QUEUE_SIZE 10
httpd_handle_t server_handle = NULL;

static bool netif_initialized = false;
static bool event_loop_created = false;
static bool filesystem_mounted = false;

static int total_size=0;
esp_ota_handle_t ota_handle = 0;
static bool ota_started = false;
static int total_received = 0;  // Общее количество полученных байт
static int32_t rssi=0;
 
static uint8_t request[64];  // Буфер для запроса 

static const char *TAG = "web_server";
// Структура для хранения всех настроек
typedef struct {
    float max_current;  // Максимальный ток
    int voltage_limit;  // Пороговое напряжение
    int speed_limit;    // Ограничение скорости
    char wifi_ssid[16]; // SSID Wi-Fi
    char wifi_password[16]; // Пароль Wi-Fi
    char wifi_ap_ssid[16]; // SSID точки доступа AP
    char wifi_ap_password[16]; // Пароль точки доступа AP
    char wifi_mode[8];   // Режим работы Wi-Fi ("STA" или "AP")
    char ap_ip[16];      // IP адрес точки доступа
    char ap_netmask[16]; // Маска подсети
    char ap_gateway[16]; // Шлюз
} device_settings_t;

// Значения по умолчанию для настроек
device_settings_t default_settings = {
    .max_current = 1.0,
    .voltage_limit = 12,
    .speed_limit = 1500,
  //  .wifi_ssid = "Medical",
  //  .wifi_password = "0445026833",
    .wifi_ssid = "TP-Link_FA4F",
    .wifi_password = "19481555",
    .wifi_ap_ssid = "TP-Link_FA4F",
    .wifi_ap_password = "19481555",
    .wifi_mode = "STA",
    .ap_ip = "192.168.1.1",
    .ap_netmask = "255.255.255.0",
    .ap_gateway = "192.168.1.1"
};


typedef struct {
    char token[33];    // Токен
    bool is_active;    // Флаг активности
} client_token_t;

client_token_t clients[MAX_CLIENTS];
// Структура для данных
typedef struct {
    float voltage[3];  // Массив для трех фаз напряжения
    float current[3];  // Массив для трех фаз тока
    float total_load;  // Суммарная нагрузка
    int speed;         // Скорость
    float temperature; // Температура
    int signal_strength; // Сигнал (RSSI)
} sensor_data_t;

//static QueueHandle_t uart_command_queue;

// Объявляем мьютекс
SemaphoreHandle_t uart_mutex;

uint8_t rx_buffer[RX_BUFFER_SIZE];
void reset_to_factory_settings();
void init_wifi_ap();

// Инициализация пина для сброса
void init_reset_pin() {
    gpio_config_t io_conf = {};
    io_conf.intr_type = GPIO_INTR_DISABLE;  // Прерывания не нужны
    io_conf.mode = GPIO_MODE_INPUT;         // Режим ввода
    io_conf.pin_bit_mask = (1ULL << RESET_PIN); // Пин для отслеживания
    io_conf.pull_up_en = GPIO_PULLUP_ENABLE; // Включаем внутреннюю подтяжку
    gpio_config(&io_conf);
}




// Задача для отслеживания состояния пина
void reset_task(void *arg) {
    while (1) {
        // Проверяем состояние пина
        int pin_state = gpio_get_level(RESET_PIN);

        // Если пин замкнут (например, на землю)
        if (pin_state == 0) {
            ESP_LOGI("RESET", "Reset pin activated. Resetting settings...");
            reset_to_factory_settings();  // Вызываем функцию сброса настроек
            esp_restart();  // Перезагружаем устройство
        }
        vTaskDelay(pdMS_TO_TICKS(100));  // Проверяем каждые 100 мс
    }
}


void init_nvs() {
    ESP_LOGI("NVS", "Initializing NVS...");

    esp_err_t ret = nvs_flash_init();

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES) {
        ESP_LOGW("NVS", "NVS Flash has no free pages. Erasing and reinitializing...");
    } else if (ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_LOGW("NVS", "NVS Flash has a new version. Erasing and reinitializing...");
    }

    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }

    if (ret == ESP_OK) {
        ESP_LOGI("NVS", "NVS successfully initialized.");
    } else {
        ESP_LOGE("NVS", "Error initializing NVS: %s", esp_err_to_name(ret));
    }

    ESP_ERROR_CHECK(ret);
}



esp_err_t save_settings_to_nvs(const device_settings_t *settings) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
        return err;
    }

    // Сохраняем всю структуру как blob (бинарные данные)
    err = nvs_set_blob(nvs_handle, "device_settings", settings, sizeof(device_settings_t));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving settings to NVS: %s", esp_err_to_name(err));
        nvs_close(nvs_handle);
        return err;
    }

    // Подтверждаем изменения
    err = nvs_commit(nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error commiting changes to NVS: %s", esp_err_to_name(err));
    }

    nvs_close(nvs_handle);
    return err;
}



esp_err_t load_settings_from_nvs(device_settings_t *settings) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);  // Открываем NVS с правами на чтение и запись
    if (err != ESP_OK) {
        if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_LOGW(TAG, "NVS is corrupted or needs re-initialization. Erasing and re-initializing...");
            ESP_ERROR_CHECK(nvs_flash_erase());  // Очищаем NVS
            ESP_ERROR_CHECK(nvs_flash_init());   // Переинициализация
            err = nvs_open("storage", NVS_READWRITE, &nvs_handle);  // Пробуем открыть снова
            if (err != ESP_OK) {
                ESP_LOGE(TAG, "Error opening NVS after re-initialization: %s", esp_err_to_name(err));
                // Если всё ещё не удаётся открыть, то записываем настройки по умолчанию
                ESP_LOGI(TAG, "Writing default settings to NVS due to failed re-initialization.");
                memcpy(settings, &default_settings, sizeof(device_settings_t));
                save_settings_to_nvs(settings);
                return ESP_ERR_NVS_NOT_FOUND;  // Возвращаем ошибку, что не удалось открыть и прочитать
            }
        } else {
            ESP_LOGE(TAG, "Error opening NVS: %s", esp_err_to_name(err));
            // Если произошла другая ошибка открытия, записываем настройки по умолчанию
            ESP_LOGI(TAG, "Writing default settings to NVS.");
            memcpy(settings, &default_settings, sizeof(device_settings_t));
            save_settings_to_nvs(settings);
            return ESP_ERR_NVS_NOT_FOUND;
        }
    }

    // Чтение настроек из NVS
    size_t required_size = sizeof(device_settings_t);
    err = nvs_get_blob(nvs_handle, "device_settings", settings, &required_size);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Settings not found, writing default settings to NVS.");
        memcpy(settings, &default_settings, sizeof(device_settings_t));  // Используем значения по умолчанию
        save_settings_to_nvs(settings);  // Записываем в NVS
    } else if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error loading settings from NVS: %s", esp_err_to_name(err));
        return err;
    } else {
        ESP_LOGI(TAG, "User settings successfully loaded from NVS.");
    }

    nvs_close(nvs_handle);  // Закрываем NVS

    // Логируем текущие настройки
    ESP_LOGI(TAG, "Settings after load:");
    //ESP_LOGI(TAG, "Max current: %.2f", settings->max_current);
   // ESP_LOGI(TAG, "Voltage limit: %d", settings->voltage_limit);
   // ESP_LOGI(TAG, "Speed limit: %d", settings->speed_limit);
    ESP_LOGI(TAG, "Wi-Fi STA SSID: %s", settings->wifi_ssid);
    ESP_LOGI(TAG, "Wi-Fi STA Password: %s", settings->wifi_password);
   // ESP_LOGI(TAG, "Wi-Fi AP SSID: %s", settings->wifi_ap_ssid);
  //  ESP_LOGI(TAG, "Wi-Fi AP Password: %s", settings->wifi_ap_password);
    ESP_LOGI(TAG, "Wi-Fi Mode: %s", settings->wifi_mode);
    ESP_LOGI(TAG, "AP IP: %s", settings->ap_ip);
  //  ESP_LOGI(TAG, "AP Netmask: %s", settings->ap_netmask);
  //  ESP_LOGI(TAG, "AP Gateway: %s", settings->ap_gateway);

    return ESP_OK;
}



void setup_random() {
    // Инициализация генератора случайных чисел
    srand(time(NULL));
}

void list_files(const char *base_path);

void wifi_signal_strength_task(void *pvParameters) {
    while (true) {
        wifi_ap_record_t ap_info;
        if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
            // Получаем уровень сигнала в dBm
           rssi = ap_info.rssi;
            ESP_LOGI(TAG, "Current WiFi Signal Strength: %ld dBm",(long int)rssi);
        } else {
            ESP_LOGE(TAG, "Failed to get AP info");
        }
        vTaskDelay(pdMS_TO_TICKS(3000));
    }
}

void mount_littlefs() {
    esp_vfs_littlefs_conf_t conf = {
        .base_path = "/littlefs",
        .partition_label = "storage",
        .format_if_mount_failed = true,
        .dont_mount = false,
    };

    esp_err_t ret = esp_vfs_littlefs_register(&conf);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to mount LittleFS (%s)", esp_err_to_name(ret));
        return;
    }

    size_t total = 0, used = 0;
    ret = esp_littlefs_info("storage", &total, &used);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get LittleFS partition information (%s)", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "LittleFS partition size: total: %d, used: %d", total, used);
    }
}


void list_files(const char *base_path) {
    // Открываем каталог
    DIR *dir = opendir(base_path);
    if (dir == NULL) {
        ESP_LOGE(TAG, "Failed to open directory %s", base_path);
        return;
    }

    struct dirent *entry;
    while ((entry = readdir(dir)) != NULL) {
        ESP_LOGI(TAG, "Found file: %s", entry->d_name);
    }

    closedir(dir);
}

// Пример функции для печати IP-адреса
void print_ip_info() {
    esp_netif_ip_info_t ip_info;
    esp_netif_t* netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");

    if (netif == NULL) {
        ESP_LOGE(TAG, "Failed to get netif handle for STA");
        return;
    }

    esp_err_t ret = esp_netif_get_ip_info(netif, &ip_info);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to get IP info: %s", esp_err_to_name(ret));
        return;
    }

    if (ip_info.ip.addr == 0) {
        ESP_LOGW(TAG, "No valid IP address assigned yet (IP = 0.0.0.0)");
        return;
    }

    ESP_LOGI(TAG, "IP Address: " IPSTR, IP2STR(&ip_info.ip));
    ESP_LOGI(TAG, "Netmask: " IPSTR, IP2STR(&ip_info.netmask));
    ESP_LOGI(TAG, "Gateway: " IPSTR, IP2STR(&ip_info.gw));
}


esp_netif_t* netif_sta = NULL;  // Интерфейс для Station
esp_netif_t* netif_ap = NULL;   // Интерфейс для Access Point


// Функция остановки веб-сервера
void stop_webserver() {
    if (server_handle != NULL) {
        ESP_LOGI(TAG, "Stopping web server...");
        // Остановка сервера и освобождение ресурсов
        httpd_stop(server_handle);
        server_handle = NULL;
    }
}




// Функция для остановки Wi-Fi, если он был запущен
void check_and_stop_wifi() {
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Wi-Fi mode: %d", mode);
        if (mode != WIFI_MODE_NULL) {
            ESP_LOGI(TAG, "Stopping Wi-Fi...");
            esp_wifi_stop();  // Останавливаем Wi-Fi
            esp_wifi_deinit();  // Освобождаем ресурсы Wi-Fi
            ESP_LOGI(TAG, "Wi-Fi stopped.");
        } else {
            ESP_LOGI(TAG, "Wi-Fi is not running.");
        }
    } else {
        ESP_LOGE(TAG, "Failed to get Wi-Fi mode: %s", esp_err_to_name(err));
    }
}


 

// Функция для определения MIME-типа по расширению файла
const char* get_mime_type(const char* path) {
    const char* ext = strrchr(path, '.');
    if (!ext) {
        return "text/plain";
    }
    if (strcmp(ext, ".html") == 0) return "text/html";
    if (strcmp(ext, ".css") == 0) return "text/css";
    if (strcmp(ext, ".js") == 0) return "application/javascript";
    if (strcmp(ext, ".png") == 0) return "image/png";
    if (strcmp(ext, ".jpg") == 0 || strcmp(ext, ".jpeg") == 0) return "image/jpeg";
    if (strcmp(ext, ".gif") == 0) return "image/gif";
    if (strcmp(ext, ".bin") == 0) return "application/octet-stream"; 
    if (strcmp(ext, ".svg") == 0) return "image/svg+xml";
    // Добавьте другие типы по необходимости
    return "text/plain";
}



httpd_handle_t start_webserver(void);

// Функция для перезапуска сервера
void restart_webserver(void) {
    ESP_LOGI(TAG, "Restarting web server...");
    stop_webserver();  // Останавливаем текущий сервер
    start_webserver(); // Запускаем сервер заново
}

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    static bool ap_mode_started = false;
   

    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to Wi-Fi (STA mode)");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected from Wi-Fi (STA mode)");
                esp_wifi_connect();  // Повторная попытка подключения
                break;
            case WIFI_EVENT_AP_START:
                ESP_LOGI(TAG, "Wi-Fi AP started");
                if (!ap_mode_started) {
                    ap_mode_started = true;
                }
                break;
            case WIFI_EVENT_AP_STOP:
                ESP_LOGI(TAG, "Wi-Fi AP stopped");
                if (ap_mode_started) {
                    stop_webserver();  // Остановка веб-сервера в режиме AP
                    ap_mode_started = false;
                }
                break;
            case WIFI_EVENT_AP_STACONNECTED: {
                wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
                ESP_LOGI(TAG, "Client connected to AP. MAC: "MACSTR, MAC2STR(event->mac));
                break;
            }
            case WIFI_EVENT_AP_STADISCONNECTED: {
                wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
                ESP_LOGI(TAG, "Client disconnected from AP. MAC: "MACSTR, MAC2STR(event->mac));
                break;
            }
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP: {
                ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
                ESP_LOGI(TAG, "Got IP: " IPSTR, IP2STR(&event->ip_info.ip));
                break;
            }
            default:
                break;
        }
    }
}

esp_err_t get_ip_handler(httpd_req_t *req) {
    char response[256];  //  буфер для ответа
    esp_netif_ip_info_t ip_info;
    wifi_mode_t mode;
    wifi_config_t wifi_config;
    esp_err_t err = esp_wifi_get_mode(&mode);

    ESP_LOGI("IP_HANDLER", "Received request for URI: %s", req->uri);

    // Получаем User-Agent для логирования (можно удалить, если не требуется)
    size_t header_count = httpd_req_get_hdr_value_len(req, "User-Agent") + 1;
    if (header_count > 1) {
        char *header_value = malloc(header_count);
        if (httpd_req_get_hdr_value_str(req, "User-Agent", header_value, header_count) == ESP_OK) {
            ESP_LOGI("IP_HANDLER", "User-Agent: %s", header_value);
        }
        free(header_value);
    }

    if (err != ESP_OK) {
        ESP_LOGE("IP_HANDLER", "Failed to get Wi-Fi mode: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    esp_netif_t *netif = (mode == WIFI_MODE_STA) ? esp_netif_get_handle_from_ifkey("WIFI_STA_DEF")
                                                 : esp_netif_get_handle_from_ifkey("WIFI_AP_DEF");

    if (netif == NULL) {
        ESP_LOGE("IP_HANDLER", "Failed to get netif handle");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    err = esp_netif_get_ip_info(netif, &ip_info);
    if (err != ESP_OK) {
        ESP_LOGE("IP_HANDLER", "Failed to get IP info: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    // Получаем текущую конфигурацию Wi-Fi
    err = esp_wifi_get_config((mode == WIFI_MODE_STA) ? WIFI_IF_STA : WIFI_IF_AP, &wifi_config);
    if (err != ESP_OK) {
        ESP_LOGE("IP_HANDLER", "Failed to get Wi-Fi config: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }
      // Убедимся, что SSID завершен нулевым байтом
    wifi_config.ap.ssid[sizeof(wifi_config.ap.ssid) - 1] = '\0';
    wifi_config.sta.ssid[sizeof(wifi_config.sta.ssid) - 1] = '\0';
    
     // Логируем SSID и пароль
    ESP_LOGI("IP_HANDLER", "Wi-Fi mode: %s", (mode == WIFI_MODE_AP) ? "AP" : "STA");
    ESP_LOGI("IP_HANDLER", "SSID: %s", (mode == WIFI_MODE_AP) ? (char *)wifi_config.ap.ssid : (char *)wifi_config.sta.ssid);
    
  // Формируем ответ в зависимости от режима работы
    if (mode == WIFI_MODE_AP) {
        snprintf(response, sizeof(response),
                 "{\"ip\": \"" IPSTR "\", \"netmask\": \"" IPSTR "\", \"gateway\": \"" IPSTR "\", \"mode\": \"AP\", \"ssid\": \"%s\", \"password\": \"%s\"}",
                 IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw),
                 (char *)wifi_config.ap.ssid, (char *)wifi_config.ap.password);
    } else if (mode == WIFI_MODE_STA) {
        snprintf(response, sizeof(response),
                 "{\"ip\": \"" IPSTR "\", \"netmask\": \"" IPSTR "\", \"gateway\": \"" IPSTR "\", \"mode\": \"STA\", \"ssid\": \"%s\"}",
                 IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw),
                 (char *)wifi_config.sta.ssid);
    }

    httpd_resp_set_type(req, "application/json");
    esp_err_t res = httpd_resp_send(req, response, strlen(response));
    if (res != ESP_OK) {
        ESP_LOGE("IP_HANDLER", "Failed to send response: %s", esp_err_to_name(res));
    }
//send_ip_config_command();
    return res;
}

esp_err_t wifi_mode_handler(httpd_req_t *req) {
    char buf[100];
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1) {
        ESP_LOGW(TAG, "Request body is too large");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    ESP_LOGI(TAG, "Received Wi-Fi mode request: %s", buf);

    // Загружаем текущие настройки из NVS
    device_settings_t settings;
    esp_err_t err = load_settings_from_nvs(&settings);
    if (err != ESP_OK && err != ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGE(TAG, "Error loading settings: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    // Определяем, какой режим был запрошен (например, "STA" или "AP")
    if (strstr(buf, "STA") != NULL) {
        ESP_LOGI(TAG, "Switching to STA mode");
        strncpy(settings.wifi_mode, "STA", sizeof(settings.wifi_mode));
        esp_wifi_set_mode(WIFI_MODE_STA);
    } else if (strstr(buf, "AP") != NULL) {
        ESP_LOGI(TAG, "Switching to AP mode");
        strncpy(settings.wifi_mode, "AP", sizeof(settings.wifi_mode));
        init_wifi_ap();  // Вызов функции настройки AP
    } else {
        ESP_LOGW(TAG, "Unknown mode requested: %s", buf);
        httpd_resp_send_500(req); // Неправильный запрос
        return ESP_FAIL;
    }

    // Сохраняем обновленные настройки в NVS
    err = save_settings_to_nvs(&settings);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Error saving settings to NVS: %s", esp_err_to_name(err));
        httpd_resp_send_500(req);
        return err;
    }

    httpd_resp_send(req, "Wi-Fi mode updated", HTTPD_RESP_USE_STRLEN);
    esp_restart();
    return ESP_OK;
}


  void init_wifi_ap() {
		    
		    
		    
		    check_and_stop_wifi();
		// Инициализация netif
		ESP_LOGI(TAG, "Initializing network interface...");
		
		// Создание Wi-Fi интерфейса для AP
		esp_netif_t *netif = esp_netif_create_default_wifi_ap();
		assert(netif != NULL);
		
		// Инициализируем конфигурацию Wi-Fi
		wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
		ESP_ERROR_CHECK(esp_wifi_init(&cfg));
		
		// Остановка DHCP сервера перед установкой статического IP
		ESP_ERROR_CHECK(esp_netif_dhcps_stop(netif));
		
		// Настройка статического IP для AP
		esp_netif_ip_info_t ip_info;
		ip4addr_aton(AP_IP, &ip_info.ip);
		ip4addr_aton(AP_GW, &ip_info.gw);
		ip4addr_aton(AP_NETMASK, &ip_info.netmask);
		ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));
		
		ESP_LOGI(TAG, "Setting static IP Config - IP: %s, Gateway: %s, Netmask: %s", AP_IP, AP_GW, AP_NETMASK);
		
		// Запуск DHCP сервера
        ESP_ERROR_CHECK(esp_netif_dhcps_start(netif));
		
		// Получение уникального SSID из MAC-адреса
		uint8_t mac[6];
		ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_SOFTAP));
	//	ESP_ERROR_CHECK(esp_read_mac(mac, ESP_MAC_WIFI_STA));
		char unique_id[7];
		snprintf(unique_id, sizeof(unique_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);
		
		// Формирование полного SSID
		char ssid[16];
		snprintf(ssid, sizeof(ssid), "%s%s", AP_SSID_PREFIX, unique_id);
		
		// Настройка Wi-Fi AP
		wifi_config_t wifi_config = {
		    .ap = {
		        .ssid = {0},
		        .ssid_len = strlen(ssid),
		        .password = AP_PASS,
		        .channel = AP_CHANNEL,
		        .max_connection = AP_MAX_CONN,
		        .authmode = WIFI_AUTH_WPA_WPA2_PSK,
		    },
		};
		strcpy((char *)wifi_config.ap.ssid, ssid);
		
		if (strlen(AP_PASS) == 0) {
		    wifi_config.ap.authmode = WIFI_AUTH_OPEN;
		}
		
		ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));  // Устанавливаем режим AP
		ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));  // Применяем настройки AP
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_START, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STOP, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STACONNECTED, &wifi_event_handler, NULL));
        ESP_ERROR_CHECK(esp_event_handler_register(WIFI_EVENT, WIFI_EVENT_AP_STADISCONNECTED, &wifi_event_handler, NULL));

		ESP_ERROR_CHECK(esp_wifi_start());  // Запускаем Wi-Fi
		
		ESP_LOGI(TAG, "Wi-Fi AP Mode Initialized. SSID: %s, Password: %s", ssid, AP_PASS);
		ESP_LOGI(TAG, "AP IP Address: %s", AP_IP);
   }

void init_wifi_sta(const device_settings_t *settings) {
    // Остановим любые текущие Wi-Fi соединения перед запуском STA
    check_and_stop_wifi();

    // Инициализация Wi-Fi интерфейса
    ESP_LOGI(TAG, "Initializing network interface for STA mode...");

    // Создание Wi-Fi интерфейса для STA
    esp_netif_t *netif = esp_netif_create_default_wifi_sta();
    assert(netif != NULL);

    // Инициализация Wi-Fi с помощью стандартной конфигурации
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    // Настройка Wi-Fi конфигурации для STA
    wifi_config_t wifi_config = {
        .sta = {
            .ssid = {0},
            .password = {0},
        },
    };

    // Копируем SSID и пароль из настроек
    strncpy((char *)wifi_config.sta.ssid, settings->wifi_ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, settings->wifi_password, sizeof(wifi_config.sta.password));
    // Устанавливаем режим STA
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    
    // Применяем конфигурацию STA
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config));
      // Регистрация обработчиков событий Wi-Fi для STA
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);


    // Запускаем Wi-Fi
    ESP_ERROR_CHECK(esp_wifi_start());

    ESP_LOGI(TAG, "Wi-Fi STA Mode Initialized. Connecting to SSID: %s", settings->wifi_ssid);
}

void init_wifi_from_nvs() {
    device_settings_t current_settings;
    
    // Загружаем настройки из NVS
    load_settings_from_nvs(&current_settings);
    
    // Определяем режим Wi-Fi из настроек
    if (strcmp(current_settings.wifi_mode, "AP") == 0) {
        ESP_LOGI("init_wifi_from_nvs", "Initializing Wi-Fi in AP mode...");
        init_wifi_ap(&current_settings);  // Инициализируем AP с использованием настроек
    } else {
        ESP_LOGI("init_wifi_from_nvs", "Initializing Wi-Fi in STA mode...");
        init_wifi_sta(&current_settings);  // Ваш код для инициализации STA
    }
}
// Функция для сброса настроек на заводские
void reset_to_factory_settings() {
    esp_err_t err;

    // Очищаем NVS
    err = nvs_flash_erase();  // Очищаем всю память NVS
    if (err != ESP_OK) {
        ESP_LOGE("NVS", "Error erasing NVS: %s", esp_err_to_name(err));
    } else {
        ESP_LOGI("NVS", "NVS erased successfully");
    }
    // Применение сохраненных настроек (опционально)
    save_settings_to_nvs(&default_settings);
}


const esp_partition_t *update_partition = NULL;


// Функция для получения строкового представления метода
const char* get_method_string(httpd_method_t method) {
    switch (method) {
        case HTTP_GET: return "GET";
        case HTTP_POST: return "POST";
        case HTTP_PUT: return "PUT";
        case HTTP_DELETE: return "DELETE";
        // Добавьте другие методы по мере необходимости
        default: return "UNKNOWN";
    }
}
 
esp_err_t ota_post_handler(httpd_req_t *req) {
    char buffer[4096]; // Буфер для получения данных
    int received = 0;  // Количество полученных байт
   
    char content_type[256] = {0};
    char *boundary = NULL;
   
   	 static esp_ota_handle_t ota_handle=0; // Делаем её static или глобальной

    esp_err_t ret;
    if(ota_started==false){
	

    const esp_partition_t *ota_partition = esp_ota_get_next_update_partition(NULL);
   
    ESP_LOGI(TAG, "Found OTA partition: %s, size: %lu", ota_partition->label, (long unsigned int)ota_partition->size);
    if (ota_partition == NULL) {
        ESP_LOGE(TAG, "OTA partition not found");
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA partition not found");
        return ESP_FAIL;
    }

    ret = esp_ota_begin(ota_partition, OTA_SIZE_UNKNOWN, &ota_handle);
    ota_started=true;
    if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_ota_begin failed! err = %d", ret);
    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_begin failed");
    return ESP_FAIL;
    } else {
    ESP_LOGI(TAG, "esp_ota_begin succeeded, OTA handle initialized.");
      }
    }
    
     const esp_partition_t *running_partition = esp_ota_get_running_partition();
  

    ESP_LOGI(TAG, "Request method: %s", get_method_string(req->method));
    ESP_LOGI(TAG, "Request URI: %s", req->uri);
    ESP_LOGI(TAG, "Content Length: %d", req->content_len);
    ESP_LOGI(TAG, "Running partition: %s", running_partition->label);
    // Читаем данные из запроса
httpd_req_get_hdr_value_str(req, "Content-Type", content_type, sizeof(content_type));

// Получаем заголовок Content-Type  
ESP_LOGI(TAG, "Received Content-Type: %s", content_type);

// Находим boundary в заголовке Content-Type
boundary = strstr(content_type, "boundary=");
if (!boundary) {
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "No boundary found");
    return ESP_FAIL;
}
boundary += strlen("boundary="); // Пропускаем "boundary="
ESP_LOGI(TAG, "Boundary: %s", boundary);

// Читаем тело запроса и обрабатываем чанки

while ((received = httpd_req_recv(req, buffer, sizeof(buffer) - 1)) > 0) {
    buffer[received] = '\0';  // Завершаем строку для безопасности
 
    char *data_start = buffer;
    char *boundary_pos = strstr(data_start, boundary);
    
   // Обрабатываем части, пока находим boundary
while (boundary_pos) {
    // Пропускаем boundary и ищем следующий блок данных
    data_start = boundary_pos + strlen(boundary);
    size_t total_length = received - (data_start - buffer);
    // Ищем заголовок Content-Disposition
 //   char *content_disp = strstr(data_start, "Content-Disposition");
    char *content_disp = memmem(data_start, total_length, "Content-Disposition", strlen("Content-Disposition"));
      ESP_LOGI(TAG, "Content-Disposition:%s",content_disp);  // Выводим данные как строку
    if (!content_disp) {
        ESP_LOGE(TAG, "Content-Disposition not found");
        break; // Если не найден Content-Disposition, завершаем
    }
		// Находим "name=\"totalSize\""
		char *total_size_start = memmem(data_start, total_length, "name=\"totalSize\"", strlen("name=\"totalSize\""));
		if (!total_size_start) {
		    ESP_LOGE(TAG, "Total size field not found");
		    break;
		}
		
		// Смещаем указатель до конца строки "name=\"totalSize\""
		total_size_start += strlen("name=\"totalSize\"");
		
		// Пропускаем любые пробелы, символы новой строки или символы перевода строки
		while (*total_size_start == '\n' || *total_size_start == '\r' || *total_size_start == ' ') {
		    total_size_start++;
		}
		
		// Теперь total_size_start указывает на значение totalSize, считываем его
		char total_size_str[16] = {0}; // Для хранения значения totalSize
		int i = 0;
		while (*total_size_start >= '0' && *total_size_start <= '9' && i < sizeof(total_size_str) - 1) {
		    total_size_str[i++] = *total_size_start++;
		}
		total_size_str[i] = '\0'; // Завершаем строку
		
		// Преобразуем строку в целое число
		total_size = atoi(total_size_str);
		
		ESP_LOGI(TAG, "Total Size: %d", total_size);
     // Ищем конец заголовка Content-Disposition
    char *content_end = memmem(content_disp, total_length - (content_disp - data_start), "application/octet-stream", strlen("application/octet-stream"));
   
    if (!content_end) {
        ESP_LOGE(TAG, "End of Content-Disposition not found");
        break;
    }
    ESP_LOGI(TAG, "End of Content-Disposition found");

    // Устанавливаем конец заголовка
    content_end += strlen("application/octet-stream"); // Пропускаем "application/octet-stream"

    // Находим начало данных (после заголовка)
    char *data_start = content_end + 4; // Пропускаем следующий символ (который, скорее всего, '\n' или '\r')
   
    // Находим следующую границу (boundary) для определения конца данных
    
    size_t data_length = received - (data_start - buffer); // Остаток данных
    char *boundary_position = (char*)memmem(data_start, total_length, boundary, strlen(boundary));

     // Если граница найдена
    if (boundary_position) {
        data_length = boundary_position - data_start-4;  // Длина данных до границы
        ESP_LOGI(TAG, "Binary data length: %d bytes", data_length);

        // Выводим двоичные данные
      //  ESP_LOG_BUFFER_HEX(TAG, data_start, data_length);  // Выводим двоичные данные в лог
           // Проверка инициализации OTA
          //  if (ota_handle == NULL) { 
            if (ota_handle ==0) {
            ESP_LOGE(TAG, "OTA handle is null, something went wrong with esp_ota_begin.");
              return ESP_FAIL;}



        // Записываем данные в OTA
                ret = esp_ota_write(ota_handle, data_start, data_length);
                if (ret != ESP_OK) {
                    ESP_LOGE(TAG, "esp_ota_write failed! err = %d", ret);
                    esp_ota_end(ota_handle); // Завершаем OTA
                    httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "esp_ota_write failed");
                    return ESP_FAIL;
                }
        
        
    } else {
        ESP_LOGE(TAG, "Boundary not found");
        break;  // Завершаем, если не найдено следующее boundary
    }
    
    total_received +=data_length;
     ESP_LOGI(TAG, "OTA update: %d bytes", total_received);
     // Выход из внутреннего цикла при нахождении границы
            break;
   
}
// Очищаем буфер после обработки
    memset(buffer, 0, sizeof(buffer));
}
   
    if (received == 0) {	
    // Обработка завершена, данные полностью переданы
    httpd_resp_set_type(req, "text/plain");
    httpd_resp_send(req, "OTA update completed", HTTPD_RESP_USE_STRLEN);
}
     if( total_received==total_size){
  // Обработка завершена, данные полностью переданы
       esp_err_t end_err = esp_ota_end(ota_handle);
    if (end_err != ESP_OK) {
        httpd_resp_send_err(req, HTTPD_500_INTERNAL_SERVER_ERROR, "OTA end failed");
        ESP_LOGI(TAG, "OTA Update ERROR.....");
    } else {
        httpd_resp_send(req, "OTA Update Success", HTTPD_RESP_USE_STRLEN);
          ESP_LOGI(TAG, "OTA Update Success.Now rebooting.....");
  // Определяем следующий раздел для загрузки
    const esp_partition_t *next_partition = (running_partition->subtype == ESP_PARTITION_SUBTYPE_APP_OTA_0) ?
                                            esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_1, NULL) :
                                            esp_partition_find_first(ESP_PARTITION_TYPE_APP, ESP_PARTITION_SUBTYPE_APP_OTA_0, NULL);

    if (next_partition != NULL) {
        ESP_LOGI(TAG, "Next partition: %s", next_partition->label);

        // Устанавливаем новый раздел для загрузки при следующем перезапуске
        esp_err_t err = esp_ota_set_boot_partition(next_partition);
        if (err == ESP_OK) {
            ESP_LOGI(TAG, "Set next partition as boot");
        } else {
            ESP_LOGE(TAG, "Failed to set boot partition: %s", esp_err_to_name(err));
        }

        // Перезагрузка для загрузки нового приложения
        esp_restart();
    } else {
        ESP_LOGE(TAG, "Next partition not found!");
    }  esp_restart();
    }}
    return ESP_OK;
    
}




esp_err_t save_wifi_settings_handler(httpd_req_t *req) {
    // Буфер для приема данных
    char buf[100];
    int ret, remaining = req->content_len;

    // Буфер для хранения SSID и пароля
    char ssid[16] = {0};
    char password[16] = {0};

    // Читаем содержимое POST-запроса
    while (remaining > 0) {
        // Чтение запроса в буфер
        ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf)));
        if (ret <= 0) {
            // Ошибка при получении данных
            if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
                httpd_resp_send_408(req);
            }
            return ESP_FAIL;
        }

        // Обработка содержимого запроса
        remaining -= ret;
        buf[ret] = '\0'; // Добавляем нулевой символ для завершения строки

        // Примерный парсинг данных (зависит от формата, как их передает фронтенд)
        sscanf(buf, "{\"ssid\":\"%31[^\"]\",\"password\":\"%63[^\"]\"}", ssid, password);
    }

    ESP_LOGI(TAG, "Received SSID: %s, Password: %s", ssid, password);

    // Сохраняем настройки Wi-Fi в NVS
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("wifi_config", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        nvs_set_str(nvs_handle, "ssid", ssid);
        nvs_set_str(nvs_handle, "password", password);
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Error opening NVS handle!");
        return ESP_FAIL;
    }

    // Применение новых настроек Wi-Fi
    wifi_config_t wifi_config = {};
    strncpy((char *)wifi_config.sta.ssid, ssid, sizeof(wifi_config.sta.ssid));
    strncpy((char *)wifi_config.sta.password, password, sizeof(wifi_config.sta.password));

    esp_wifi_disconnect();
    esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config);
    esp_wifi_connect();

    // Отправляем ответ клиенту
    httpd_resp_set_type(req, "application/json");
    httpd_resp_sendstr(req, "{\"status\": \"success\"}");

    return ESP_OK;
}

// Обработчик для POST-запросов
esp_err_t post_handler(httpd_req_t *req) {
    char buf[200];  // Увеличиваем размер буфера для дополнительных параметров
    int ret, remaining = req->content_len;

    if (remaining > sizeof(buf) - 1) {
        ESP_LOGW(TAG, "Request body is too large");
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    ret = httpd_req_recv(req, buf, MIN(remaining, sizeof(buf) - 1));
    if (ret <= 0) {
        if (ret == HTTPD_SOCK_ERR_TIMEOUT) {
            httpd_resp_send_408(req);
        }
        return ESP_FAIL;
    }

    buf[ret] = '\0';
    ESP_LOGI(TAG, "Received POST data: %s", buf);
    
    uart_command_t cmd;
    
    // Проверяем, что запрос содержит данные
    if (ret > 0) {
        buf[ret] = 0;  // Завершаем строку

        // Находим параметры для батареи в теле запроса
        char *lowThreshold = strstr(buf, "lowThreshold=");
        char *highThreshold = strstr(buf, "highThreshold=");
        char *maxCurrentRange = strstr(buf, "maxCurrentRange=");

        // Проверяем наличие параметров для батареи
        if (lowThreshold && highThreshold && maxCurrentRange) {
			cmd.cmd_type=1;
            cmd.command = 3;  // Команда для настройки батареи

            // Извлекаем значения и преобразуем их в float
            float low = atof(lowThreshold + strlen("lowThreshold="));	
            float high = atof(highThreshold + strlen("highThreshold="));
            float maxCurrent = atof(maxCurrentRange + strlen("maxCurrentRange="));

            // Кодируем параметры в uint16_t с точностью до 0.1
            uint16_t low_value = (uint16_t)(low * 10);
            uint16_t high_value = (uint16_t)(high * 10);
            uint16_t current_value = (uint16_t)(maxCurrent * 10);

            // Заполняем payload данными
            cmd.payload[0] = (low_value >> 8) & 0xFF;  // Старший байт lowThreshold
            cmd.payload[1] = low_value & 0xFF;         // Младший байт lowThreshold
            cmd.payload[2] = (high_value >> 8) & 0xFF;  // Старший байт highThreshold
            cmd.payload[3] = high_value & 0xFF;         // Младший байт highThreshold
            cmd.payload[4] = (current_value >> 8) & 0xFF;  // Старший байт maxCurrentRange
            cmd.payload[5] = current_value & 0xFF;         // Младший байт maxCurrentRange
            cmd.payload_len = 6;

            // Логируем полученные данные
            ESP_LOGI(TAG, "Battery settings - Low: %.2f, High: %.2f, Max Current: %.2f",
                     low, high, maxCurrent);
        }
        
        // Находим параметры для настройки нагрузки
        char *maxLoad = strstr(buf, "maxLoad=");
        char *outputVoltage = strstr(buf, "outputVoltage=");
        char *maxCurrentDifference = strstr(buf, "maxCurrentDifference=");

        // Проверяем наличие параметров для нагрузки
        if (maxLoad && outputVoltage && maxCurrentDifference) {
			cmd.cmd_type=1;
            cmd.command = 4;  // Команда для настройки нагрузки

            // Извлекаем значения и преобразуем их в float
            float load = atof(maxLoad + strlen("maxLoad="));
            float voltage = atof(outputVoltage + strlen("outputVoltage="));
            float currentDiff = atof(maxCurrentDifference + strlen("maxCurrentDifference="));

            // Кодируем параметры в uint16_t с точностью до 0.1
            uint16_t load_value = (uint16_t)(load * 10);
            uint16_t voltage_value = (uint16_t)(voltage * 10);
            uint16_t current_diff_value = (uint16_t)(currentDiff * 10);

            // Заполняем payload данными
            cmd.payload[0] = (load_value >> 8) & 0xFF;      // Старший байт maxLoad
            cmd.payload[1] = load_value & 0xFF;             // Младший байт maxLoad
            cmd.payload[2] = (voltage_value >> 8) & 0xFF;   // Старший байт outputVoltage
            cmd.payload[3] = voltage_value & 0xFF;          // Младший байт outputVoltage
            cmd.payload[4] = (current_diff_value >> 8) & 0xFF;  // Старший байт maxCurrentDifference
            cmd.payload[5] = current_diff_value & 0xFF;         // Младший байт maxCurrentDifference
            cmd.payload_len = 6;

            // Логируем полученные данные
            ESP_LOGI(TAG, "Load settings - Max Load: %.2f, Output Voltage: %.2f, Max Current Difference: %.2f",
                     load, voltage, currentDiff);
        }

    } else {
        ESP_LOGE(TAG, "Failed to receive POST data");
        return ESP_FAIL;
    }
   
   
     // Отправляем команду в очередь UART
    if (xQueueSend(uart_command_queue, &cmd, 0) != pdPASS) {
        ESP_LOGW(TAG, "Failed to send command to UART queue");
    }

    // Отправляем успешный ответ
    httpd_resp_send(req, "Настройки успешно сохранены", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t data_get_handler(httpd_req_t *req) {
    // Пример данных
   // Генерация примеров данных
    sensor_data_t data;

    float batt_voltage = 40.0 + ((float)(rand() % 101)) / 10.0;  // Генерация напряжения от 10 до 20 В
    data.speed = 1500 + (rand() % 1001);  // Генерация скорости от 1500 до 2500 об/мин
    data.temperature = 20.0 + ((float)(rand() % 301)) / 10.0;  // Генерация температуры от 20 до 50 °C
    float batt_current = 0.5 + ((float)(rand() % 100)) / 100.0;  // Генерация тока от 0.5 до 1.5 А
    
     for (int i = 0; i < 3; i++) {
        data.voltage[i] = 220.0 + ((float)(rand() % 101)) / 10.0;  // Напряжение по каждой фазе
        data.current[i] = 10.0 + ((float)(rand() % 101)) / 10.0;   // Ток по каждой фазе
    }
    data.total_load=30.0 + ((float)(rand() % 101)) / 10.0;
    ESP_LOGI("WEB_SERVER", "Request received");
     // Формирование JSON-ответа
    char response[264];
        snprintf(response, sizeof(response),
             "{\"v_ac_out\": [%.2f, %.2f, %.2f], "
             "\"I_AC_Out\": [%.2f, %.2f, %.2f], "
             "\"total_load\": %.2f,"
             "\"temperature\": %.1f,"
             "\"batt_voltage\":%.2f,"
             "\"batt_curr\":%.2f,"
             "\"speed\": %d,"
             " \"Signal\": %ld}",
             data.voltage[0], data.voltage[1], data.voltage[2],
             data.current[0], data.current[1], data.current[2],
             data.total_load,data.temperature,batt_voltage, batt_current,data.speed , (long int)rssi);

    // Отправка ответа клиенту
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}

// Функция добавления токена клиента
bool add_client_token(const char *token) {
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (!clients[i].is_active) {  // Ищем неактивный слот
            strcpy(clients[i].token, token);  // Копируем токен в массив
            clients[i].is_active = true;      // Отмечаем сессию как активную
            return true;                      // Токен успешно добавлен
        }
    }
    return false;  // Нет свободных слотов для добавления нового токена
}


bool is_max_clients_reached() {
    int active_clients = 0;
    for (int i = 0; i < MAX_CLIENTS; i++) {
        if (clients[i].is_active) {
            active_clients++;
        }
    }
    return active_clients >= MAX_CLIENTS;
}
// Функция для генерации токена (в данном случае простой случайный токен)
char *generate_token() {
    static char token[33];
    for (int i = 0; i < 32; i++) {
        token[i] = 'A' + (rand() % 26);
    }
    token[32] = '\0';
    return token;
}


// Реализация httpd_resp_send_503
esp_err_t httpd_resp_send_503(httpd_req_t *req) {
    httpd_resp_set_status(req, "503 Service Unavailable");  // Устанавливаем статус 503
    httpd_resp_set_type(req, "text/plain");                 // Устанавливаем тип контента
    return httpd_resp_sendstr(req, "Server overloaded, please try again later.");
}

esp_err_t login_handler(httpd_req_t *req) {
    if (is_max_clients_reached()) {
        httpd_resp_send_503(req);  // Ошибка, сервер перегружен
        return ESP_FAIL;
    }

    // Генерация токена и добавление клиента в активные
    char *token = generate_token();
    add_client_token(token);
    httpd_resp_sendstr(req, token);  // Возвращаем токен
    return ESP_OK;
}

esp_err_t file_get_handler(httpd_req_t *req) {
    char filepath[522];
	    if (strcmp(req->uri, "/") == 0) {
	    snprintf(filepath, sizeof(filepath), "/littlefs/index.html");
		} else if (strcmp(req->uri, "/scripts.js") == 0) {
		    snprintf(filepath, sizeof(filepath), "/littlefs/scripts.js");
		} else if (strcmp(req->uri, "/power-settings.js") == 0) {
		    snprintf(filepath, sizeof(filepath), "/littlefs/power-settings.js");
		}
		else {
		    snprintf(filepath, sizeof(filepath), "/littlefs%s", req->uri);
		}
	    ESP_LOGI(TAG, "Requested URI: %s, Filepath: %s", req->uri, filepath);

    // Открываем файл
    FILE* file = fopen(filepath, "r");
    if (!file) {
        ESP_LOGE(TAG, "Failed to open file: %s", filepath);
        httpd_resp_send_err(req, HTTPD_404_NOT_FOUND, "File Not Found");
        return ESP_FAIL;
    }

    // Определяем MIME-тип и устанавливаем заголовок
    const char* mime_type = get_mime_type(filepath);
    httpd_resp_set_type(req, mime_type);

    // Читаем и отправляем файл
    char buf[1024];
    size_t read_bytes;
    while ((read_bytes = fread(buf, 1, sizeof(buf), file)) > 0) {
        if (httpd_resp_send_chunk(req, buf, read_bytes) != ESP_OK) {
            fclose(file);
            return ESP_FAIL;
        }
    }

    // Завершаем отправку
    httpd_resp_send_chunk(req, NULL, 0);
    fclose(file);

    return ESP_OK;
}


static httpd_handle_t server = NULL;
static bool server_started = false;

httpd_handle_t start_webserver(void) {
	
	 httpd_stop(server_handle);
    if (server_started) {
        ESP_LOGI(TAG, "Server is already started");
        return server;
    }
	
	 ESP_LOGI(TAG, "Initializing web server...");
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;
    config.max_uri_handlers = 16;  // Увеличиваем количество слотов для URI-обработчиков
    config.stack_size = 8192;  // Увеличьте размер стека
    httpd_handle_t server = NULL;
    
   
    
    if (httpd_start(&server, &config) == ESP_OK) {		
		   ESP_LOGI(TAG, "Web server started on port: %d", config.server_port);		   
				httpd_uri_t root_get_uri = {
				    .uri = "/",  // Обработка корневого запроса
				    .method = HTTP_GET,
				    .handler = file_get_handler,  // Обработчик для загрузки файла
				    .user_ctx = NULL
				};
				httpd_register_uri_handler(server, &root_get_uri);

                httpd_uri_t settings_get_uri = {
			    .uri = "/settings.html",  // Обработка конкретного запроса
			    .method = HTTP_GET,
			    .handler = file_get_handler,
			    .user_ctx = NULL
		     	};
		    	httpd_register_uri_handler(server, &settings_get_uri);

		        httpd_uri_t style_get_uri = {
			    .uri = "/style.css",  // Обработка стилей
			    .method = HTTP_GET,
			    .handler = file_get_handler,
			    .user_ctx = NULL
	            };
	            httpd_register_uri_handler(server, &style_get_uri);
   
		   // Обработчик для JavaScript
		        httpd_uri_t script_get_uri = {
		            .uri = "/scripts.js",  // Обработка JavaScript
		            .method = HTTP_GET,
		            .handler = file_get_handler,
		            .user_ctx = NULL
		        };
		        httpd_register_uri_handler(server, &script_get_uri);
		        
		     
		   // Обработчик для JavaScript
		        httpd_uri_t ps_script_get_uri = {
		            .uri = "/power-settings.js",  // Обработка JavaScript
		            .method = HTTP_GET,
		            .handler = file_get_handler,
		            .user_ctx = NULL
		        };
		        httpd_register_uri_handler(server, &ps_script_get_uri);   
   
			   // Регистрация URI-обработчика
			    httpd_uri_t data_uri = {
			    .uri      = "/data",
			    .method   = HTTP_GET,
			    .handler  = data_get_handler,
			    .user_ctx = NULL
		        };
		
		      httpd_register_uri_handler(server, &data_uri);
					
				httpd_uri_t get_ip_uri = {
				    .uri = "/get_ip",
				    .method = HTTP_GET,
				    .handler = get_ip_handler,
				    .user_ctx = NULL
				};
			httpd_register_uri_handler(server, &get_ip_uri);
						
						
			httpd_uri_t favicon_uri = {
			    .uri = "/favicon.png",
			    .method = HTTP_GET,
			    .handler = file_get_handler,
			    .user_ctx = NULL
			};
			httpd_register_uri_handler(server, &favicon_uri);
					
			httpd_uri_t wifi_mode_uri = {
			    .uri       = "/wifi_mode",
			    .method    = HTTP_POST,
			    .handler   = wifi_mode_handler,
			    .user_ctx  = NULL
			};
               // Добавляем маршрут к серверу
            httpd_register_uri_handler(server, &wifi_mode_uri);
			
	        httpd_uri_t post_uri = {
	            .uri = "/post",
	            .method = HTTP_POST,
	            .handler = post_handler,
	            .user_ctx = NULL
	        };
	        httpd_register_uri_handler(server, &post_uri);
	        
	        httpd_uri_t save_wifi_settings_uri = {
                .uri = "/save_wifi_settings",
                .method = HTTP_POST,
                .handler = save_wifi_settings_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler(server, &save_wifi_settings_uri);

            httpd_uri_t login_uri = {
                .uri = "/login",
                .method = HTTP_POST,
                .handler = login_handler,
                .user_ctx = NULL
            };
            httpd_register_uri_handler(server, &login_uri);
       
            httpd_uri_t ota_post_uri = {
                .uri       = "/ota",
                .method    = HTTP_POST,
                .handler   = ota_post_handler,
                .user_ctx  = NULL
                };
                if (httpd_register_uri_handler(server, &ota_post_uri) == ESP_OK) {
                ESP_LOGI(TAG, "OTA POST handler registered at URI: /ota");
            } else {
                ESP_LOGE(TAG, "Failed to register OTA POST handler");
            }
    

    } else {
        ESP_LOGE(TAG, "Failed to start the server");
    }

    return server;
}

void app_main(void) {
	 
     // Инициализация NVS
    init_nvs();
    init_reset_pin();
    // Загружаем настройки
    device_settings_t current_settings;
    load_settings_from_nvs(&current_settings);
     
    // Инициализация TCP/IP стека
    if (!netif_initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        netif_initialized = true;
    }

    // Инициализация цикла событий
    if (!event_loop_created) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        event_loop_created = true;
    }  
  

    // Чтение сохраненного режима Wi-Fi из NVS
    ESP_LOGI("app_main", "Reading saved Wi-Fi mode from NVS...");
 
    init_wifi_from_nvs();
  
    // Монтирование файловой системы LittleFS
    if (!filesystem_mounted) {
        mount_littlefs();
        filesystem_mounted = true;
    }
    // Вывод списка файлов
    list_files("/littlefs");
 
    start_webserver();
    
    // Инициализация мьютекса
    uart_mutex = xSemaphoreCreateMutex();
    if (uart_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create UART mutex");
        return;  // Обработка ошибки
    }
   uart_command_queue = xQueueCreate(10, sizeof(uart_command_t));
   init_uart();
   
	 // Создание очередей
    //uart_command_queue = xQueueCreate(10, sizeof(uart_command_t));
		// Создаём задачу для обработки UART событий
    xTaskCreate(uart_event_task, "uart_event_task", 2048, NULL, 12, NULL);

    // Создание задач
    xTaskCreate(wifi_signal_strength_task, "wifi_signal_strength_task", 2048, NULL, 5, NULL);
    xTaskCreate(uart_command_task, "uart_command_task", 2048, NULL, 5, NULL);
    xTaskCreate(periodic_request_task, "periodic_request_task", 2048, NULL, 5, NULL);
      // Запуск задачи для отслеживания состояния пина заводских настроек
    xTaskCreate(reset_task, "reset_task",2048, NULL,2, NULL);
    
}

