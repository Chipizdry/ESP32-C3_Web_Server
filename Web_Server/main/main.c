#include <stdbool.h>
#include <unistd.h>
#include <stdio.h>
#include <string.h>
#include <esp_log.h>
#include <esp_err.h>
#include <esp_system.h>
#include <esp_netif.h>
#include <esp_event.h>
#include <esp_wifi.h>
#include <esp_http_server.h>
#include <nvs_flash.h>
#include "esp_littlefs.h"
#include "dirent.h"
#include "esp_err.h"
#include "esp_mac.h"
#include "esp_mac.h"
#include "nvs.h"
#include <time.h>
#include "lwip/ip4_addr.h"


// Статический IP и параметры Wi-Fi AP
//#define AP_SSID "ESP32AP"
#define AP_SSID_PREFIX "FlyWheel-"  // Префикс для SSID
#define AP_PASS "12345678"
#define AP_CHANNEL 1
#define AP_MAX_CONN 4
#define AP_IP "192.168.1.1"
#define AP_GW "192.168.1.1"
#define AP_NETMASK "255.255.255.0"



void init_nvs() {
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);
}




void list_files(const char *base_path);
static const char *TAG = "web_server";

esp_netif_t* netif_sta = NULL;  // Интерфейс для Station
esp_netif_t* netif_ap = NULL;   // Интерфейс для Access Point




void init_wifi_ap() {
	 // Получаем MAC-адрес устройства
    uint8_t mac[6];
    esp_read_mac(mac, ESP_MAC_WIFI_STA);  // Читаем MAC-адрес Wi-Fi станции

    // Преобразуем последние 3 байта MAC-адреса в шестнадцатеричную строку
    char unique_id[7];  // 6 символов + терминатор
    snprintf(unique_id, sizeof(unique_id), "%02X%02X%02X", mac[3], mac[4], mac[5]);

    // Формируем полный SSID с уникальной частью
    char ssid[32];  // Размер SSID максимум 32 байта
    snprintf(ssid, sizeof(ssid), "%s%s", AP_SSID_PREFIX, unique_id);

    // Настройка Wi-Fi AP
    wifi_config_t wifi_config = {
        .ap = {
            //.ssid = AP_SSID,
             .ssid_len = strlen(ssid),
            .password = AP_PASS,
            .channel = AP_CHANNEL,
            .max_connection = AP_MAX_CONN,
            .authmode = WIFI_AUTH_WPA_WPA2_PSK,
        },
    };
 // Копируем SSID в конфигурацию
    strcpy((char *)wifi_config.ap.ssid, ssid);

    // Если пароль пустой, задаем открытую аутентификацию
    if (strlen(AP_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    esp_netif_t *netif = esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    // Настройка статического IP для AP
    esp_netif_ip_info_t ip_info;
    ip4addr_aton(AP_IP, &ip_info.ip);         // Установка IP-адреса
    ip4addr_aton(AP_GW, &ip_info.gw);         // Установка шлюза
    ip4addr_aton(AP_NETMASK, &ip_info.netmask); // Установка маски подсети
    
    ESP_ERROR_CHECK(esp_netif_dhcps_stop(netif));  // Останавливаем DHCP-сервер для установки IP вручную
    ESP_ERROR_CHECK(esp_netif_set_ip_info(netif, &ip_info));
    ESP_ERROR_CHECK(esp_netif_dhcps_start(netif));  // Включаем DHCP-сервер

    ESP_LOGI(TAG, "Wi-Fi AP Mode Initialized. SSID: %s, Password: %s",  ssid, AP_PASS);
    ESP_LOGI(TAG, "AP IP Address: %s", AP_IP);
}

void load_settings(int* setting1, int* setting2) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error opening NVS handle");
        return;
    }

    // Чтение значений
    err = nvs_get_i32(nvs_handle, "setting1", setting1);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Setting1 not found");
        *setting1 = 0;  // Значение по умолчанию
    }
    
    err = nvs_get_i32(nvs_handle, "setting2", setting2);
    if (err == ESP_ERR_NVS_NOT_FOUND) {
        ESP_LOGI(TAG, "Setting2 not found");
        *setting2 = 0;  // Значение по умолчанию
    }

    nvs_close(nvs_handle);
}


void save_settings(int setting1, int setting2) {
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err != ESP_OK) {
        ESP_LOGI(TAG, "Error opening NVS handle");
        return;
    }

    // Запись значений
    err = nvs_set_i32(nvs_handle, "setting1", setting1);
    ESP_ERROR_CHECK(err);

    err = nvs_set_i32(nvs_handle, "setting2", setting2);
    ESP_ERROR_CHECK(err);

    // Сохранение изменений
    err = nvs_commit(nvs_handle);
    ESP_ERROR_CHECK(err);

    nvs_close(nvs_handle);
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


void initialize_netifs() {
    netif_sta = esp_netif_create_default_wifi_sta();  // Интерфейс для режима Station
    if (netif_sta == NULL) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi STA interface");
        return;
    }

    netif_ap = esp_netif_create_default_wifi_ap();    // Интерфейс для режима Access Point
    if (netif_ap == NULL) {
        ESP_LOGE(TAG, "Failed to create default Wi-Fi AP interface");
    }
}

void setup_random() {
    // Инициализация генератора случайных чисел
    srand(time(NULL));
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
    // Добавьте другие типы по необходимости
    return "text/plain";
}

// Обработчик событий Wi-Fi
static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data) {
    if (event_base == WIFI_EVENT) {
        switch (event_id) {
            case WIFI_EVENT_STA_START:
                esp_wifi_connect();
                break;
            case WIFI_EVENT_STA_CONNECTED:
                ESP_LOGI(TAG, "Connected to Wi-Fi");
                break;
            case WIFI_EVENT_STA_DISCONNECTED:
                ESP_LOGI(TAG, "Disconnected from Wi-Fi");
                esp_wifi_connect();
                break;
            default:
                break;
        }
    } else if (event_base == IP_EVENT) {
        switch (event_id) {
            case IP_EVENT_STA_GOT_IP:
                ESP_LOGI(TAG, "Got IP address");
                break;
            default:
                break;
        }
    }
}


esp_err_t get_ip_handler(httpd_req_t *req) {
    char response[128];
    esp_netif_ip_info_t ip_info;
    wifi_mode_t mode;
    esp_err_t err = esp_wifi_get_mode(&mode);
    
     ESP_LOGI("IP_HANDLER", "Received request for URI: %s", req->uri);
    
    // Логируем заголовки запроса
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

    // Формируем JSON-ответ с IP, маской и шлюзом
    snprintf(response, sizeof(response),
             "{\"ip\": \"" IPSTR "\", \"netmask\": \"" IPSTR "\", \"gateway\": \"" IPSTR "\"}",
             IP2STR(&ip_info.ip), IP2STR(&ip_info.netmask), IP2STR(&ip_info.gw));

    httpd_resp_set_type(req, "application/json");
    esp_err_t res = httpd_resp_send(req, response, strlen(response));
    if (res != ESP_OK) {
        ESP_LOGE("IP_HANDLER", "Failed to send response: %s", esp_err_to_name(res));
    }

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

    // Определяем, какой режим был запрошен (например, "sta" или "ap")
    
    if (strstr(buf, "sta") != NULL) {
        esp_wifi_set_mode(WIFI_MODE_STA);
        ESP_LOGI(TAG, "Switching to STA mode");
    } else if (strstr(buf, "ap") != NULL) {
        init_wifi_ap();  // Вызов функции настройки AP
        ESP_LOGI(TAG, "Switching to AP mode");
    }

    // Сохранение текущего режима Wi-Fi в NVS для будущего использования
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READWRITE, &nvs_handle);
    if (err == ESP_OK) {
        if (strstr(buf, "sta") != NULL) {
            nvs_set_u8(nvs_handle, "wifi_mode", WIFI_MODE_STA);
        } else {
            nvs_set_u8(nvs_handle, "wifi_mode", WIFI_MODE_AP);
        }
        nvs_commit(nvs_handle);
        nvs_close(nvs_handle);
    } else {
        ESP_LOGE(TAG, "Failed to save Wi-Fi mode to NVS");
    }

    httpd_resp_send(req, "Wi-Fi mode updated", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

void init_wifi_from_nvs() {
    uint8_t wifi_mode = WIFI_MODE_STA;
    nvs_handle_t nvs_handle;
    esp_err_t err = nvs_open("storage", NVS_READONLY, &nvs_handle);

    if (err == ESP_OK) {
        // Reading the saved Wi-Fi mode from NVS
        err = nvs_get_u8(nvs_handle, "wifi_mode", &wifi_mode);
        nvs_close(nvs_handle);
        
        if (err == ESP_ERR_NVS_NOT_FOUND) {
            ESP_LOGI(TAG, "Wi-Fi mode not found in NVS, defaulting to STA");
        } else if (err != ESP_OK) {
            ESP_LOGE(TAG, "Error reading Wi-Fi mode from NVS: %s", esp_err_to_name(err));
        }
    } else {
        ESP_LOGE(TAG, "Failed to open NVS: %s", esp_err_to_name(err));
    }

    // Apply the Wi-Fi mode based on the value from NVS
    if (wifi_mode == WIFI_MODE_AP) {
        init_wifi_ap();  // Initialize as Access Point
        ESP_LOGI(TAG, "Initializing Wi-Fi in AP mode");
    } else {
        ESP_LOGI(TAG, "Initializing Wi-Fi in STA mode");
        esp_wifi_set_mode(WIFI_MODE_STA);
        esp_wifi_start();  // Start the STA mode
    }
}


// Обработчик для POST-запросов
esp_err_t post_handler(httpd_req_t *req) {
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
    ESP_LOGI(TAG, "Received POST data: %s", buf);
    
    // Разбор полученных данных (например, JSON)
    int setting1, setting2;
    sscanf(buf, "{\"setting1\":%d, \"setting2\":%d}", &setting1, &setting2);

    // Сохранение настроек в NVS
    save_settings(setting1, setting2);
    
    
    
    httpd_resp_send(req, "POST data received", HTTPD_RESP_USE_STRLEN);
    return ESP_OK;
}

esp_err_t data_get_handler(httpd_req_t *req) {
    // Пример данных
  //  float voltage = 12.5;  // Здесь будет ваше реальное значение напряжения
  //  int speed = 1500;      // Здесь будет реальная скорость
  // Генерация случайного напряжения от 10.0 до 14.0 В
  // float voltage = ((float)(rand() % 100)) / 10.0;  // Генерация напряжения от 0 до 10 В
   float voltage = 10.0 + ((float)(rand() % 101)) / 10.0;  // Генерация напряжения от 10 до 20 В

   // int speed = rand() % 5000;  // Генерация скорости от 0 до 5000 об/мин
   int speed = 1500 + (rand() % 1001);  // Генерация скорости от 1500 до 4500 об/мин

    // Формирование JSON-ответа
    char response[100];
    snprintf(response, sizeof(response), "{\"voltage\": %.2f, \"speed\": %d}", voltage, speed);

    // Отправка ответа клиенту
    httpd_resp_set_type(req, "application/json");
    httpd_resp_send(req, response, strlen(response));

    return ESP_OK;
}



esp_err_t file_get_handler(httpd_req_t *req) {
    char filepath[600];
    if (strcmp(req->uri, "/") == 0) {
        snprintf(filepath, sizeof(filepath), "/littlefs/index.html");
    } else if (strcmp(req->uri, "/favicon.png") == 0) {
        snprintf(filepath, sizeof(filepath), "/littlefs/favicon.png");  // Обслуживаем favicon
    } else {
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



// Функция запуска веб-сервера
httpd_handle_t start_webserver(void) {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.server_port = 80;

    httpd_handle_t server = NULL;
    if (httpd_start(&server, &config) == ESP_OK) {
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
			    .handler = get_ip_handler,
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
    } else {
        ESP_LOGE(TAG, "Failed to start the server");
    }

    return server;
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

// Основная функция приложения
static bool nvs_initialized = false;
static bool netif_initialized = false;
static bool event_loop_created = false;
static bool wifi_initialized = false;
static bool filesystem_mounted = false;

void app_main(void) {
     // Инициализация NVS
    if (!nvs_initialized) {
        esp_err_t ret = nvs_flash_init();
        if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
            ESP_ERROR_CHECK(nvs_flash_erase());
            ret = nvs_flash_init();
        }
        ESP_ERROR_CHECK(ret);
        nvs_initialized = true;
    }



//  init_nvs();
    
    int setting1, setting2;
    load_settings(&setting1, &setting2);

    ESP_LOGI(TAG, "Loaded settings: setting1 = %d, setting2 = %d", setting1, setting2);



    // Инициализация TCP/IP стека
    if (!netif_initialized) {
        ESP_ERROR_CHECK(esp_netif_init());
        netif_initialized = true;
    }

    // Инициализация сетевых интерфейсов
    if (!netif_initialized) {
        initialize_netifs();
        netif_initialized = true;
    }

    // Инициализация цикла событий
    if (!event_loop_created) {
        ESP_ERROR_CHECK(esp_event_loop_create_default());
        event_loop_created = true;
    }
    // Инициализация Wi-Fi
    if (!wifi_initialized) {
        esp_netif_create_default_wifi_sta();
        wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
        ESP_ERROR_CHECK(esp_wifi_init(&cfg));
        ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));

        wifi_config_t wifi_config = {
            .sta = {
                .ssid = "Medical",
                .password = "0445026833"
                
             //    .ssid = "TP-Link_FA4F",
             //   .password = "19481555"
            },
        };
        ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));

        // Регистрация обработчиков событий Wi-Fi
        esp_event_handler_register(WIFI_EVENT, ESP_EVENT_ANY_ID, &wifi_event_handler, NULL);
        esp_event_handler_register(IP_EVENT, IP_EVENT_STA_GOT_IP, &wifi_event_handler, NULL);

        ESP_ERROR_CHECK(esp_wifi_start());
        wifi_initialized = true;
    }

    
    // Монтирование файловой системы LittleFS
    if (!filesystem_mounted) {
        mount_littlefs();
        filesystem_mounted = true;
    }

    // Вывод списка файлов
    list_files("/littlefs");

    // Запуск веб-сервера
    start_webserver();
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