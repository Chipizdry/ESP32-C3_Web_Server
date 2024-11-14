# CMakeLists.txt for ESP-IDF component

idf_component_register(
    SRCS "main.c" "uart.c"
    INCLUDE_DIRS "."
    REQUIRES esp_wifi 
    esp_netif
     esp_http_server
      nvs_flash 
      littlefs
       esp_http_client
        app_update 
        driver 
        esp_http_client cJSON
)
set(PARTITION_TABLE csv)
set(PARTITION_TABLE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/partitions.csv")