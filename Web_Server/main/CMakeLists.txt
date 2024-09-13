# See the build system documentation in IDF programming guide
# for more information about component CMakeLists.txt files.


idf_component_register(
    SRCS main.c
    INCLUDE_DIRS
    PRIV_INCLUDE_DIRS
    REQUIRES esp_wifi esp_netif esp_http_server nvs_flash littlefs 
    PRIV_REQUIRES
)

# Note: The line below might be unnecessary if you're using `idf.py` for configuration and building
 set(FLASH_TARGET esp32c3)
