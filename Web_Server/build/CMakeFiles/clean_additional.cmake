# Additional clean files
cmake_minimum_required(VERSION 3.16)

if("${CONFIG}" STREQUAL "" OR "${CONFIG}" STREQUAL "")
  file(REMOVE_RECURSE
  "app-template.map"
  "esp-idf\\esptool_py\\flasher_args.json.in"
  "esp-idf\\mbedtls\\x509_crt_bundle"
  "flash_app_args"
  "flash_bootloader_args"
  "flasher_args.json"
  "ldgen_libraries"
  "ldgen_libraries.in"
  "littlefs_py_venv"
  "project_elf_src_esp32c3.c"
  "x509_crt_bundle.S"
  )
endif()
