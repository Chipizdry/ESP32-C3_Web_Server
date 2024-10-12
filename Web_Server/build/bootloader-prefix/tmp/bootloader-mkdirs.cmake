# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "G:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "C:/Work/Espressif_work/Web_Server/build/bootloader"
  "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix"
  "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/tmp"
  "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp"
  "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/src"
  "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "C:/Work/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
