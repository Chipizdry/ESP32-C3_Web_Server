# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "G:/Espressif/frameworks/esp-idf-v5.3.1/components/bootloader/subproject"
  "G:/Espressif_work/Web_Server/build/bootloader"
  "G:/Espressif_work/Web_Server/build/bootloader-prefix"
  "G:/Espressif_work/Web_Server/build/bootloader-prefix/tmp"
  "G:/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp"
  "G:/Espressif_work/Web_Server/build/bootloader-prefix/src"
  "G:/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "G:/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "G:/Espressif_work/Web_Server/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
