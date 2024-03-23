# Distributed under the OSI-approved BSD 3-Clause License.  See accompanying
# file Copyright.txt or https://cmake.org/licensing for details.

cmake_minimum_required(VERSION 3.5)

file(MAKE_DIRECTORY
  "D:/Users/waelk/v5.2.1/esp-idf/components/bootloader/subproject"
  "D:/Users/waelk/solve/build/bootloader"
  "D:/Users/waelk/solve/build/bootloader-prefix"
  "D:/Users/waelk/solve/build/bootloader-prefix/tmp"
  "D:/Users/waelk/solve/build/bootloader-prefix/src/bootloader-stamp"
  "D:/Users/waelk/solve/build/bootloader-prefix/src"
  "D:/Users/waelk/solve/build/bootloader-prefix/src/bootloader-stamp"
)

set(configSubDirs )
foreach(subDir IN LISTS configSubDirs)
    file(MAKE_DIRECTORY "D:/Users/waelk/solve/build/bootloader-prefix/src/bootloader-stamp/${subDir}")
endforeach()
if(cfgdir)
  file(MAKE_DIRECTORY "D:/Users/waelk/solve/build/bootloader-prefix/src/bootloader-stamp${cfgdir}") # cfgdir has leading slash
endif()
