cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_FETCH_FROM_GIT on)
include (pico_sdk_import.cmake)

project(rp2040_mouse)
enable_language( C CXX ASM )
pico_sdk_init()

include ($ENV{PICO_PIO_USB_DIR}/CMakeLists.txt)

set(target_name rp2040_mouse)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 main.c
 # usb_descriptors.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# use tinyusb implementation
target_compile_definitions(${target_name} PRIVATE PIO_USB_USE_TINYUSB)

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore pico_pio_usb tinyusb_device tinyusb_host)
pico_add_extra_outputs(${target_name})