cmake_minimum_required(VERSION 3.13)
set(PICO_SDK_FETCH_FROM_GIT on)
include ($ENV{PICO_SDK_PATH}/external/pico_sdk_import.cmake)

project(rp2040_mouse)
enable_language( C CXX ASM )
pico_sdk_init()

include ($ENV{PICO_PIO_USB_DIR}/CMakeLists.txt)

set(target_name rp2040_mouse)
add_executable(${target_name})

target_sources(${target_name} PRIVATE
 main.c
 ${PICO_TINYUSB_PATH}/src/portable/raspberrypi/pio_usb/hcd_pio_usb.c
 )

 pico_enable_stdio_uart(${target_name} 1)
 pico_enable_stdio_usb(${target_name} 0)

# set info for picotool

pico_set_program_name(rp2040_mouse "RP2040 Mouse")
pico_set_program_description(rp2040_mouse "RP2040 Based USB to Quadrature mouse adaptor for retro computers")
pico_set_program_version(rp2040_mouse "1.0")
pico_set_program_url(rp2040_mouse "https://github.com/Black-FX/RP2040_Mouse")
 
# print memory usage, enable all warnings
target_link_options(${target_name} PRIVATE -Xlinker --print-memory-usage)
target_compile_options(${target_name} PRIVATE -Wall -Wextra)

# use tinyusb implementation
target_compile_definitions(${target_name} PRIVATE PIO_USB_USE_TINYUSB)

# needed so tinyusb can find tusb_config.h
target_include_directories(${target_name} PRIVATE ${CMAKE_CURRENT_LIST_DIR})

target_link_libraries(${target_name} PRIVATE pico_stdlib pico_multicore pico_pio_usb tinyusb_host)
pico_add_extra_outputs(${target_name})