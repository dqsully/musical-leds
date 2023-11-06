cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(musical_leds_project C CXX ASM)
set(CMAKE_C_STANDARD 11)
set(CMAKE_CSS_STANDARD 17)
pico_sdk_init()

add_executable(musical_leds)

file(MAKE_DIRECTORY ${CMAKE_CURRENT_LIST_DIR}/generated)

# generate the header file into the source tree as it is included in the RP2040 datasheet
pico_generate_pio_header(musical_leds ${CMAKE_CURRENT_LIST_DIR}/ws2812.pio OUTPUT_DIR ${CMAKE_CURRENT_LIST_DIR}/generated)

target_sources(musical_leds PRIVATE musical_leds.c)

target_link_libraries(musical_leds PRIVATE
    pico_stdlib
    hardware_pio
    hardware_dma
    pico_multicore)
pico_add_extra_outputs(musical_leds)

pico_enable_stdio_usb(musical_leds 1)
pico_enable_stdio_uart(musical_leds 0)
