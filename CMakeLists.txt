
cmake_minimum_required(VERSION 3.13)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(PICO_BOARD pico_w CACHE STRING "Board type")
include(pico_sdk_import.cmake)

project(smart_home_mqtt C CXX ASM)
pico_sdk_init()

add_executable(${PROJECT_NAME} lib/ssd1306.c smart_home_mqtt.c )

pico_enable_stdio_uart(${PROJECT_NAME} 1)
pico_enable_stdio_usb(${PROJECT_NAME} 1)

pico_generate_pio_header(smart_home_mqtt ${CMAKE_CURRENT_LIST_DIR}/smart_home_mqtt.pio)
target_sources(smart_home_mqtt PRIVATE smart_home_mqtt.c)

target_link_libraries(${PROJECT_NAME}
        pico_stdlib
        hardware_gpio
        hardware_adc
        hardware_i2c
        hardware_pwm
        hardware_pio
        pico_cyw43_arch_lwip_threadsafe_background
        pico_lwip_mqtt
        pico_mbedtls
        pico_lwip_mbedtls
        )

target_include_directories(smart_home_mqtt PRIVATE
        ${CMAKE_CURRENT_LIST_DIR}
)

pico_add_extra_outputs(smart_home_mqtt)