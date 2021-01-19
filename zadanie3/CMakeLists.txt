cmake_minimum_required(VERSION 3.17)
project(zadanie3 C)

set(CMAKE_C_STANDARD 11)
set(CMAKE_CPP_FLAGS DSTM32F411xE)

add_executable(zadanie3
        main.c
        leds.c leds.h
        print_dma.c print_dma.h
        timer.c timer.h
        i2c.c i2c.h
        utils.c utils.h)

add_definitions(-DSTM32F411xE)
include_directories(/opt/arm/stm32/inc)
include_directories(/opt/arm/stm32/CMSIS/Include)
include_directories(/opt/arm/stm32/CMSIS/Device/ST/STM32F4xx/Include)
link_directories(/opt/arm/stm32/lds)