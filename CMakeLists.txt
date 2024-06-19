cmake_minimum_required(VERSION 3.13)

include(pico_sdk_import.cmake)

project(main C CXX ASM)
set(CMAKE_C STANDARD 11)
set(CMAKE_CXX_STANDARD 17)

pico_sdk_init()

add_executable(${CMAKE_PROJECT_NAME}
    src/main.c
)

FILE(GLOB FreeRTOS_src FreeRTOS-Kernel/*.c)

add_library( FreeRTOS STATIC
			${FreeRTOS_src}
			FreeRTOS-Kernel/portable/GCC/ARM_CM0/port.c
			FreeRTOS-Kernel/portable/MemMang/heap_4.c)


target_include_directories( FreeRTOS PUBLIC
						FreeRTOS-Kernel/include
						include/
						FreeRTOS-Kernel/portable/GCC/ARM_CM0)


target_link_libraries(${CMAKE_PROJECT_NAME}  
					pico_stdlib
					hardware_gpio
					hardware_adc
					FreeRTOS)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
