cmake_minimum_required(VERSION 3.20)

set(LIBRARY_SOURCE ../libraries)
set(PICO_SOURCE ../pico-sdk)
include(${PICO_SOURCE}/pico_sdk_init.cmake)

project(main C CXX ASM)
set(CMAKE_C STANDARD 11)
set(CMAKE_CXX_STANDARD 17)
set(PICO_SDK_FREERTOS_SOURCE ../FreeRTOS-LTS/FreeRTOS/FreeRTOS-Kernel)

pico_sdk_init()

add_executable(${CMAKE_PROJECT_NAME}
	src/main.c
)

FILE(GLOB FreeRTOS_src ${PICO_SDK_FREERTOS_SOURCE}/*.c)

add_library(FreeRTOS STATIC
			${FreeRTOS_src} 	
			${PICO_SDK_FREERTOS_SOURCE}/portable/GCC/ARM_CM0/port.c
			${PICO_SDK_FREERTOS_SOURCE}/portable/MemMang/heap_4.c)


target_include_directories(	FreeRTOS PUBLIC
							${PICO_SDK_FREERTOS_SOURCE}/include
							${PICO_SDK_FREERTOS_SOURCE}/portable/GCC/ARM_CM0
							config/
							${LIBRARY_SOURCE}/
							)

target_link_libraries(	${CMAKE_PROJECT_NAME}  
						pico_stdlib
						hardware_gpio
						hardware_adc
						hardware_pwm
						hardware_irq
						FreeRTOS)

pico_add_extra_outputs(${CMAKE_PROJECT_NAME})
pico_enable_stdio_usb(${CMAKE_PROJECT_NAME} 1)
pico_enable_stdio_uart(${CMAKE_PROJECT_NAME} 0)
