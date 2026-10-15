cmake_minimum_required(VERSION 3.16)
project(STM32_Auxiliary C ASM)
set(CMAKE_C_STANDARD 11)
set(HAL Drivers/STM32F1xx_HAL_Driver)
set(RTOS Middlewares/Third_Party/FreeRTOS/Source)
file(GLOB APP Core/Src/*.c Core/BSP/*.c Core/Tasks/*.c Core/Middleware/microros/*.c)
file(GLOB HAL_SRC ${HAL}/Src/*.c)
add_executable(${PROJECT_NAME}.elf ${APP} ${HAL_SRC} Core/Startup/startup_stm32f103xb.s
    ${RTOS}/tasks.c ${RTOS}/queue.c ${RTOS}/list.c ${RTOS}/timers.c ${RTOS}/event_groups.c
    ${RTOS}/stream_buffer.c ${RTOS}/croutine.c ${RTOS}/portable/MemMang/heap_4.c
    ${RTOS}/portable/GCC/ARM_CM3/port.c ${RTOS}/CMSIS_RTOS_V2/cmsis_os2.c)
target_include_directories(${PROJECT_NAME}.elf PRIVATE Core/Inc Core/Inc/interfaces Core/BSP Core/Tasks
    Core/Middleware/microros ${HAL}/Inc Drivers/CMSIS/Include Drivers/CMSIS/Device/ST/STM32F1xx/Include
    ${RTOS}/include ${RTOS}/portable/GCC/ARM_CM3 ${RTOS}/CMSIS_RTOS_V2)
target_compile_definitions(${PROJECT_NAME}.elf PRIVATE USE_HAL_DRIVER STM32F103xB)
target_compile_options(${PROJECT_NAME}.elf PRIVATE -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    -Os -g3 -ffunction-sections -fdata-sections -Wall -Wextra)
target_link_options(${PROJECT_NAME}.elf PRIVATE -mcpu=cortex-m3 -mthumb -mfloat-abi=soft
    --specs=nano.specs --specs=nosys.specs -T${CMAKE_CURRENT_SOURCE_DIR}/STM32F103C8TX_FLASH.ld
    -Wl,--gc-sections,-Map=${PROJECT_NAME}.map,--print-memory-usage)
target_link_libraries(${PROJECT_NAME}.elf PRIVATE m)
add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
    COMMAND arm-none-eabi-objcopy -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
    COMMAND arm-none-eabi-size $<TARGET_FILE:${PROJECT_NAME}.elf>)
