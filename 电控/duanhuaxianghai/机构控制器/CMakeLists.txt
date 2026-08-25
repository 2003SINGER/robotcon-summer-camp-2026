cmake_minimum_required(VERSION 3.22)
project(mechanism_controller C ASM)

set(MCU_FLAGS -mcpu=cortex-m7 -mthumb -mfpu=fpv5-d16 -mfloat-abi=hard)
set(COMMON_FLAGS ${MCU_FLAGS} -ffunction-sections -fdata-sections -Wall -Wextra -Werror -Wno-unused-parameter)

add_executable(${PROJECT_NAME}.elf
  Core/Src/main.c
  Core/Src/gpio.c
  Core/Src/fdcan.c
  Core/Src/stm32h7xx_it.c
  Core/Src/stm32h7xx_hal_msp.c
  Core/Src/system_stm32h7xx.c
  Core/Src/app_tasks.c
  Core/Src/can_bus.c
  Core/Src/mechanism.c
  Core/Src/motor.c
  Core/Src/pid.c
  Core/Src/robot_fsm.c
  Core/Src/tuning.c
  Middlewares/FreeRTOS/Source/tasks.c
  Middlewares/FreeRTOS/Source/queue.c
  Middlewares/FreeRTOS/Source/list.c
  Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c
  Middlewares/FreeRTOS/Source/portable/MemMang/heap_4.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c
  Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c
  Core/Startup/startup_stm32h723xx.s
)

target_compile_definitions(${PROJECT_NAME}.elf PRIVATE STM32H723xx USE_HAL_DRIVER USE_PWR_LDO_SUPPLY)
target_compile_options(${PROJECT_NAME}.elf PRIVATE ${COMMON_FLAGS})
target_include_directories(${PROJECT_NAME}.elf PRIVATE
  Core/Inc
  Drivers/STM32H7xx_HAL_Driver/Inc
  Drivers/STM32H7xx_HAL_Driver/Inc/Legacy
  Drivers/CMSIS/Device/ST/STM32H7xx/Include
  Drivers/CMSIS/Include
  Middlewares/FreeRTOS/Source/include
  Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F
)
target_link_options(${PROJECT_NAME}.elf PRIVATE
  ${MCU_FLAGS}
  -T${CMAKE_CURRENT_SOURCE_DIR}/STM32H723ZGTX_FLASH.ld
  -Wl,--gc-sections
  -Wl,-Map=${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}.map
  --specs=nano.specs
  --specs=nosys.specs
)

add_custom_command(TARGET ${PROJECT_NAME}.elf POST_BUILD
  COMMAND ${CMAKE_OBJCOPY} -O ihex $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.hex
  COMMAND ${CMAKE_OBJCOPY} -O binary $<TARGET_FILE:${PROJECT_NAME}.elf> ${PROJECT_NAME}.bin
  COMMAND ${CMAKE_SIZE} $<TARGET_FILE:${PROJECT_NAME}.elf>
  VERBATIM
)
