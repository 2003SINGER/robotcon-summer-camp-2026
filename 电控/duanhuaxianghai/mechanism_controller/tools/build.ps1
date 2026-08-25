param(
  [ValidateSet('Debug', 'Release')]
  [string]$Configuration = 'Debug'
)

$ErrorActionPreference = 'Stop'
$projectRoot = Split-Path -Parent $PSScriptRoot
$toolRoot = 'E:\STM32CubeCLT_1.21.0\GNU-tools-for-STM32\bin'
$gcc = Join-Path $toolRoot 'arm-none-eabi-gcc.exe'
$objcopy = Join-Path $toolRoot 'arm-none-eabi-objcopy.exe'
$size = Join-Path $toolRoot 'arm-none-eabi-size.exe'
$outputDirectory = Join-Path $projectRoot 'build'
$elf = Join-Path $outputDirectory 'mechanism_controller.elf'

foreach ($tool in @($gcc, $objcopy, $size)) {
  if (-not (Test-Path -LiteralPath $tool)) { throw "STM32CubeCLT tool missing: $tool" }
}
New-Item -ItemType Directory -Force -Path $outputDirectory | Out-Null

$sources = @(
  'Core/Src/main.c', 'Core/Src/gpio.c', 'Core/Src/fdcan.c', 'Core/Src/usart.c',
  'Core/Src/stm32h7xx_it.c', 'Core/Src/stm32h7xx_hal_msp.c',
  'Core/Src/system_stm32h7xx.c', 'Core/Src/app_tasks.c',
  'Core/Src/command_mailbox.c', 'Core/Src/can_bus.c', 'Core/Src/mechanism.c',
  'Core/Src/motor.c', 'Core/Src/pid.c', 'Core/Src/robot_fsm.c', 'Core/Src/tuning.c',
  'Middlewares/FreeRTOS/Source/tasks.c', 'Middlewares/FreeRTOS/Source/queue.c',
  'Middlewares/FreeRTOS/Source/list.c',
  'Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F/port.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_cortex.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_dma_ex.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_exti.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_fdcan.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_flash_ex.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_gpio.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_hsem.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_mdma.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_pwr_ex.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_rcc_ex.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart.c',
  'Drivers/STM32H7xx_HAL_Driver/Src/stm32h7xx_hal_uart_ex.c',
  'Core/Startup/startup_stm32h723xx.s'
) | ForEach-Object { Join-Path $projectRoot $_ }

$includeDirectories = @(
  'Core/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc', 'Drivers/STM32H7xx_HAL_Driver/Inc/Legacy',
  'Drivers/CMSIS/Device/ST/STM32H7xx/Include', 'Drivers/CMSIS/Include',
  'Middlewares/FreeRTOS/Source/include', 'Middlewares/FreeRTOS/Source/portable/GCC/ARM_CM4F'
) | ForEach-Object { '-I' + (Join-Path $projectRoot $_) }

$flags = @('-mcpu=cortex-m7', '-mthumb', '-mfpu=fpv5-d16', '-mfloat-abi=hard',
           '-ffunction-sections', '-fdata-sections', '-Wall', '-Wextra', '-Werror',
           '-Wno-unused-parameter', '-DSTM32H723xx', '-DUSE_HAL_DRIVER', '-DUSE_PWR_LDO_SUPPLY')
if ($Configuration -eq 'Debug') { $flags += @('-Og', '-g3') } else { $flags += '-Os' }

Push-Location $projectRoot
try {
  & $gcc @flags @includeDirectories @sources "-T$projectRoot/STM32H723ZGTX_FLASH.ld" `
    '-Wl,--gc-sections' "-Wl,-Map=$outputDirectory/mechanism_controller.map" `
    '--specs=nano.specs' '--specs=nosys.specs' '-o' $elf
  if ($LASTEXITCODE -ne 0) { throw "GCC exited with $LASTEXITCODE" }
  & $objcopy '-O' 'ihex' $elf (Join-Path $outputDirectory 'mechanism_controller.hex')
  if ($LASTEXITCODE -ne 0) { throw "objcopy (hex) exited with $LASTEXITCODE" }
  & $objcopy '-O' 'binary' $elf (Join-Path $outputDirectory 'mechanism_controller.bin')
  if ($LASTEXITCODE -ne 0) { throw "objcopy (bin) exited with $LASTEXITCODE" }
  & $size $elf
  if ($LASTEXITCODE -ne 0) { throw "size exited with $LASTEXITCODE" }
} finally {
  Pop-Location
}
