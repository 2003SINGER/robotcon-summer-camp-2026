#include "valve.h"
#include "main.h"

/*
 * Pin assignment (matches the hardware wiring). Edit here if a wire moves.
 *   Valve_rotator -> PB0
 *   Valve_loader  -> PC12
 *   Valve_pallet  -> PB1
 * "on" = pin HIGH, "off" = pin LOW.
 */
#define ROTATOR_PORT VALVE_ROTATOR_GPIO_Port
#define ROTATOR_PIN  VALVE_ROTATOR_Pin
#define LOADER_PORT  VALVE_LOADER_GPIO_Port
#define LOADER_PIN   VALVE_LOADER_Pin
#define PALLET_PORT  VALVE_PALLET_GPIO_Port
#define PALLET_PIN   VALVE_PALLET_Pin

void Valve_Init(void)
{
  GPIO_InitTypeDef gpio = {0};
  gpio.Mode = GPIO_MODE_OUTPUT_PP;
  gpio.Pull = GPIO_NOPULL;
  gpio.Speed = GPIO_SPEED_FREQ_LOW;

  gpio.Pin = ROTATOR_PIN;
  HAL_GPIO_Init(ROTATOR_PORT, &gpio);
  gpio.Pin = LOADER_PIN;
  HAL_GPIO_Init(LOADER_PORT, &gpio);
  gpio.Pin = PALLET_PIN;
  HAL_GPIO_Init(PALLET_PORT, &gpio);

  /* Default everything off (de-energized). */
  HAL_GPIO_WritePin(ROTATOR_PORT, ROTATOR_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(LOADER_PORT, LOADER_PIN, GPIO_PIN_RESET);
  HAL_GPIO_WritePin(PALLET_PORT, PALLET_PIN, GPIO_PIN_RESET);
}

void Valve_rotator_on(void)
{
  HAL_GPIO_WritePin(ROTATOR_PORT, ROTATOR_PIN, GPIO_PIN_SET);
}

void Valve_rotator_off(void)
{
  HAL_GPIO_WritePin(ROTATOR_PORT, ROTATOR_PIN, GPIO_PIN_RESET);
}

void Valve_loader_on(void)
{
  HAL_GPIO_WritePin(LOADER_PORT, LOADER_PIN, GPIO_PIN_SET);
}

void Valve_loader_off(void)
{
  HAL_GPIO_WritePin(LOADER_PORT, LOADER_PIN, GPIO_PIN_RESET);
}

void Valve_pallet_on(void)
{
  HAL_GPIO_WritePin(PALLET_PORT, PALLET_PIN, GPIO_PIN_SET);
}

void Valve_pallet_off(void)
{
  HAL_GPIO_WritePin(PALLET_PORT, PALLET_PIN, GPIO_PIN_RESET);
}
