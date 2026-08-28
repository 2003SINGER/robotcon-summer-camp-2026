#include "valve.h"
#include "main.h"

/*
 * Pin assignment (matches the hardware wiring). Edit here if a wire moves.
 *   Valve_rotator -> PB8
 *   Valve_loader  -> PB9
 *   Valve_pallet  -> PB4
 * "on" = pin HIGH, "off" = pin LOW.
 */
#define ROTATOR_PORT GPIOB
#define ROTATOR_PIN  GPIO_PIN_8
#define LOADER_PORT  GPIOB
#define LOADER_PIN   GPIO_PIN_9
#define PALLET_PORT  GPIOB
#define PALLET_PIN   GPIO_PIN_4

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
