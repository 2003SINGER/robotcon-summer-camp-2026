#ifndef VALVE_H
#define VALVE_H

/*
 * Pneumatic / solenoid control API for mechanism_controller.
 *
 * Three actuators, each driven by a GPIO output on the main board through an
 * electronic switch (ULN2003-style low-side driver). "on" drives the pin HIGH
 * (energize), "off" drives it LOW (de-energize). If a particular channel's
 * driver board is inverting, swap GPIO_PIN_SET / GPIO_PIN_RESET inside
 * valve.c -- that is the ONLY place polarity lives.
 *
 * Pin assignment (matches the hardware wiring):
 *   Valve_rotator -> PB8  (rotating suction cup, via electronic switch)
 *   Valve_loader  -> PB9  (front/back arm suction cup, grabs blocks)
 *   Valve_pallet  -> PB4  (bottom pallet cylinder solenoid valve)
 */
void Valve_Init(void);

void Valve_rotator_on(void);
void Valve_rotator_off(void);

void Valve_loader_on(void);
void Valve_loader_off(void);

void Valve_pallet_on(void);
void Valve_pallet_off(void);

#endif /* VALVE_H */
