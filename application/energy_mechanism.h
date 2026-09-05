#ifndef TEXT1_ENERGY_MECHANISM_H
#define TEXT1_ENERGY_MECHANISM_H

#include <stdint.h>

typedef enum
{
    ENERGY_STATUS_UNKNOWN = 0U,
    ENERGY_STATUS_INACTIVE,
    ENERGY_STATUS_ACTIVATING,
    ENERGY_STATUS_ACTIVE,
} EnergyMechanismStatus_e;

void EnergyMechanismInit(void);
void EnergyMechanismStartReceive(void);
void EnergyMechanismRxByte(uint8_t byte);
extern uint8_t energy_rx_byte;
void EnergyMechanismUpdateSwitches(uint8_t color_switch,
                                   uint8_t size_switch,
                                   uint8_t rune_switch,
                                   uint8_t motors_enabled);
EnergyMechanismStatus_e EnergyMechanismGetStatus(void);
uint8_t EnergyMechanismIsLarge(void);
uint8_t EnergyMechanismIsBlue(void);

#endif
