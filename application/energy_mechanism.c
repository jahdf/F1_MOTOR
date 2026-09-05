#include "energy_mechanism.h"

#include "main.h"
#include "usart.h"
#include "remote_control.h"

#define ENERGY_FRAME_HEAD_0 0xAAU
#define ENERGY_FRAME_HEAD_1 0x55U
#define ENERGY_FRAME_SIZE 4U

#define ENERGY_CMD_MODE 0x01U
#define ENERGY_CMD_COLOR 0x02U
#define ENERGY_CMD_ACTIVATE 0x03U
#define ENERGY_CMD_RESET 0x04U

uint8_t energy_rx_byte;
static uint8_t rx_frame[ENERGY_FRAME_SIZE];
static uint8_t rx_index;
static volatile EnergyMechanismStatus_e status;
static uint8_t status_initialized;
static uint8_t color_switch_last;
static uint8_t size_switch_last;
static uint8_t rune_switch_last;
static uint8_t switches_initialized;
static volatile uint8_t blue;
static volatile uint8_t large;

static uint8_t IsTwoPosition(uint8_t value)
{
    return (uint8_t)(switch_is_up(value) || switch_is_down(value));
}

static void SendCommand(uint8_t command, uint8_t value)
{
    uint8_t frame[ENERGY_FRAME_SIZE] = {
        ENERGY_FRAME_HEAD_0, ENERGY_FRAME_HEAD_1, command, value
    };
    (void)HAL_UART_Transmit(&huart2, frame, ENERGY_FRAME_SIZE, 20U);
}

void EnergyMechanismInit(void)
{
    rx_index = 0U;
    status = ENERGY_STATUS_UNKNOWN;
    status_initialized = 0U;
    switches_initialized = 0U;
    blue = 1U;
    large = 0U;
}

void EnergyMechanismStartReceive(void)
{
    rx_index = 0U;
    (void)HAL_UART_Receive_IT(&huart2, &energy_rx_byte, 1U);
}

void EnergyMechanismRxByte(uint8_t byte)
{
    if (rx_index == 0U)
    {
        if (byte == ENERGY_FRAME_HEAD_0) rx_frame[rx_index++] = byte;
        return;
    }
    if (rx_index == 1U)
    {
        if (byte == ENERGY_FRAME_HEAD_1)
        {
            rx_frame[rx_index++] = byte;
        }
        else
        {
            rx_index = (byte == ENERGY_FRAME_HEAD_0) ? 1U : 0U;
            if (rx_index != 0U) rx_frame[0] = byte;
        }
        return;
    }

    rx_frame[rx_index++] = byte;
    if (rx_index < ENERGY_FRAME_SIZE) return;

    if (rx_frame[2] == 0x11U && rx_frame[3] == 0x10U)
    {
        status = ENERGY_STATUS_INACTIVE;
        status_initialized = 1U;
    }
    else if (rx_frame[2] == 0x12U && rx_frame[3] == 0x11U)
    {
        status = ENERGY_STATUS_ACTIVATING;
        status_initialized = 1U;
    }
    else if (rx_frame[2] == 0x13U && rx_frame[3] == 0x12U)
    {
        status = ENERGY_STATUS_ACTIVE;
        status_initialized = 1U;
    }
    rx_index = 0U;
}

void EnergyMechanismUpdateSwitches(uint8_t color_switch,
                                   uint8_t size_switch,
                                   uint8_t rune_switch,
                                   uint8_t motors_enabled)
{
    uint8_t color_changed;
    uint8_t size_changed;
    uint8_t rune_changed;

    if (!switches_initialized)
    {
        color_switch_last = color_switch;
        size_switch_last = size_switch;
        rune_switch_last = rune_switch;
        switches_initialized = 1U;
        if (switch_is_up(color_switch)) blue = 1U;
        else if (switch_is_down(color_switch)) blue = 0U;
        if (switch_is_up(size_switch)) large = 0U;
        else if (switch_is_down(size_switch)) large = 1U;
        return;
    }

    color_changed = (uint8_t)(IsTwoPosition(color_switch) &&
                              IsTwoPosition(color_switch_last) &&
                              color_switch != color_switch_last);
    size_changed = (uint8_t)(IsTwoPosition(size_switch) &&
                             IsTwoPosition(size_switch_last) &&
                             size_switch != size_switch_last);
    rune_changed = (uint8_t)(rune_switch != rune_switch_last);
    color_switch_last = color_switch;
    size_switch_last = size_switch;
    rune_switch_last = rune_switch;

    if (switch_is_up(color_switch)) blue = 1U;
    else if (switch_is_down(color_switch)) blue = 0U;
    if (switch_is_up(size_switch)) large = 0U;
    else if (switch_is_down(size_switch)) large = 1U;


    (void)motors_enabled;
    if (size_changed) SendCommand(ENERGY_CMD_MODE, 0x00U);
    if (color_changed) SendCommand(ENERGY_CMD_COLOR, 0x01U);
    if (rune_changed)
    {
        if (switch_is_up(rune_switch)) SendCommand(ENERGY_CMD_RESET, 0x03U);
        else if (switch_is_down(rune_switch)) SendCommand(ENERGY_CMD_ACTIVATE, 0x02U);
    }
}

EnergyMechanismStatus_e EnergyMechanismGetStatus(void)
{
    return status_initialized ? status : ENERGY_STATUS_UNKNOWN;
}

uint8_t EnergyMechanismIsLarge(void)
{
    return large;
}

uint8_t EnergyMechanismIsBlue(void)
{
    return blue;
}
