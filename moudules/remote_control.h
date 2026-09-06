#ifndef TEXT1_REMOTE_CONTROL_H
#define TEXT1_REMOTE_CONTROL_H

#include <stdint.h>

#define LAST 1U
#define TEMP 0U

#define KEY_PRESS 0U
#define KEY_STATE 1U
#define KEY_PRESS_WITH_CTRL 1U
#define KEY_PRESS_WITH_SHIFT 2U

#define RC_CH_VALUE_MIN ((uint16_t)1000U)
#define RC_CH_VALUE_OFFSET ((uint16_t)1500U)
#define RC_CH_VALUE_MAX ((uint16_t)2000U)
#define RC_IBUS_CHANNEL_COUNT 10U

#define RC_SW_UP ((uint16_t)1U)
#define RC_SW_MID ((uint16_t)3U)
#define RC_SW_DOWN ((uint16_t)2U)

#define switch_is_down(value) ((value) == RC_SW_DOWN)
#define switch_is_mid(value) ((value) == RC_SW_MID)
#define switch_is_up(value) ((value) == RC_SW_UP)

typedef union
{
    struct
    {
        uint16_t w : 1;
        uint16_t s : 1;
        uint16_t d : 1;
        uint16_t a : 1;
        uint16_t shift : 1;
        uint16_t ctrl : 1;
        uint16_t q : 1;
        uint16_t e : 1;
        uint16_t r : 1;
        uint16_t f : 1;
        uint16_t g : 1;
        uint16_t z : 1;
        uint16_t x : 1;
        uint16_t c : 1;
        uint16_t v : 1;
        uint16_t b : 1;
    };
    uint16_t keys;
} Key_t;

typedef struct
{
    struct
    {
        int16_t rocker_l_;
        int16_t rocker_l1;
        int16_t rocker_r_;
        int16_t rocker_r1;
        int16_t dial;
        uint16_t knob_chassis;
        uint16_t knob_gimbal;
        uint8_t switch_enable;
        uint8_t switch_cloor;
        uint8_t switch_big;
        uint8_t switch_energy_mode;
    } rc;
    struct
    {
        int16_t x;
        int16_t y;
        int16_t z;
        uint8_t press_l;
        uint8_t press_r;
    } mouse;
    Key_t key[3];
    uint8_t key_count[3][16];
} RC_ctrl_t;

void RemoteControlInit(void);
void RemoteControlRxByte(uint8_t byte);
void RemoteControlRxIdle(void);
void RemoteControlStartReceive(void);
const RC_ctrl_t *RemoteControlGetData(void);
uint8_t RemoteControlIsOnline(void);
uint8_t RemoteControlFrameValid(void);

#endif
