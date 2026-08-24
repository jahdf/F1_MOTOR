#include "remote_control.h"

#include "main.h"
#include "string.h"

#define REMOTE_CONTROL_FRAME_SIZE 18U
#define REMOTE_CONTROL_TIMEOUT_MS 100U

static RC_ctrl_t rc_data[2];
static uint8_t rc_frame[REMOTE_CONTROL_FRAME_SIZE];
static uint8_t rc_frame_index;
static uint8_t rc_initialized;
static uint32_t rc_last_frame_tick;

static uint16_t DecodeChannel0(const uint8_t *frame)
{
    return (uint16_t)((frame[0] | (frame[1] << 8U)) & 0x07FFU);
}

static uint16_t DecodeChannel1(const uint8_t *frame)
{
    return (uint16_t)(((frame[1] >> 3U) | (frame[2] << 5U)) & 0x07FFU);
}

static uint16_t DecodeChannel2(const uint8_t *frame)
{
    return (uint16_t)(((frame[2] >> 6U) | (frame[3] << 2U) |
                       (frame[4] << 10U)) & 0x07FFU);
}

static uint16_t DecodeChannel3(const uint8_t *frame)
{
    return (uint16_t)(((frame[4] >> 1U) | (frame[5] << 7U)) & 0x07FFU);
}

static uint16_t DecodeDial(const uint8_t *frame)
{
    return (uint16_t)((frame[16] | (frame[17] << 8U)) & 0x07FFU);
}

static uint8_t FrameLooksValid(const uint8_t *frame)
{
    uint16_t channels[5];
    uint8_t switch_right;
    uint8_t switch_left;

    channels[0] = DecodeChannel0(frame);
    channels[1] = DecodeChannel1(frame);
    channels[2] = DecodeChannel2(frame);
    channels[3] = DecodeChannel3(frame);
    channels[4] = DecodeDial(frame);
    switch_right = (uint8_t)((frame[5] >> 4U) & 0x03U);
    switch_left = (uint8_t)((frame[5] >> 4U) & 0x0CU) >> 2U;

    for (uint8_t channel_index = 0U; channel_index < 5U; channel_index++)
    {
        if (channels[channel_index] < RC_CH_VALUE_MIN ||
            channels[channel_index] > RC_CH_VALUE_MAX)
        {
            return 0U;
        }
    }

    return ((switch_is_up(switch_right) || switch_is_mid(switch_right) ||
             switch_is_down(switch_right)) &&
            (switch_is_up(switch_left) || switch_is_mid(switch_left) ||
             switch_is_down(switch_left)));
}

static void RectifyJoysticks(RC_ctrl_t *decoded)
{
    int16_t *joystick = &decoded->rc.rocker_l_;

    for (uint8_t joystick_index = 0U; joystick_index < 5U; joystick_index++)
    {
        if (joystick[joystick_index] > 660 || joystick[joystick_index] < -660)
        {
            joystick[joystick_index] = 0;
        }
    }
}

static void DecodeFrame(const uint8_t *frame)
{
    RC_ctrl_t decoded;
    uint16_t key_now;
    uint16_t key_last;
    uint16_t key_with_ctrl;
    uint16_t key_with_shift;
    uint16_t key_last_with_ctrl;
    uint16_t key_last_with_shift;

    memcpy(&decoded, &rc_data[TEMP], sizeof(decoded));
    decoded.rc.rocker_r_ = (int16_t)DecodeChannel0(frame) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_r1 = (int16_t)DecodeChannel1(frame) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_l_ = (int16_t)DecodeChannel2(frame) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_l1 = (int16_t)DecodeChannel3(frame) - RC_CH_VALUE_OFFSET;
    decoded.rc.dial = (int16_t)DecodeDial(frame) - RC_CH_VALUE_OFFSET;
    decoded.rc.switch_right = (uint8_t)((frame[5] >> 4U) & 0x03U);
    decoded.rc.switch_left = (uint8_t)(((frame[5] >> 4U) & 0x0CU) >> 2U);
    decoded.mouse.x = (int16_t)(frame[6] | (frame[7] << 8U));
    decoded.mouse.y = (int16_t)(frame[8] | (frame[9] << 8U));
    decoded.mouse.z = (int16_t)(frame[10] | (frame[11] << 8U));
    decoded.mouse.press_l = frame[12];
    decoded.mouse.press_r = frame[13];
    decoded.key[KEY_PRESS].keys = (uint16_t)(frame[14] | (frame[15] << 8U));

    if (decoded.key[KEY_PRESS].ctrl)
    {
        decoded.key[KEY_PRESS_WITH_CTRL] = decoded.key[KEY_PRESS];
    }
    else
    {
        memset(&decoded.key[KEY_PRESS_WITH_CTRL], 0, sizeof(Key_t));
    }

    if (decoded.key[KEY_PRESS].shift)
    {
        decoded.key[KEY_PRESS_WITH_SHIFT] = decoded.key[KEY_PRESS];
    }
    else
    {
        memset(&decoded.key[KEY_PRESS_WITH_SHIFT], 0, sizeof(Key_t));
    }

    key_now = decoded.key[KEY_PRESS].keys;
    key_last = rc_data[LAST].key[KEY_PRESS].keys;
    key_with_ctrl = decoded.key[KEY_PRESS_WITH_CTRL].keys;
    key_with_shift = decoded.key[KEY_PRESS_WITH_SHIFT].keys;
    key_last_with_ctrl = rc_data[LAST].key[KEY_PRESS_WITH_CTRL].keys;
    key_last_with_shift = rc_data[LAST].key[KEY_PRESS_WITH_SHIFT].keys;

    for (uint16_t key_index = 0U, key_mask = 1U; key_index < 16U;
         key_index++, key_mask <<= 1U)
    {
        if (key_index == 4U || key_index == 5U)
        {
            continue;
        }
        if ((key_now & key_mask) && !(key_last & key_mask) &&
            !(key_with_ctrl & key_mask) && !(key_with_shift & key_mask))
        {
            decoded.key_count[KEY_PRESS][key_index]++;
        }
        if ((key_with_ctrl & key_mask) && !(key_last_with_ctrl & key_mask))
        {
            decoded.key_count[KEY_PRESS_WITH_CTRL][key_index]++;
        }
        if ((key_with_shift & key_mask) && !(key_last_with_shift & key_mask))
        {
            decoded.key_count[KEY_PRESS_WITH_SHIFT][key_index]++;
        }
    }

    RectifyJoysticks(&decoded);
    memcpy(&rc_data[TEMP], &decoded, sizeof(decoded));
    memcpy(&rc_data[LAST], &rc_data[TEMP], sizeof(rc_data[TEMP]));
    rc_last_frame_tick = HAL_GetTick();
}

void RemoteControlInit(void)
{
    memset(rc_data, 0, sizeof(rc_data));
    rc_frame_index = 0U;
    rc_last_frame_tick = 0U;
    rc_initialized = 1U;
}

void RemoteControlRxByte(uint8_t byte)
{
    if (rc_frame_index < REMOTE_CONTROL_FRAME_SIZE)
    {
        rc_frame[rc_frame_index++] = byte;
    }

    if (rc_frame_index == REMOTE_CONTROL_FRAME_SIZE)
    {
        if (FrameLooksValid(rc_frame))
        {
            DecodeFrame(rc_frame);
            rc_frame_index = 0U;
        }
        else
        {
            rc_frame_index = 0U;
        }
    }
}

void RemoteControlRxIdle(void)
{
    rc_frame_index = 0U;
}

const RC_ctrl_t *RemoteControlGetData(void)
{
    return &rc_data[TEMP];
}

uint8_t RemoteControlIsOnline(void)
{
    return (uint8_t)(rc_initialized &&
                     ((HAL_GetTick() - rc_last_frame_tick) <=
                      REMOTE_CONTROL_TIMEOUT_MS));
}
