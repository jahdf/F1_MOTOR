#include "remote_control.h"

#include "main.h"
#include "usart.h"
#include "bsp_log.h"
#include "string.h"

#define REMOTE_CONTROL_FRAME_SIZE 32U
#define REMOTE_CONTROL_CHANNEL_COUNT RC_IBUS_CHANNEL_COUNT
#define REMOTE_CONTROL_TIMEOUT_MS 100U

static RC_ctrl_t rc_data[2];
static uint8_t rc_frame[REMOTE_CONTROL_FRAME_SIZE];
static uint8_t rc_frame_index;
static uint8_t rc_initialized;
static uint32_t rc_last_frame_tick;

static uint8_t rc_rx_byte;
static uint8_t rc_frame_state;
static uint8_t rc_frame_valid;

static uint16_t DecodeIbusChannel(const uint8_t *frame, uint8_t channel)
{
    uint8_t offset = (uint8_t)(2U + channel * 2U);
    return (uint16_t)(frame[offset] | ((uint16_t)frame[offset + 1U] << 8U));
}

static uint8_t DecodeSwitch(uint16_t value)
{
    if (value < 1300U) return RC_SW_UP;
    if (value > 1700U) return RC_SW_DOWN;
    return RC_SW_MID;
}

static uint8_t IbusFrameValid(const uint8_t *frame)
{
    uint16_t checksum = 0xffffU;
    uint16_t received;

    if (frame[0] != 0x20U || frame[1] != 0x40U) return 0U;
    for (uint8_t i = 0U; i < 30U; i++) checksum = (uint16_t)(checksum - frame[i]);
    received = (uint16_t)(frame[30] | ((uint16_t)frame[31] << 8U));
    return (uint8_t)(checksum == received);
}

static uint8_t FrameLooksValid(const uint8_t *frame)
{
    if (!IbusFrameValid(frame)) return 0U;
    for (uint8_t channel_index = 0U; channel_index < REMOTE_CONTROL_CHANNEL_COUNT; channel_index++)
    {
        uint16_t value = DecodeIbusChannel(frame, channel_index);
        if (value < 800U || value > 2200U)
        {
            return 0U;
        }
    }
    return 1U;
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
    decoded.rc.rocker_r_ = (int16_t)DecodeIbusChannel(frame, 0U) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_r1 = (int16_t)DecodeIbusChannel(frame, 1U) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_l_ = (int16_t)DecodeIbusChannel(frame, 2U) - RC_CH_VALUE_OFFSET;
    decoded.rc.rocker_l1 = (int16_t)DecodeIbusChannel(frame, 3U) - RC_CH_VALUE_OFFSET;
    decoded.rc.knob_chassis = DecodeIbusChannel(frame, 4U);
    decoded.rc.knob_gimbal = DecodeIbusChannel(frame, 5U);
    decoded.rc.dial = (int16_t)decoded.rc.knob_chassis - RC_CH_VALUE_OFFSET;
    decoded.rc.switch_chassis_mode = DecodeSwitch(DecodeIbusChannel(frame, 6U));//通道SWA设置底盘模式
    decoded.rc.switch_enable = DecodeSwitch(DecodeIbusChannel(frame, 7U));// CH8, 两档使能
    decoded.rc.switch_gimbal_speed = DecodeSwitch(DecodeIbusChannel(frame, 8U));// CH9, 云台速度档位
    decoded.rc.switch_gimbal_speed_enable = DecodeSwitch(DecodeIbusChannel(frame, 9U));// CH10, CH9使能
    memset(&decoded.mouse, 0, sizeof(decoded.mouse));
    memset(&decoded.key, 0, sizeof(decoded.key));

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
    rc_frame_state = 0U;
    rc_frame_valid = 0U;
    LOGINFO("[remote] i-BUS initialized: USART1, 115200 8N1, 10 channels");
}

void RemoteControlStartReceive(void)
{
    rc_frame_state = 0U;
    HAL_UART_Receive_IT(&huart1, &rc_rx_byte, 1U);
}

void RemoteControlRxByte(uint8_t byte)
{
    if (rc_frame_state == 0U)
    {
        if (byte != 0x20U) return;
        rc_frame[0] = byte;
        rc_frame_index = 1U;
        rc_frame_state = 1U;
        return;
    }

    if (rc_frame_index < REMOTE_CONTROL_FRAME_SIZE) rc_frame[rc_frame_index++] = byte;
    if (rc_frame_index == REMOTE_CONTROL_FRAME_SIZE)
    {
        if (FrameLooksValid(rc_frame))
        {
            DecodeFrame(rc_frame);
            rc_frame_valid = 1U;
        }
        else
        {
            rc_frame_valid = 0U;
        }
        rc_frame_index = 0U;
        rc_frame_state = 0U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART1)
    {
        RemoteControlRxByte(rc_rx_byte);
        HAL_UART_Receive_IT(&huart1, &rc_rx_byte, 1U);
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
                     rc_frame_valid &&
                     ((HAL_GetTick() - rc_last_frame_tick) <=
                      REMOTE_CONTROL_TIMEOUT_MS));
}

uint8_t RemoteControlFrameValid(void)
{
    return rc_frame_valid;
}
