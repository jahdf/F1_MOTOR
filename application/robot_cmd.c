#include "robot_cmd.h"

#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"

#define RC_DIAL_DISABLE_THRESHOLD 300
#define GIMBAL_SPEED_SCALE (5.0f / 660.0f)
#define GIMBAL_ROTATE_SPEED 300.0f

static const RC_ctrl_t *remote_data;
static uint8_t motor_enabled;
static uint8_t right_switch_was_up;

static uint8_t RemoteInputIsValid(void)
{
    if (!RemoteControlIsOnline())
    {
        return 0U;
    }

    if (remote_data == NULL)
    {
        return 0U;
    }

    return (uint8_t)((switch_is_up(remote_data->rc.switch_left) ||
                      switch_is_mid(remote_data->rc.switch_left) ||
                      switch_is_down(remote_data->rc.switch_left)) &&
                     (switch_is_up(remote_data->rc.switch_right) ||
                      switch_is_mid(remote_data->rc.switch_right) ||
                      switch_is_down(remote_data->rc.switch_right)));
}

static uint8_t RightSwitchChangedFromUpToMid(void)
{
    uint8_t current_switch = remote_data->rc.switch_right;

    if (switch_is_up(current_switch))
    {
        right_switch_was_up = 1U;
    }
    else if (switch_is_mid(current_switch) && right_switch_was_up)
    {
        right_switch_was_up = 0U;
        return 1U;
    }
    else if (switch_is_down(current_switch))
    {
        right_switch_was_up = 0U;
    }

    return 0U;
}

void RobotCmdInit(void)
{
    RemoteControlInit();
    remote_data = RemoteControlGetData();
    motor_enabled = 0U;
    right_switch_was_up = 0U;
    ChassisSetEnabled(0U);
    GimbalSetEnabled(0U);
}

void RobotCmdTask(void)
{
    if (!RemoteInputIsValid())
    {
        motor_enabled = 0U;
        right_switch_was_up = 0U;
        ChassisSetEnabled(0U);
        GimbalSetReference(0.0f);
        GimbalSetEnabled(0U);
        return;
    }

    if (remote_data->rc.dial > RC_DIAL_DISABLE_THRESHOLD)
    {
        motor_enabled = 0U;
        right_switch_was_up = 0U;
    }
    else if (RightSwitchChangedFromUpToMid())
    {
        motor_enabled = 1U;
    }

    if (!motor_enabled)
    {
        ChassisSetEnabled(0U);
        GimbalSetReference(0.0f);
        GimbalSetEnabled(0U);
        return;
    }

    ChassisSetEnabled(1U);
    GimbalSetEnabled(1U);
    if (switch_is_down(remote_data->rc.switch_right))
    {
        GimbalSetReference(GIMBAL_ROTATE_SPEED);
    }
    else
    {
        GimbalSetReference(-GIMBAL_SPEED_SCALE * remote_data->rc.rocker_l_);
    }
}

uint8_t RobotCmdMotorEnabled(void)
{
    return motor_enabled;
}
