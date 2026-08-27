#include "robot_cmd.h"

#include "chassis.h"
#include "gimbal.h"
#include "remote_control.h"
#include "message_center.h"
#include "bsp_log.h"

#define RC_CHANNEL_SCALE (1.0f / 500.0f)
#define CHASSIS_SPEED_MAX 5.0f
#define CHASSIS_KNOB_ZERO_MAX 1050U
#define GIMBAL_SPEED_MIN 1.0f
#define GIMBAL_SPEED_MAX 5.0f
#define ARM_STICK_DEADBAND 50

static const RC_ctrl_t *remote_data;
static uint8_t motor_enabled;
static uint8_t arm_ready;
static float chassis_cmd_vx;
static float gimbal_cmd_wz;
static Publisher_t *chassis_cmd_pub;
static Publisher_t *gimbal_cmd_pub;
static Chassis_Cmd_t chassis_cmd_msg;
static Gimbal_Cmd_t gimbal_cmd_msg;

static void SetMotorEnabled(uint8_t enabled)
{
    uint8_t requested_state = enabled ? 1U : 0U;
    if (requested_state == motor_enabled)
    {
        return;
    }

    motor_enabled = requested_state;
    if (motor_enabled)
    {
        LOGINFO("[robot_cmd] Motors enabled");
    }
    else
    {
        LOGINFO("[robot_cmd] Motors disabled");
    }
}

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

    return RemoteControlFrameValid();
}

static float KnobToGimbalSpeed(uint16_t knob)
{
    if (knob < RC_CH_VALUE_MIN) knob = RC_CH_VALUE_MIN;
    if (knob > RC_CH_VALUE_MAX) knob = RC_CH_VALUE_MAX;
    return GIMBAL_SPEED_MIN + (GIMBAL_SPEED_MAX - GIMBAL_SPEED_MIN) *
           (float)(knob - RC_CH_VALUE_MIN) /
           (float)(RC_CH_VALUE_MAX - RC_CH_VALUE_MIN);
}

static float KnobToChassisSpeed(uint16_t knob)
{
    if (knob <= CHASSIS_KNOB_ZERO_MAX)
    {
        return 0.0f;
    }
    if (knob > RC_CH_VALUE_MAX) knob = RC_CH_VALUE_MAX;
    return CHASSIS_SPEED_MAX *
           (float)(knob - CHASSIS_KNOB_ZERO_MAX) /
           (float)(RC_CH_VALUE_MAX - CHASSIS_KNOB_ZERO_MAX);
}

static float StickToChassisSpeed(int16_t stick)
{
    if (stick >= -ARM_STICK_DEADBAND && stick <= ARM_STICK_DEADBAND)
    {
        return 0.0f;
    }
    if (stick > 500) stick = 500;
    if (stick < -500) stick = -500;
    return (float)stick * RC_CHANNEL_SCALE * CHASSIS_SPEED_MAX;
}

static uint8_t DriveStickIsCentered(void)
{
    return (uint8_t)(remote_data->rc.rocker_r_ >= -ARM_STICK_DEADBAND &&
                     remote_data->rc.rocker_r_ <= ARM_STICK_DEADBAND &&
                     remote_data->rc.rocker_r1 >= -ARM_STICK_DEADBAND &&
                     remote_data->rc.rocker_r1 <= ARM_STICK_DEADBAND);
}

void RobotCmdInit(void)
{
    RemoteControlInit();
    remote_data = RemoteControlGetData();
    motor_enabled = 0U;
    arm_ready = 0U;
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Cmd_t));
    gimbal_cmd_pub = PubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));
    LOGINFO("[robot_cmd] Initialized; motors disabled");
}

void RobotCmdTask(void)
{
    if (!RemoteInputIsValid())
    {
        SetMotorEnabled(0U);
        arm_ready = 0U;
        chassis_cmd_vx = 0.0f;
        gimbal_cmd_wz = 0.0f;
        chassis_cmd_msg = (Chassis_Cmd_t){0};
        gimbal_cmd_msg = (Gimbal_Cmd_t){0};
        PubPushMessage(chassis_cmd_pub, &chassis_cmd_msg);
        PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_msg);
        return;
    }

    /* SwC upper/middle are safe states; lower is the only enable request. */
    if (!switch_is_down(remote_data->rc.switch_enable))
    {
        SetMotorEnabled(0U);
        arm_ready = 1U;
    }
    else if (arm_ready && DriveStickIsCentered())
    {
        SetMotorEnabled(1U);
    }

    if (!motor_enabled)
    {
        chassis_cmd_vx = 0.0f;
        gimbal_cmd_wz = 0.0f;
        chassis_cmd_msg = (Chassis_Cmd_t){0};
        gimbal_cmd_msg = (Gimbal_Cmd_t){0};
        PubPushMessage(chassis_cmd_pub, &chassis_cmd_msg);
        PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_msg);
        return;
    }

    chassis_cmd_vx = 0.0f;
    float chassis_cmd_vy = 0.0f;
    chassis_cmd_msg.control_mode = CHASSIS_MODE_STOP;

    if (switch_is_down(remote_data->rc.switch_chassis_speed))
    {
        chassis_cmd_msg.control_mode = CHASSIS_MODE_JOYSTICK;
        chassis_cmd_vx = StickToChassisSpeed(remote_data->rc.rocker_r1);
        chassis_cmd_vy = StickToChassisSpeed(remote_data->rc.rocker_r_);
    }
    else if (switch_is_up(remote_data->rc.switch_chassis_speed))
    {
        chassis_cmd_msg.control_mode = CHASSIS_MODE_ODOMETRY;
        chassis_cmd_vx = KnobToChassisSpeed(remote_data->rc.knob_chassis);
    }

    gimbal_cmd_wz = KnobToGimbalSpeed(remote_data->rc.knob_gimbal);

    chassis_cmd_msg.vx_mps = chassis_cmd_vx;
    chassis_cmd_msg.vy_mps = chassis_cmd_vy;
    chassis_cmd_msg.enabled = 1U;
    gimbal_cmd_msg.wz_rad_s = gimbal_cmd_wz;
    gimbal_cmd_msg.enabled = 1U;
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_msg);
    PubPushMessage(gimbal_cmd_pub, &gimbal_cmd_msg);
}

uint8_t RobotCmdMotorEnabled(void)
{
    return motor_enabled;
}
