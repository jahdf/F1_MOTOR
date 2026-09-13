#include "robot_cmd.h"

#include "chassis.h"
#include "energy_mechanism.h"
#include "remote_control.h"
#include "message_center.h"
#include "bsp_log.h"
#include "general_def.h"
#include <math.h>

static const RC_ctrl_t *remote_data;
static uint8_t motor_enabled;
static uint8_t arm_ready;
static Publisher_t *chassis_cmd_pub;
static Chassis_Cmd_t chassis_cmd_msg;
static EnergyMechanismStatus_e energy_last_status;
static float energy_a;
static float energy_omega;
static uint32_t energy_start_tick;
static uint32_t energy_random_state = 0x13579BDFU;

#define ENERGY_FIXED_SPEED_RAD_S (PI / 3.0f)
#define ENERGY_A_MIN 0.780f
#define ENERGY_A_MAX 1.045f
#define ENERGY_OMEGA_MIN 1.884f
#define ENERGY_OMEGA_MAX 2.000f
#define ENERGY_B 2.090f

static float EnergyRandomRange(float min_value, float max_value)
{
    energy_random_state = energy_random_state * 1664525U + 1013904223U;
    return min_value + (max_value - min_value) *
           ((float)(energy_random_state & 0x00ffffffU) / 16777215.0f);
}

static float EnergySpeedWz(void)
{
    EnergyMechanismStatus_e status = EnergyMechanismGetStatus();


    if (status == ENERGY_STATUS_UNKNOWN)
    {
        status = ENERGY_STATUS_INACTIVE;
    }

    if (status != energy_last_status)
    {
        if (status == ENERGY_STATUS_ACTIVATING)
        {
            energy_a = EnergyRandomRange(ENERGY_A_MIN, ENERGY_A_MAX);
            energy_omega = EnergyRandomRange(ENERGY_OMEGA_MIN, ENERGY_OMEGA_MAX);
            energy_start_tick = HAL_GetTick();
        }
        energy_last_status = status;
    }

    float speed = ENERGY_FIXED_SPEED_RAD_S;
    if (EnergyMechanismIsLarge() && status == ENERGY_STATUS_ACTIVATING)
    {
        float elapsed_s = (float)(HAL_GetTick() - energy_start_tick) * 0.001f;
        speed = energy_a * sinf(energy_omega * elapsed_s) +
                (ENERGY_B - energy_a);
    }
    return EnergyMechanismIsBlue() ? speed : -speed;
}

static void SetMotorEnabled(uint8_t enabled)
{
    uint8_t requested_state = enabled ? 1U : 0U;
    if (requested_state == motor_enabled) return;
    motor_enabled = requested_state;
    if (motor_enabled) LOGINFO("[robot_cmd] Motors enabled");
    else LOGINFO("[robot_cmd] Motors disabled");
}

static uint8_t RemoteInputIsValid(void)
{
    return (uint8_t)(remote_data != NULL &&
                     RemoteControlIsOnline() &&
                     RemoteControlFrameValid());
}

static void PublishDisabledCommand(void)
{
    chassis_cmd_msg = (Chassis_Cmd_t){0};
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_msg);
}

void RobotCmdInit(void)
{
    EnergyMechanismInit();
    RemoteControlInit();
    remote_data = RemoteControlGetData();
    motor_enabled = 0U;
    arm_ready = 0U;
    energy_last_status = ENERGY_STATUS_UNKNOWN;
    energy_a = ENERGY_A_MIN;
    energy_omega = ENERGY_OMEGA_MIN;
    energy_start_tick = HAL_GetTick();
    chassis_cmd_pub = PubRegister("chassis_cmd", sizeof(Chassis_Cmd_t));
    LOGINFO("[robot_cmd] Initialized; motors disabled");
}

void RobotCmdTask(void)
{
    if (!RemoteInputIsValid())
    {
        SetMotorEnabled(0U);
        arm_ready = 0U;
        ChassisSetEnabled(0U);
        PublishDisabledCommand();
        return;
    }

    if (!switch_is_down(remote_data->rc.switch_enable))
    {
        SetMotorEnabled(0U);
        arm_ready = 1U;
    }
    else if (arm_ready)
    {
        SetMotorEnabled(1U);
    }

    EnergyMechanismUpdateSwitches(remote_data->rc.switch_cloor,
                                  remote_data->rc.switch_energy_mode,
                                  remote_data->rc.switch_big,
                                  motor_enabled);

    if (!motor_enabled)
    {
        ChassisSetEnabled(0U);
        PublishDisabledCommand();
        return;
    }


    chassis_cmd_msg.wz =  EnergySpeedWz();
    chassis_cmd_msg.enabled = 1U;
    chassis_cmd_msg.control_mode = CHASSIS_MODE_JOYSTICK;
    PubPushMessage(chassis_cmd_pub, &chassis_cmd_msg);
}

uint8_t RobotCmdMotorEnabled(void)
{
    return motor_enabled;
}
