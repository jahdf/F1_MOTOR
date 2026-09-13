#include "chassis.h"

#include "bsp_log.h"
#include "can.h"
#include "dji_motor.h"
#include "general_def.h"
#include "message_center.h"

static DJIMotorInstance *motor_lf;
static uint8_t chassis_enabled;
static uint8_t chassis_feedback_blocked;
static Subscriber_t *chassis_cmd_sub;
static Chassis_Cmd_t chassis_cmd;

static uint8_t ChassisMotorFeedbackValid(void)
{
    return (uint8_t)(motor_lf != NULL &&
                     motor_lf->dt > 0.0f &&
                     motor_lf->lost_flag == 0U);
}

void ChassisInit(void)
{
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config = {
            .can_handle = &hcan,
            .tx_id = 1U,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.8f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .Kf = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral |
                           PID_Integral_Limit |
                           PID_Derivative_On_Measurement |
                           PID_FeedForward,
                .MaxOut = 30000,
            },
            .current_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral |
                           PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
                .MaxOut = 15000,
            },
        },
        .controller_setting_init_config = {
            .angle_feedback_source = MOTOR_FEED,
            .speed_feedback_source = MOTOR_FEED,
            .outer_loop_type = SPEED_LOOP,
            .close_loop_type = SPEED_LOOP,
            .motor_reverse_flag = MOTOR_DIRECTION_NORMAL,
            .power_limit_flag = POWER_LIMIT_ON,
        },
        .motor_type = M3508,
        .motor_control_type = CURRENT_CONTROL,
    };

    motor_lf = DJIMotorInit(&chassis_motor_config);
    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Cmd_t));
    chassis_cmd = (Chassis_Cmd_t){0};
    chassis_feedback_blocked = 0U;
    ChassisSetEnabled(0U);

    if (motor_lf != NULL && chassis_cmd_sub != NULL)
    {
        LOGINFO("[chassis] Initialized: M3508 ID 1, motor disabled");
    }
    else
    {
        LOGERROR("[chassis] Initialization failed");
    }
}

void ChassisSetEnabled(uint8_t enabled)
{
    chassis_enabled = enabled ? 1U : 0U;
    if (motor_lf == NULL) return;

    if (chassis_enabled)
    {
        DJIMotorEnable(motor_lf);
    }
    else
    {
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorStop(motor_lf);
        chassis_feedback_blocked = 0U;
    }
}

void ChassisSetRef(float chassis_cmd_wz)
{
    chassis_cmd.wz = chassis_cmd_wz;
}

void ChassisTask(void)
{
    if (chassis_cmd_sub != NULL)
    {
        (void)SubGetMessage(chassis_cmd_sub, &chassis_cmd);
        if ((chassis_cmd.enabled ? 1U : 0U) != chassis_enabled)
        {
            ChassisSetEnabled(chassis_cmd.enabled);
        }
    }

    if (motor_lf == NULL) return;
    if (!chassis_cmd.enabled || !chassis_enabled)
    {
        DJIMotorSetRef(motor_lf, 0.0f);
        return;
    }

    if (!ChassisMotorFeedbackValid())
    {
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorStop(motor_lf);
        if (!chassis_feedback_blocked)
        {
            chassis_feedback_blocked = 1U;
            LOGWARNING("[chassis] Encoder feedback unavailable, output stopped");
        }
        return;
    }

    if (chassis_feedback_blocked)
    {
        chassis_feedback_blocked = 0U;
        DJIMotorEnable(motor_lf);
        LOGINFO("[chassis] Encoder feedback recovered");
    }
    //19.2电机减速比，3是齿轮的传动比
    DJIMotorSetRef(motor_lf, chassis_cmd.wz * RAD_2_DEGREE * 19.2 * 3);
}
