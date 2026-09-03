#include "gimbal.h"

#include "can.h"
#include "dji_motor.h"
#include "general_def.h"
#include "motor_def.h"
#include "message_center.h"
#include "bsp_log.h"

static DJIMotorInstance *gimbal_motor;
static float gimbal_reference;
static uint8_t gimbal_enabled;
static Subscriber_t *gimbal_cmd_sub;
static Gimbal_Cmd_t gimbal_cmd;

void GimbalInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan,
            .tx_id = 7U,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 22.0f,
                .Ki = 0.5f,
                .Kd = 0.001f,
                .Kf = 0.0f,
                .IntegralLimit = 300,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement |FF_Proportional,
                .MaxOut = 30000,
            },
            .current_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
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
        .motor_type = GM6020,
        .motor_control_type = CURRENT_CONTROL,
    };

    gimbal_motor = DJIMotorInit(&config);
    gimbal_cmd_sub = SubRegister("gimbal_cmd", sizeof(Gimbal_Cmd_t));
    gimbal_cmd.wz_rad_s = 0.0f;
    gimbal_cmd.enabled = 0U;
    gimbal_reference = 0.0f;
    gimbal_enabled = 0U;
    if (gimbal_motor != NULL)
    {
        DJIMotorSetRef(gimbal_motor, 0.0f);
        DJIMotorStop(gimbal_motor);
    }
    if (gimbal_motor != NULL && gimbal_cmd_sub != NULL)
    {
        LOGINFO("[gimbal] Initialized: GM6020 ID 5, motor disabled");
    }
    else
    {
        LOGERROR("[gimbal] Initialization failed");
    }
}

void GimbalSetRef(float speed_ref)
{
    gimbal_reference = speed_ref;
}

void GimbalSetEnabled(uint8_t enabled)
{
    gimbal_enabled = enabled ? 1U : 0U;
    if (!gimbal_enabled && gimbal_motor != NULL)
    {
        gimbal_reference = 0.0f;
        PIDReset(&gimbal_motor->motor_controller.speed_PID);
        DJIMotorSetRef(gimbal_motor, 0.0f);
        DJIMotorStop(gimbal_motor);
    }
}

void GimbalTask(void)
{
    if (gimbal_cmd_sub != NULL)
    {
        (void)SubGetMessage(gimbal_cmd_sub, &gimbal_cmd);
        gimbal_reference = gimbal_cmd.wz_rad_s;
        gimbal_enabled = gimbal_cmd.enabled;
    }
    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_enabled)
    {
        DJIMotorEnable(gimbal_motor);
        DJIMotorSetRef(gimbal_motor, gimbal_reference * RAD_2_DEGREE);
    }
    else
    {
        DJIMotorSetRef(gimbal_motor, 0.0f);
        PIDReset(&gimbal_motor->motor_controller.speed_PID);
        DJIMotorStop(gimbal_motor);
    }
}
