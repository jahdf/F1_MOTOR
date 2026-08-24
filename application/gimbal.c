#include "gimbal.h"

#include "can.h"
#include "dji_motor.h"
#include "motor_def.h"

static DJIMotorInstance *gimbal_motor;
static float gimbal_reference;
static uint8_t gimbal_enabled;

void GimbalInit(void)
{
    Motor_Init_Config_s config = {
        .can_init_config = {
            .can_handle = &hcan,
            .tx_id = 5U,
        },
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.001f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit |
                           PID_Derivative_On_Measurement,
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
        .motor_type = M3508,
        .motor_control_type = CURRENT_CONTROL,
    };

    gimbal_motor = DJIMotorInit(&config);
    gimbal_reference = 0.0f;
    gimbal_enabled = 0U;
    if (gimbal_motor != NULL)
    {
        DJIMotorSetRef(gimbal_motor, 0.0f);
        DJIMotorStop(gimbal_motor);
    }
}

void GimbalSetReference(float speed_ref)
{
    gimbal_reference = speed_ref;
}

void GimbalSetEnabled(uint8_t enabled)
{
    gimbal_enabled = enabled ? 1U : 0U;
    if (!gimbal_enabled && gimbal_motor != NULL)
    {
        gimbal_reference = 0.0f;
        DJIMotorSetRef(gimbal_motor, 0.0f);
        DJIMotorStop(gimbal_motor);
    }
}

void GimbalTask(void)
{
    if (gimbal_motor == NULL)
    {
        return;
    }

    if (gimbal_enabled)
    {
        DJIMotorEnable(gimbal_motor);
        DJIMotorSetRef(gimbal_motor, gimbal_reference);
    }
    else
    {
        DJIMotorSetRef(gimbal_motor, 0.0f);
        DJIMotorStop(gimbal_motor);
    }
}
