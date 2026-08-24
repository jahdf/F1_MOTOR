//
// Created by zgb on 2026/5/31.
//
#include "chassis.h"
#include "general_def.h"
#include "bsp_dwt.h"
#include "arm_math.h"
#include "dji_motor.h"
#include "user_lib.h"
#include "motor_def.h"
static DJIMotorInstance *motor_lf, *motor_rf, *motor_lb, *motor_rb;

void ChassisInit(Motor_Type_e motor_type) {
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan,
        .controller_param_init_config = {
            .speed_PID = {
                .Kp = 0.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 30000,
        },
        .current_PID = {
            .Kp = 0.0f,
            .Ki = 0.0f,
            .Kd = 0.0f,
            .IntegralLimit = 3000,
            .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
            .MaxOut = 15000,
    },
},
.controller_setting_init_config = {
    .angle_feedback_source = MOTOR_FEED,
    .speed_feedback_source = MOTOR_FEED,
    .outer_loop_type = SPEED_LOOP,
    .close_loop_type = SPEED_LOOP, // CURRENT_LOOP,
    .power_limit_flag = POWER_LIMIT_ON,
},
.motor_type = motor_type,
.motor_control_type =  CURRENT_CONTROL
};
    //右前
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 1.0f;
    chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.001f;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    //左前
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 1.0f;
    chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.001f;
    motor_lf = DJIMotorInit(&chassis_motor_config);
    //左后
    chassis_motor_config.can_init_config.tx_id = 3;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 1.0f;
    chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.001f;
    motor_lb = DJIMotorInit(&chassis_motor_config);
    //右后
    chassis_motor_config.can_init_config.tx_id = 4;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    chassis_motor_config.controller_param_init_config.speed_PID.Kp = 1.0f;
    chassis_motor_config.controller_param_init_config.speed_PID.Ki = 0;
    chassis_motor_config.controller_param_init_config.speed_PID.Kd = 0.001f;
    motor_rb = DJIMotorInit(&chassis_motor_config);
}
 float speed = 2765;
void ChassisSetRef(void)
{
    DJIMotorSetRef(motor_lf, speed);
    DJIMotorSetRef(motor_rf, speed);
    DJIMotorSetRef(motor_lb, speed);
    DJIMotorSetRef(motor_rb, speed);
}
