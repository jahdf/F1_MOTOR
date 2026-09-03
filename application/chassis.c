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
#include "message_center.h"
#include "bsp_log.h"
#include "slope_planner.h"
#include <math.h>

#define CHASSIS_WHEEL_RADIUS_M 0.0765f//轮半径
#define CHASSIS_REDUCTION_RATIO 19.0f//减速比
#define CHASSIS_ODOMETRY_LEG_M 1.3f//运行距离
#define CHASSIS_TASK_DT_S 0.001f
/* Planner units are wheel linear speed (m/s). */
#define CHASSIS_SAME_DIRECTION_ACCEL 6.0f
#define CHASSIS_REVERSE_ACCEL 10.0f

static DJIMotorInstance *motor_lf, *motor_rf;
static uint8_t chassis_enabled;
static uint8_t chassis_feedback_blocked;
static Subscriber_t *chassis_cmd_sub;
static Chassis_Cmd_t chassis_cmd;
static Chassis_Control_Mode_e active_control_mode;
static uint8_t odometry_initialized;
static int8_t odometry_direction;
static float odometry_start_l_deg;
static float odometry_start_r_deg;
static SlopePlanner_t left_speed_planner;
static SlopePlanner_t right_speed_planner;

static void ChassisResetSpeedPlanners(void)
{
    SlopePlanner_Init(&left_speed_planner);
    SlopePlanner_Init(&right_speed_planner);
}


//单位换算
static float ChassisLinearToMotorSpeed(float speed_mps)
{
    return speed_mps / CHASSIS_WHEEL_RADIUS_M * CHASSIS_REDUCTION_RATIO * RAD_2_DEGREE;
}
static float ChassisMotorAngleToDistance(float motor_angle_deg)
{
    return motor_angle_deg * DEGREE_2_RAD /
           CHASSIS_REDUCTION_RATIO * CHASSIS_WHEEL_RADIUS_M;
}

static uint8_t ChassisMotorFeedbackValid(void)
{
    return (uint8_t)(motor_lf != NULL && motor_rf != NULL &&
                     motor_lf->dt > 0.0f && motor_rf->dt > 0.0f &&
                     motor_lf->lost_flag == 0U && motor_rf->lost_flag == 0U);
}
//设置底盘模式
static void ChassisResetOdometry(void)
{
    odometry_initialized = 0U;
    odometry_direction = 1;
}
static void ChassisOdometrySpeedRef(float speed_mps,
                                    float *left_ref_deg,
                                    float *right_ref_deg)
{
    if (speed_mps <= 0.0f)
    {
        *left_ref_deg = motor_lf->measure.total_angle;
        *right_ref_deg = motor_rf->measure.total_angle;
        ChassisResetOdometry();
        return;
    }

    if (!odometry_initialized)
    {
        odometry_start_l_deg = motor_lf->measure.total_angle;
        odometry_start_r_deg = motor_rf->measure.total_angle;
        odometry_initialized = 1U;
    }

    float left_distance = fabsf(ChassisMotorAngleToDistance(
        motor_lf->measure.total_angle - odometry_start_l_deg));
    float right_distance = fabsf(ChassisMotorAngleToDistance(
        motor_rf->measure.total_angle - odometry_start_r_deg));
    float travelled_distance = (left_distance + right_distance) * 0.5f;

    if (travelled_distance >= CHASSIS_ODOMETRY_LEG_M)
    {
        odometry_start_l_deg = motor_lf->measure.total_angle;
        odometry_start_r_deg = motor_rf->measure.total_angle;
        odometry_direction = (int8_t)-odometry_direction;
        if (odometry_direction > 0)
        {
            LOGINFO("[chassis] Odometry leg complete, moving forward");
        }
        else
        {
            LOGINFO("[chassis] Odometry leg complete, moving backward");
        }
    }

    float leg_angle_deg = CHASSIS_ODOMETRY_LEG_M /
                          CHASSIS_WHEEL_RADIUS_M * CHASSIS_REDUCTION_RATIO * RAD_2_DEGREE;
    float signed_leg_angle_deg = (float)odometry_direction * leg_angle_deg;
    *left_ref_deg = odometry_start_l_deg + signed_leg_angle_deg;
    /* The right motor is mechanically reversed and its reference is negated
       inside DJIMotorControl, so use the opposite raw encoder direction. */
    *right_ref_deg = odometry_start_r_deg - signed_leg_angle_deg;
}

static void ChassisSetControlMode(Chassis_Control_Mode_e mode)
{
    if (mode == active_control_mode)
    {
        return;
    }

    active_control_mode = mode;
    ChassisResetOdometry();
    if (mode == CHASSIS_MODE_JOYSTICK)
    {
        LOGINFO("[chassis] Joystick mode");
    }
    else if (mode == CHASSIS_MODE_ODOMETRY)
    {
        LOGINFO("[chassis] Odometry mode");
    }
    else
    {
        LOGINFO("[chassis] Control mode stopped");
    }
}



void ChassisInit() {
    // 四个轮子的参数一样,改tx_id和反转标志位即可
    Motor_Init_Config_s chassis_motor_config = {
        .can_init_config.can_handle = &hcan,
        .controller_param_init_config = {
            .angle_PID = {
                .Kp = 10.0f,//两个都可以50
                .Ki = 0.0f,//1,//0
                .Kd = 0.01f,//0.0f
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .IntegralLimit = 0.3f,
                .MaxOut = 5,

        },
            .speed_PID = {
                .Kp = 1.0f,
                .Ki = 0.0f,
                .Kd = 0.0f,
                .IntegralLimit = 3000,
                .Improve = PID_Trapezoid_Intergral | PID_Integral_Limit | PID_Derivative_On_Measurement,
                .MaxOut = 30000,
        },

},
.controller_setting_init_config = {
    .angle_feedback_source = MOTOR_FEED,
    .speed_feedback_source = MOTOR_FEED,
    .outer_loop_type = ANGLE_LOOP,
    .close_loop_type = ANGLE_AND_SPEED_LOOP, // CURRENT_LOOP,
    .power_limit_flag = POWER_LIMIT_ON,
},
.motor_type = M3508,
.motor_control_type =  CURRENT_CONTROL
};
    //右前
    chassis_motor_config.can_init_config.tx_id = 2;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_REVERSE;
    motor_rf = DJIMotorInit(&chassis_motor_config);
    //左前
    chassis_motor_config.can_init_config.tx_id = 1;
    chassis_motor_config.controller_setting_init_config.motor_reverse_flag = MOTOR_DIRECTION_NORMAL;
    motor_lf = DJIMotorInit(&chassis_motor_config);

    chassis_cmd_sub = SubRegister("chassis_cmd", sizeof(Chassis_Cmd_t));
    chassis_cmd.vx_mps = 0.0f;
    chassis_cmd.vy_mps = 0.0f;
    chassis_cmd.enabled = 0U;
    chassis_cmd.control_mode = CHASSIS_MODE_STOP;
    active_control_mode = CHASSIS_MODE_STOP;
    chassis_feedback_blocked = 0U;
    ChassisResetOdometry();
    ChassisResetSpeedPlanners();

    ChassisSetEnabled(0U);
    if (motor_lf != NULL && motor_rf != NULL && chassis_cmd_sub != NULL)
    {
        LOGINFO("[chassis] Initialized: M3508 IDs 1/2, motors disabled");
    }
    else
    {
        LOGERROR("[chassis] Initialization failed");
    }
}

void ChassisSetEnabled(uint8_t enabled)
{
    chassis_enabled = enabled ? 1U : 0U;
    if (chassis_enabled)
    {
        if (motor_lf != NULL) DJIMotorEnable(motor_lf);
        if (motor_rf != NULL) DJIMotorEnable(motor_rf);

    }
    else
    {
        if (motor_lf != NULL) DJIMotorStop(motor_lf);
        if (motor_rf != NULL) DJIMotorStop(motor_rf);
        chassis_feedback_blocked = 0U;
        ChassisResetOdometry();
        ChassisResetSpeedPlanners();
    }
}

void ChassisSetRef(float chassis_cmd_vx) {
    chassis_cmd.vx_mps = chassis_cmd_vx;
}

void ChassisTask(void)
{
    if (chassis_cmd_sub != NULL)
    {
        (void)SubGetMessage(chassis_cmd_sub, &chassis_cmd);
        ChassisSetControlMode(chassis_cmd.control_mode);
        uint8_t requested_enabled = chassis_cmd.enabled ? 1U : 0U;
        if (requested_enabled != chassis_enabled)
        {
            ChassisSetEnabled(requested_enabled);
        }
    }

    if (motor_lf == NULL || motor_rf == NULL) return;
    if (!chassis_cmd.enabled || !chassis_enabled)
    {
        float left_planned_speed = SlopePlanner_Update(&left_speed_planner,
                                                       CHASSIS_SAME_DIRECTION_ACCEL,
                                                       CHASSIS_REVERSE_ACCEL, 0.0f,
                                                       0.0f, NAN, CHASSIS_TASK_DT_S);
        float right_planned_speed = SlopePlanner_Update(&right_speed_planner,
                                                        CHASSIS_SAME_DIRECTION_ACCEL,
                                                        CHASSIS_REVERSE_ACCEL, 0.0f,
                                                        0.0f, NAN, CHASSIS_TASK_DT_S);
        DJIMotorSetRef(motor_lf, ChassisLinearToMotorSpeed(left_planned_speed));
        DJIMotorSetRef(motor_rf, ChassisLinearToMotorSpeed(right_planned_speed));
        return;
    }

    if (!ChassisMotorFeedbackValid())
    {
        ChassisResetSpeedPlanners();
        DJIMotorSetRef(motor_lf, 0.0f);
        DJIMotorSetRef(motor_rf, 0.0f);
        DJIMotorStop(motor_lf);
        DJIMotorStop(motor_rf);
        ChassisResetOdometry();
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
        DJIMotorEnable(motor_rf);
        LOGINFO("[chassis] Encoder feedback recovered");
    }

    float chassis_speed_vx = 0.0f;
    float chassis_speed_vy = 0.0f;
    if (active_control_mode == CHASSIS_MODE_JOYSTICK)
    {
        chassis_speed_vx = chassis_cmd.vx_mps;
        chassis_speed_vy = chassis_cmd.vy_mps;
    }
    else if (active_control_mode == CHASSIS_MODE_ODOMETRY)
    {
        float left_position_ref_deg;
        float right_position_ref_deg;
        ChassisOdometrySpeedRef(chassis_cmd.vx_mps,
                                &left_position_ref_deg,
                                &right_position_ref_deg);
        DJIMotorSetRef(motor_lf, left_position_ref_deg);
        DJIMotorSetRef(motor_rf, right_position_ref_deg);
        return;
    }

    float left_target_speed = chassis_speed_vx - chassis_speed_vy;
    float right_target_speed = chassis_speed_vx + chassis_speed_vy;
    float left_planned_speed = SlopePlanner_Update(&left_speed_planner,
                                                   CHASSIS_SAME_DIRECTION_ACCEL,
                                                   CHASSIS_REVERSE_ACCEL, 1.0f,
                                                   left_target_speed,
                                                   motor_lf->measure.speed_ms,
                                                   CHASSIS_TASK_DT_S);
    float right_planned_speed = SlopePlanner_Update(&right_speed_planner,
                                                    CHASSIS_SAME_DIRECTION_ACCEL,
                                                    CHASSIS_REVERSE_ACCEL, 1.0f,
                                                    right_target_speed,
                                                    -motor_rf->measure.speed_ms,
                                                    CHASSIS_TASK_DT_S);
    DJIMotorSetRef(motor_lf, ChassisLinearToMotorSpeed(left_planned_speed));
    DJIMotorSetRef(motor_rf, ChassisLinearToMotorSpeed(right_planned_speed));
}
