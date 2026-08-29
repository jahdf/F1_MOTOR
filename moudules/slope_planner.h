//
// Created by Castin on 2026/2/2.
//

#ifndef SLOPE_PLANNER_H
#define SLOPE_PLANNER_H

#include <stdint.h>
#include <math.h>

typedef struct {
    float Same_direction_accel; // 同向加速度 (绝对值)
    float Reverse_accel;        // 反向/突变加速度 (绝对值)
    uint8_t soft_planning;      // 软规划开启标志 (1:开启, 0:关闭)

    float target;               // 当前目标值
    float last_target;          // 上一次目标值
    float real;                 // 真实反馈值
    float planned;              // 当前规划/输出值
} SlopePlanner_t;

/**
 * @brief 斜坡规划器初始化
 * @param sp 指向规划器结构体的指针
 */
void SlopePlanner_Init(SlopePlanner_t *sp);

/**
 * @brief 斜坡规划更新函数
 * @param sp        指向规划器结构体的指针
 * @param same_a    同向加速度 (传 NAN 则不更新)
 * @param reverse_a 反向加速度 (传 NAN 则不更新)
 * @param soft      软规划开关 (1.0f:开启, 0.0f:关闭, NAN:不更新)
 * @param target    新的目标值 (传 NAN 则不更新)
 * @param real      新的真实值 (传 NAN 则不更新)
 * @param dt        控制周期时间 (秒)
 * @return float    规划后的输出值 (planned)
 */
float SlopePlanner_Update(SlopePlanner_t *sp, float same_a, float reverse_a, float soft, float target, float real, float dt);

#endif //SLOPE_PLANNER_H
