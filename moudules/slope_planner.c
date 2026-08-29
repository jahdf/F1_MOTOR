//
// Created by Castin on 2026/2/2.
//

#include "slope_planner.h"
#include <stdlib.h>
#include "controller.h"
void SlopePlanner_Init(SlopePlanner_t *sp) {
    if (sp == NULL) return;

    sp->Same_direction_accel = 0.0f;
    sp->Reverse_accel = 0.0f;
    sp->soft_planning = 0;

    sp->target = 0.0f;
    sp->last_target = 0.0f;
    sp->real = 0.0f;
    sp->planned = 0.0f;
}

/**
 * @brief 速度斜坡规划  传参target要与接受return的变量是同一个变量
 * @param sp
 * @param same_a  目标值与上一次的目标值的正负（方向）一致时  加减速度是same_a
 * @param reverse_a  正负相反（方向突变）是reverse_a
 * @param soft  选择开启软规划要传入1  关闭是NAN
 * @param target
 * @param real
 * @param dt
 * @return
 */
float SlopePlanner_Update(SlopePlanner_t* sp,float same_a, float reverse_a, float soft,float target, float real, float dt) {
    // 更新配置参数(如果提供了新值)
    if (!isnan(same_a)) sp->Same_direction_accel = same_a;
    if (!isnan(reverse_a)) sp->Reverse_accel = reverse_a;
    if (!isnan(soft)) sp->soft_planning = (uint8_t)soft;
    // 更新目标值和真实值(如果提供了新值)
    if (!isnan(target)) {
        sp->last_target = sp->target;
        sp->target = target;
    }
    if (!isnan(real)) sp->real = real;
    // 计算变化量
    float delta = 0.0f;
    float dir_change = sp->target * sp->last_target;

    if (dir_change >= 0 && abs(sp->target) >= abs(sp->last_target)) {
        delta = sp->Same_direction_accel * dt;  // 加速
    } else {
        delta = sp->Reverse_accel * dt;  // 减速
    }
    // 更新规划值
    if (sp->target > sp->planned) {
        sp->planned += delta;
        if (sp->planned > sp->target) sp->planned = sp->target;
    } else if (sp->target < sp->planned) {
        sp->planned -= delta;
        if (sp->planned < sp->target) sp->planned = sp->target;
    }
    // 软规划处理  如果真实值已经在目标值和规划值之间，说明系统已经跟上或者超调了，此时直接将规划值同步为真实值，防止规划器滞后拖慢系统响应。
    if (sp->soft_planning) {
        if ((sp->real >= sp->target && sp->real <= sp->planned) ||
            (sp->real <= sp->target && sp->real >= sp->planned)) {
            sp->planned = sp->real;
            }
    }
    return sp->planned;
}