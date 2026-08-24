//
// Created by zgb on 2026/5/31.
//

#ifndef TEXT1_CHASSIS_H
#define TEXT1_CHASSIS_H

#include "bsp_can.h"
#include "motor_def.h"

void ChassisInit(Motor_Type_e motor_type);
void ChassisSetRef(void);
void ChassisSetEnabled(uint8_t enabled);

#endif //TEXT1_CHASSIS_H
