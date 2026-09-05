//
// Created by zgb on 2026/5/31.
//

#ifndef TEXT1_CHASSIS_H
#define TEXT1_CHASSIS_H

#include "bsp_can.h"
#include "motor_def.h"

typedef enum
{
    CHASSIS_MODE_STOP = 0,
    CHASSIS_MODE_JOYSTICK,
    CHASSIS_MODE_ODOMETRY,
} Chassis_Control_Mode_e;

typedef struct
{
    float wz;
    uint8_t enabled;
    Chassis_Control_Mode_e control_mode;
} Chassis_Cmd_t;

void ChassisInit();
void ChassisSetRef(float chassis_cmd_wz);
void ChassisSetEnabled(uint8_t enabled);
void ChassisTask(void);

#endif //TEXT1_CHASSIS_H
