#ifndef TEXT1_ROBOT_CMD_H
#define TEXT1_ROBOT_CMD_H

#include <stdint.h>

void RobotCmdInit(void);
void RobotCmdTask(void);
uint8_t RobotCmdMotorEnabled(void);

#endif
