#ifndef TEXT1_DISPLAY_APP_H
#define TEXT1_DISPLAY_APP_H

#include "bsp_can.h"
#include "motor_def.h"

void DisplayAppInit(void);
void DisplayAppTask(void *argument);
uint8_t DisplayAppMotorTypeConfirmed(void);
Motor_Type_e DisplayAppGetSelectedMotorType(void);

#endif
