#ifndef TEXT1_GIMBAL_H
#define TEXT1_GIMBAL_H

#include <stdint.h>

typedef struct
{
    float wz_rad_s;
    uint8_t enabled;
} Gimbal_Cmd_t;

void GimbalInit(void);
void GimbalTask(void);
void GimbalSetRef(float speed_ref);
void GimbalSetEnabled(uint8_t enabled);

#endif
