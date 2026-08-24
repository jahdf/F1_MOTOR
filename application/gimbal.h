#ifndef TEXT1_GIMBAL_H
#define TEXT1_GIMBAL_H

#include <stdint.h>

void GimbalInit(void);
void GimbalTask(void);
void GimbalSetReference(float speed_ref);
void GimbalSetEnabled(uint8_t enabled);

#endif
