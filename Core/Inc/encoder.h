#ifndef _ENCODER_H
#define _ENCODER_H
#include <stdint.h>
#include "main.h"
void Encoder_Init(void);
int16_t Encoder_LeftRead(void);
int16_t Encoder_RightRead(void);
#endif
