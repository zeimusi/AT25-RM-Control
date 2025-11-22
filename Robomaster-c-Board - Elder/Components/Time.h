#ifndef __TIME_H
#define __TIME_H

#include "tim.h"
typedef struct{
uint32_t GimbalInit;
uint16_t ShootStuck;
uint16_t Single;
}eTime;
extern eTime Time;
#endif
