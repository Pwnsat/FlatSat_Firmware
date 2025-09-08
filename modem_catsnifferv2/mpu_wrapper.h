#ifndef __MPU_WRAPPER_H
#define __MPU_WRAPPER_H

#include "proto.h"
#include <Adafruit_MPU6050.h>

sensor_status_t mpuInit();
sensor_status_t mpuReadData(uint8_t *buffer, int &offset);

#endif
