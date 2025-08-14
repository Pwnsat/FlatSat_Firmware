#include "mpu_wrapper.h"

Adafruit_MPU6050 mpu;

static sensor_status_t mpu_status = SENSOR_OK;

sensor_status_t mpuInit() {
  Serial.println("[SYS - MPU] MPU6050 Init");
  if (!mpu.begin(0x69)) {
    Serial.println("[SYS - MPU] Failed to find MPU6050 chip");
    mpu_status = SENSOR_FAIL;
    return mpu_status;
  }
  Serial.println("[SYS - MPU] MPU6050 Found!");
  mpu.setAccelerometerRange(MPU6050_RANGE_8_G);
  mpu.setGyroRange(MPU6050_RANGE_500_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);
  Serial.println("[SYS - MPU] Accelerometer range set to: +-8G");
  Serial.println("[SYS - MPU] Gyro range set to: +- 500 deg/s");
  Serial.println("[SYS - MPU] Filter bandwidth set to: 21 Hz");
  mpu_status = SENSOR_OK;
  return mpu_status;
}

sensor_status_t mpuReadData(uint8_t *buffer, int &offset) {
  sensors_event_t a, g, temp;
  if (!mpu.getEvent(&a, &g, &temp)) {
    buffer[offset++] = TM_SENSOR_STATUS_ERR;
    return SENSOR_FAIL;
  }
  buffer[offset++] = TM_SENSOR_STATUS_OK;
  floatToBigEndian(a.acceleration.x, buffer + offset);
  offset += 4;
  floatToBigEndian(a.acceleration.y, buffer + offset);
  offset += 4;
  floatToBigEndian(a.acceleration.z, buffer + offset);
  offset += 4;
  floatToBigEndian(g.gyro.x, buffer + offset);
  offset += 4;
  floatToBigEndian(g.gyro.y, buffer + offset);
  offset += 4;
  floatToBigEndian(g.gyro.z, buffer + offset);
  offset += 4;
  floatToBigEndian(temp.temperature, buffer + offset);
  offset += 4;
  return SENSOR_OK;
}