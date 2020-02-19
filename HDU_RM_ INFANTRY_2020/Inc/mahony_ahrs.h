#ifndef __mahony_ahrs_h
#define __mahony_ahrs_h
#include "main.h"
#include "math.h"
struct ahrs_sensor
{
  float ax;
  float ay;
  float az;

  float wx;
  float wy;
  float wz;

  float mx;
  float my;
  float mz;

  float temp;
};

struct attitude
{
  float roll;
  float pitch;
  float yaw;
};

float invSqrt(float x);

void madgwick_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void madgwick_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);

void mahony_ahrs_update(struct ahrs_sensor *sensor, struct attitude *atti);
void mahony_ahrs_updateIMU(struct ahrs_sensor *sensor, struct attitude *atti);




#endif
