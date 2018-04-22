//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date          Author          Notes
// 29/09/2011    SOH Madgwick    Initial release
// 02/10/2011    SOH Madgwick    Optimised for reduced CPU load
//
//=============================================================================================
#pragma once
#include <stdint.h>
#include <math.h>

class Mahony {
public:
  Mahony(uint32_t updatePeriod);
  void setP(float p);
  void setI(float i);

  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz);

  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz,
              float mx, float my, float mz);

private:
  float _twoKp = 2.0f * 0.5f;
  float _twoKi = 2.0f * 0.0f;
  float _quaternion[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
  float _integralFBx = 0.0f;
  float _integralFBy = 0.0f;
  float _integralFBz = 0.0f;
  float _invSampleFreq;
  static float _invSqrt(float x);
  void _toYawPitchRoll(float &yaw, float &pitch, float &roll);
};