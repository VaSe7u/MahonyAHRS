/*!
The MIT License (MIT)

Copyright (c) 2016 Vasil Kalchev

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.

@file       MahonyAHRS.cpp
@authors    SOH Madgwick
@authors    Vasil Kalchev
@date       2011-2018
@version    0.9.0
@copyright  The MIT License
@brief      Mahony's sensor fusion algorithm.

@todo:
*/

#include "MahonyAHRS.hpp"

Mahony::Mahony(const float samplePeriod)
  : _samplePeriod(samplePeriod) {}

void Mahony::setP(const float p) {
  _twoKp = 2.0f * p;
}

void Mahony::setI(const float i) {
  _twoKi = 2.0f * i;
}

void Mahony::update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz) {
  float recipNorm;

// Compute feedback only if accelerometer measurement valid
// (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = Mahony::_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
  float halfvx, halfvy, halfvz;
    halfvx = _quaternion[1] * _quaternion[3] - _quaternion[0] * _quaternion[2];
    halfvy = _quaternion[0] * _quaternion[1] + _quaternion[2] * _quaternion[3];
    halfvz = _quaternion[0] * _quaternion[0] - 0.5f + _quaternion[3] * _quaternion[3];

    // Error is sum of cross product between estimated
    // and measured direction of gravity
  float halfex, halfey, halfez;
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _samplePeriod;
      _integralFBy += _twoKi * halfey * _samplePeriod;
      _integralFBz += _twoKi * halfez * _samplePeriod;
      gx += _integralFBx;  // apply integral feedback
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f; // prevent integral windup
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += _twoKp * halfex;
    gy += _twoKp * halfey;
    gz += _twoKp * halfez;
  }

// Integrate rate of change of quaternion
  gx *= (0.5f * _samplePeriod);   // pre-multiply common factors
  gy *= (0.5f * _samplePeriod);
  gz *= (0.5f * _samplePeriod);
  float qa, qb, qc;
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

// Normalise quaternion
  recipNorm = Mahony::_invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;

  _toYawPitchRoll(yaw, pitch, roll);
}

void Mahony::update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz,
            float mx, float my, float mz) {
  float recipNorm;

  // Use IMU algorithm if magnetometer measurement invalid
  // (avoids NaN in magnetometer normalisation)
  if ((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
    update(yaw, pitch, roll, ax, ay, az, gx, gy, gz);
    return;
  }

  // Compute feedback only if accelerometer measurement valid
  // (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = Mahony::_invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = Mahony::_invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
    float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
    q0q0 = _quaternion[0] * _quaternion[0];
    q0q1 = _quaternion[0] * _quaternion[1];
    q0q2 = _quaternion[0] * _quaternion[2];
    q0q3 = _quaternion[0] * _quaternion[3];
    q1q1 = _quaternion[1] * _quaternion[1];
    q1q2 = _quaternion[1] * _quaternion[2];
    q1q3 = _quaternion[1] * _quaternion[3];
    q2q2 = _quaternion[2] * _quaternion[2];
    q2q3 = _quaternion[2] * _quaternion[3];
    q3q3 = _quaternion[3] * _quaternion[3];

    // Reference direction of Earth's magnetic field
  float hx, hy, bx, bz;
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
  float halfex, halfey, halfez;
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _samplePeriod;
      _integralFBy += _twoKi * halfey * _samplePeriod;
      _integralFBz += _twoKi * halfez * _samplePeriod;
      gx += _integralFBx;  // apply integral feedback
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f; // prevent integral windup
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += _twoKp * halfex;
    gy += _twoKp * halfey;
    gz += _twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * _samplePeriod);   // pre-multiply common factors
  gy *= (0.5f * _samplePeriod);
  gz *= (0.5f * _samplePeriod);
  float qa, qb, qc;
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = Mahony::_invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;


  _toYawPitchRoll(yaw, pitch, roll);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float Mahony::_invSqrt(const float x) {
  float halfx = 0.5f * x;
  float y = x;
  int32_t i = *(int32_t*)&y; // not portable
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i; // not portable
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

void Mahony::_toYawPitchRoll(float &yaw, float &pitch, float &roll) {
  // Quaternion to Euler
  // pitch (y-axis rotation)
  float sinp = +2.0 * (_quaternion[0] * _quaternion[2] - _quaternion[3] * _quaternion[1]);
  if (fabs(sinp) >= 1) {
    pitch = copysign(M_PI / 2, sinp); // use 90 degrees if out of range
  } else {
    pitch = asin(sinp);
  }

  // roll (x-axis rotation)
  float sinr = +2.0 * (_quaternion[0] * _quaternion[1] + _quaternion[2] * _quaternion[3]);
  float cosr = +1.0 - 2.0 * (_quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2]);
  roll = atan2(sinr, cosr);

  // yaw (z-axis rotation)
  float siny = +2.0 * (_quaternion[0] * _quaternion[3] + _quaternion[1] * _quaternion[2]);
  float cosy = +1.0 - 2.0 * (_quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  yaw = atan2(siny, cosy);
  yaw += M_PI;
}