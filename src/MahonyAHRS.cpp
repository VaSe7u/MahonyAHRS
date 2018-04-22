//=============================================================================================
// MahonyAHRS.c
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// From the x-io website "Open-source resources available on this website are
// provided under the GNU General Public Licence unless an alternative licence
// is provided in source."
//
// Date     Author      Notes
// 29/09/2011 SOH Madgwick    Initial release
// 02/10/2011 SOH Madgwick    Optimised for reduced CPU load
//
// Algorithm paper:
// http://ieeexplore.ieee.org/xpl/login.jsp?tp=&arnumber=4608934&url=http%3A%2F%2Fieeexplore.ieee.org%2Fstamp%2Fstamp.jsp%3Ftp%3D%26arnumber%3D4608934
//
//=============================================================================================
#include "MahonyAHRS.h"

Mahony::Mahony(uint16_t updatePeriod)
  : _invSampleFreq(updatePeriod) {}

void Mahony::setP(float p) {
  _twoKp = 2.0f * p;
}

void Mahony::setI(float i) {
  _twoKi = 2.0f * i;
}

void update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz,
            float mx, float my, float mz) {
  float recipNorm;
  float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
  float hx, hy, bx, bz;
  float halfvx, halfvy, halfvz, halfwx, halfwy, halfwz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

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
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Normalise magnetometer measurement
    recipNorm = invSqrt(mx * mx + my * my + mz * mz);
    mx *= recipNorm;
    my *= recipNorm;
    mz *= recipNorm;

    // Auxiliary variables to avoid repeated arithmetic
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
    hx = 2.0f * (mx * (0.5f - q2q2 - q3q3) + my * (q1q2 - q0q3) + mz * (q1q3 + q0q2));
    hy = 2.0f * (mx * (q1q2 + q0q3) + my * (0.5f - q1q1 - q3q3) + mz * (q2q3 - q0q1));
    bx = sqrtf(hx * hx + hy * hy);
    bz = 2.0f * (mx * (q1q3 - q0q2) + my * (q2q3 + q0q1) + mz * (0.5f - q1q1 - q2q2));

    // Estimated direction of gravity and magnetic field
    halfvx = q1q3 - q0q2;
    halfvy = q0q1 + q2q3;
    halfvz = q0q0 - 0.5f + q3q3;
    halfwx = bx * (0.5f - q2q2 - q3q3) + bz * (q1q3 - q0q2);
    halfwy = bx * (q1q2 - q0q3) + bz * (q0q1 + q2q3);
    halfwz = bx * (q0q2 + q1q3) + bz * (0.5f - q1q1 - q2q2);

    // Error is sum of cross product between estimated direction
    // and measured direction of field vectors
    halfex = (ay * halfvz - az * halfvy) + (my * halfwz - mz * halfwy);
    halfey = (az * halfvx - ax * halfvz) + (mz * halfwx - mx * halfwz);
    halfez = (ax * halfvy - ay * halfvx) + (mx * halfwy - my * halfwx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _invSampleFreq;
      _integralFBy += _twoKi * halfey * _invSampleFreq;
      _integralFBz += _twoKi * halfez * _invSampleFreq;
      gx += _integralFBx;  // apply integral feedback
      gy += _integralFBy;
      gz += _integralFBz;
    } else {
      _integralFBx = 0.0f; // prevent integral windup
      _integralFBy = 0.0f;
      _integralFBz = 0.0f;
    }

    // Apply proportional feedback
    gx += twoKp * halfex;
    gy += twoKp * halfey;
    gz += twoKp * halfez;
  }

  // Integrate rate of change of quaternion
  gx *= (0.5f * _invSampleFreq);   // pre-multiply common factors
  gy *= (0.5f * _invSampleFreq);
  gz *= (0.5f * _invSampleFreq);
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

  // Normalise quaternion
  recipNorm = _invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;


  _toYawPitchRoll(yaw, pitch, roll);
}

void update(float &yaw, float &pitch, float &roll,
            float ax, float ay, float az,
            float gx, float gy, float gz) {
  float recipNorm;
  float halfvx, halfvy, halfvz;
  float halfex, halfey, halfez;
  float qa, qb, qc;

// Compute feedback only if accelerometer measurement valid
// (avoids NaN in accelerometer normalisation)
  if (!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

    // Normalise accelerometer measurement
    recipNorm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= recipNorm;
    ay *= recipNorm;
    az *= recipNorm;

    // Estimated direction of gravity
    halfvx = _quaternion[1] * _quaternion[3] - _quaternion[0] * _quaternion[2];
    halfvy = _quaternion[0] * _quaternion[1] + _quaternion[2] * _quaternion[3];
    halfvz = _quaternion[0] * _quaternion[0] - 0.5f + _quaternion[3] * _quaternion[3];

    // Error is sum of cross product between estimated
    // and measured direction of gravity
    halfex = (ay * halfvz - az * halfvy);
    halfey = (az * halfvx - ax * halfvz);
    halfez = (ax * halfvy - ay * halfvx);

    // Compute and apply integral feedback if enabled
    if (_twoKi > 0.0f) {
      // integral error scaled by Ki
      _integralFBx += _twoKi * halfex * _invSampleFreq;
      _integralFBy += _twoKi * halfey * _invSampleFreq;
      _integralFBz += _twoKi * halfez * _invSampleFreq;
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
  gx *= (0.5f * _invSampleFreq);   // pre-multiply common factors
  gy *= (0.5f * _invSampleFreq);
  gz *= (0.5f * _invSampleFreq);
  qa = _quaternion[0];
  qb = _quaternion[1];
  qc = _quaternion[2];
  _quaternion[0] += (-qb * gx - qc * gy - _quaternion[3] * gz);
  _quaternion[1] += (qa * gx + qc * gz - _quaternion[3] * gy);
  _quaternion[2] += (qa * gy - qb * gz + _quaternion[3] * gx);
  _quaternion[3] += (qa * gz + qb * gy - qc * gx);

// Normalise quaternion
  recipNorm = _invSqrt(_quaternion[0] * _quaternion[0] + _quaternion[1] * _quaternion[1] + _quaternion[2] * _quaternion[2] + _quaternion[3] * _quaternion[3]);
  _quaternion[0] *= recipNorm;
  _quaternion[1] *= recipNorm;
  _quaternion[2] *= recipNorm;
  _quaternion[3] *= recipNorm;

  _toYawPitchRoll(yaw, pitch, roll);
}

// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root
float Mahony::_invSqrt(float x) {
  float halfx = 0.5f * x;
  float y = x;
  long i = *(long*)&y;
  i = 0x5f3759df - (i >> 1);
  y = *(float*)&i;
  y = y * (1.5f - (halfx * y * y));
  y = y * (1.5f - (halfx * y * y));
  return y;
}

