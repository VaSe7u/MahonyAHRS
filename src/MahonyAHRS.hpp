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

@file       MahonyAHRS.hpp
@authors    SOH Madgwick
@authors    Vasil Kalchev
@date       2011-2018
@version    0.9.0
@copyright  The MIT License
@brief      Mahony's sensor fusion algorithm.

@todo:
*/

#pragma once
#include <stdint.h>
#include <math.h>

#define DEFAULT_PROPORTIONAL_GAIN 0.5f
#define DEFAULT_INTEGRAL_GAIN 0.0f

class Mahony {
public:
  /*!
  @brief The constructor of the filter.
  @param[in] samplePeriod: the period between `update` calls (in
  seconds).
  @note The `samplePeriod` parameter is the time in seconds between
  calls to the `update` function/s.
  */
  explicit Mahony(const float samplePeriod);

  /*!
  @brief Proportional gain setter.
  @param[in] p: the proportional gain of the filter.
  */
  void setP(const float p);

  /*!
  @brief Integral gain setter.
  @param[in] i: the integral gain of the filter.
  */
  void setI(const float i);

  /*!
  @brief Calculates Euler angles...

  from 3-axis accelerations and 3-axis angular velocities. The
  calculated Euler angles will be written to the references passed to
  the function (in radians). The passed acceleration values can be
  relative to each other, but the passed angular velocities must be
  in radians per second.

  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  @param[in] ax: x-axis acceleration.
  @param[in] ay: y-axis acceleration.
  @param[in] az: z-axis acceleration.
  @param[in] gx: x-axis angular velocity (in radians per second).
  @param[in] gy: y-axis angular velocity (in radians per second).
  @param[in] gz: z-axis angular velocity (in radians per second).
  */
  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz);

  /*!
  @brief Calculates Euler angles...

  from 3-axis accelerations, 3-axis angular velocities and 3-axis
  magnetic fields. The calculated Euler angles will be written to the
  references passed to the function (in radians). The passed
  acceleration and magnetic field values can be relative to each
  other, but the passed angular velocities must be in radians per
  second.

  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  @param[in] ax: x-axis acceleration.
  @param[in] ay: y-axis acceleration.
  @param[in] az: z-axis acceleration.
  @param[in] gx: x-axis angular velocity (in radians per second).
  @param[in] gy: y-axis angular velocity (in radians per second).
  @param[in] gz: z-axis angular velocity (in radians per second).
  @param[in] ax: x-axis magnetic field.
  @param[in] ay: y-axis magnetic field.
  @param[in] az: z-axis magnetic field.
  */
  void update(float &yaw, float &pitch, float &roll,
              float ax, float ay, float az,
              float gx, float gy, float gz,
              float mx, float my, float mz);

private:
  float _twoKp = 2.0f * DEFAULT_PROPORTIONAL_GAIN; //! 2 * proportional gain
  float _twoKi = 2.0f * DEFAULT_INTEGRAL_GAIN; //! 2 * integral gain
  float _quaternion[4] = { 1.0f, 0.0f, 0.0f, 0.0f };
  float _integralFBx = 0.0f;
  float _integralFBy = 0.0f;
  float _integralFBz = 0.0f;
  float _samplePeriod;

  /*!
  @brief Inverse square root.
  @param[in] x.
  */
  static float _invSqrt(const float x);

  /*!
  @brief Converts quaternion to Euler angles.
  @param[out] &yaw: this reference will be set to the calculated yaw
  angle (in radians).
  @param[out] &pitch: this reference will be set to the calculated
  pitch angle (in radians).
  @param[out] &roll: this reference will be set to the calculated
  roll angle (in radians).
  */
  void _toYawPitchRoll(float &yaw, float &pitch, float &roll);
};