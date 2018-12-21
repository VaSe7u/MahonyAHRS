/*
 * MahonyAHRS library - NineAxisMahony.ino
 *
 * Demostrates converting 3-axis acceleration, 3-axis angular
 * velocities and 3-axis magnetic fields data to Euler angles.
 *
 * @note Template code is prepended with '!'.
 *
 * Created July 7, 2018
 * by Vasil Kalchev
 *
 * https://github.com/VaSe7u/MahonyAHRS
 *
 */

// An IMU has to be used to obtain the input data.
//! #include <YOUR_IMU.h>
#include <MahonyAHRS.hpp>

//! YOUR_IMU your_imu(your_imu_params...);

// The period between calls to Mahony's update function.
const float mahonySamplePeriodS = 0.05f; // in seconds
const unsigned short mahonySamplePeriodMs = 50; // in milliseconds
Mahony mahony(mahonySamplePeriodS);

bool timeToUpdateMahony();

/* Assign this function pointer to a function that returns the time
   since boot in milliseconds. Currently points to Arduino's millis()
   function. */
//! unsigned long (*milliseconds_ptr)(void) = &millis;

void setup() {
//! your_imu.initialize(your_imu_init_params...);

// Set Mahony's filter P and I gains.
// If not specified the default values are P = 0.5 and I = 0.
const float mahonyProportionalGain = 0.5f;
const float mahonyIntegralGain = 0.0f;
mahony.setP(mahonyProportionalGain);
mahony.setI(mahonyIntegralGain);
}

void loop() {

  /* IMU devices usually return their measurements in raw units
     that can be converted to m/s^2, radians/second etc... with some
     calculation specified in their datasheet. When using a Mahony
     filter, only the angular velocity data must be converted to
     radians per second, the acceleration and megnetic field data
     returned from the IMU can be passed to the algorithm as is
     because it will be normalized. */
  //! int axRaw = 0, ayRaw = 0, azRaw = 0; // 3-axis raw acceleration
  //! int gxRaw = 0, gyRaw = 0, gzRaw = 0; // 3-axis raw angular velocity
  //! int mxRaw = 0, myRaw = 0, mzRaw = 0; // 3-axis raw magnetic fields
  //! your_imu.read(&axRaw, &ayRaw &azRaw,
  //!               &gxRaw, &gyRaw, &gzRaw,
  //!               &mxRaw, &myRaw, &mzRaw);

  // Convert raw angular velocity to radians per second.
  //! float gxRadS = gxRaw * your_imu_toRads;
  //! float gyRadS = gyRaw * your_imu_toRads;
  //! float gzRadS = gzRaw * your_imu_toRads;

  /* Yaw, pitch and roll variables in radians the will be overwritten
     by the algorithm. */
  float yawRad = 0.0f;
  float pitchRad = 0.0f;
  float rollRad = 0.0f;

  /* The Mahony algorithm must be called at fixed period specified
     earlier in the constructor. In this case 50 milliseconds. */
  if (timeToUpdateMahony()) {
    /* The update function implements the Mahony's sensor fusion
       algorithm. The first three parameters are references to the
       yaw, pitch and roll variables, where the result will be
       written in radians. The next three parameters are the 3-axis
       accelerations. The last three parameters are the 3-axis
       angular velocities. */
    mahony.update(&yawRad, &pitchRad, &rollRad, // YPR references
                  axRaw, ayRaw, azRaw, // x, y, z acceleration
                  gxRadS, gyRadS, gzRadS, // x, y, z angular velocity
                  mxRaw, myRaw, mzRaw); // x, y, z magnetic field
  }

  /* Convert the result of the algorithm from radians per second to
     degrees per second. */
  float yawDeg = yawRad * 57.295779513f;
  float pitchDeg = pitchRad * 57.295779513f;
  float rollDeg = rollRad * 57.295779513f;

}

bool timeToUpdateMahony() {
  static unsigned long mahonyLMS = (*milliseconds_ptr)();
  if ((*milliseconds_ptr)() - mahonyLMS > mahonySamplePeriodMs) {
    mahonyLMS = (*milliseconds_ptr)();
    return true;
  } else {
    return false;
  }
}