/*
 * MahonyAHRS library - SixAxisMahony.ino
 *
 * Demostrates converting 3-axis acceleration and 3-axis angular
 * velocities data to Euler angles.
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

}

void loop() {

  /* IMU devices usually return their measurements in raw units
     that can be converted to m/s^2, radians/second etc... with some
     calculation specified in their datasheet. When using a Mahony
     filter, only the angular velocity data must be converted to
     radians per second, the acceleration data returned from the
     IMU can be passed to the algorithm as is because it will be
     normalized. */
  //! int axRaw = 0, ayRaw = 0, azRaw = 0; // 3-axis raw acceleration
  //! int gxRaw = 0, gyRaw = 0, gzRaw = 0; // 3-axis raw angular velocity
  //! your_imu.read(&axRaw, &ayRaw &azRaw, &gxRaw, &gyRaw, &gzRaw);

  // Convert raw angular velocity to radians per second.
  //! float gxRadS = gxRaw * your_imu_toRads;
  //! float gyRadS = gyRaw * your_imu_toRads;
  //! float gzRadS = gzRaw * your_imu_toRads;

  /* Yaw, pitch and roll variables (in radians) that will be
     overwritten by the algorithm. */
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
    mahony.update(yawRad, pitchRad, rollRad, // yaw, pitch, roll references
                  axRaw, ayRaw, azRaw, // x, y, z acceleration
                  gxRadS, gyRadS, gzRadS); // x, y, z angular velocity
  }

  // Convert the result of the algorithm from radians to degrees.
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