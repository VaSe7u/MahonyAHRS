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
const float mahonySamplePeriod = 0.05f;
Mahony mahony(mahonySamplePeriod);

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

  /* Yaw, pitch and roll variables in radians per second the will be
     overwritten by the algorithm. */
  float yawRadS = 0.0f;
  float pitchRadS = 0.0f;
  float rollRadS = 0.0f;

  /* The Mahony algorithm must be called at fixed period specified in
     earlier in the constructor. In this case 50 milliseconds. */
  static unsigned long mahonyLMS = millis();
  if (millis() - mahonyLMS > 50) {
    mahonyLMS = millis();
    /* The update function implements the Mahony's sensor fusion
       algorithm. The first three parameters are references to the
       yaw, pitch and roll variables, where the result will be
       written in radians per second. The next three parameters are
       the 3-axis accelerations. The last three parameters are the
       3-axis angular velocities. */
    mahony.update(&yawRadS, &pitchRadS, &rollRadS, // yaw, pitch, roll references
                  axRaw, ayRaw, azRaw, // x, y, z acceleration
                  gxRadS, gyRadS, gzRadS); // x, y, z angular velocity
  }

  /* Convert the result of the algorithm from radians per second to
     degrees per second. */
  float yawDegS = yawRadS * 57.295779513f;
  float pitchDegS = pitchRadS * 57.295779513f;
  float rollDegS = rollRadS * 57.295779513f;

}