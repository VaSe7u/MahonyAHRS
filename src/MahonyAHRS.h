//=============================================================================================
// MahonyAHRS.h
//=============================================================================================
//
// Madgwick's implementation of Mayhony's AHRS algorithm.
// See: http://www.x-io.co.uk/open-source-imu-and-ahrs-algorithms/
//
// Date			Author			Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
//
//=============================================================================================
#ifndef MahonyAHRS_h
#define MahonyAHRS_h
#include <math.h>

//--------------------------------------------------------------------------------------------
// Variable declaration

class Mahony {
private:
	float twoKp;		// 2 * proportional gain (Kp)
	float twoKi;		// 2 * integral gain (Ki)
	float q0, q1, q2, q3;	// quaternion of sensor frame relative to auxiliary frame
	float integralFBx, integralFBy, integralFBz;  // integral error terms scaled by Ki
	float invSampleFreq;
	static float invSqrt(float x);

//-------------------------------------------------------------------------------------------
// Function declarations

public:
	Mahony();
	void begin(float sampleFrequency) { invSampleFreq = 1.0f / sampleFrequency; }
	void update(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz);
	void updateIMU(float gx, float gy, float gz, float ax, float ay, float az);
	float getPitch() {
		float x = 2.0f * (q1*q3 - q0*q2);
		float y = 2.0f * (q0*q1 + q2*q3);
		float z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
		return atan2f(x, sqrtf(y*y + z*z));
	}
	float getRoll() {
		float x = 2.0f * (q1*q3 - q0*q2);
		float y = 2.0f * (q0*q1 + q2*q3);
		float z = q0*q0 - q1*q1 - q2*q2 + q3*q3;
		return atan2f(y, sqrtf(x*x + z*z));
	}
	float getYaw() {
		return atan2f(2.0f*q1*q2 - 2.0f*q0*q3,
			2.0f*q0*q0 + 2.0f*q1*q1 - 1.0f);
	}
};

#endif
