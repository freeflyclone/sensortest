//=====================================================================================================
// MadgwickAHRS.c
//=====================================================================================================
//
// Implementation of Madgwick's IMU and AHRS algorithms.
// See: http://www.x-io.co.uk/node/8#open_source_ahrs_and_imu_algorithms
//
// Date			Author          Notes
// 29/09/2011	SOH Madgwick    Initial release
// 02/10/2011	SOH Madgwick	Optimised for reduced CPU load
// 19/02/2012	SOH Madgwick	Magnetometer measurement is normalised
//
//=====================================================================================================

//---------------------------------------------------------------------------------------------------
// Header files

#include "MadgwickAHRS.h"
#include <math.h>
#include <stdint.h>
#include <string.h>

//---------------------------------------------------------------------------------------------------
// Definitions

#define sampleFreq	140.0f		// sample frequency in Hz EJM: setting this lower than reality seems

								// to aid in fixing the thing that adaptive "beta" gain was trying to fix.
#define betaDef		0.025f		// 2 * proportional gain

//---------------------------------------------------------------------------------------------------
// Variable definitions

volatile float beta = betaDef;								// 2 * proportional gain (Kp)
//volatile float q[0] = 1.0f, q[1] = 0.0f, q[2] = 0.0f, q[3] = 0.0f;	// quaternion of sensor frame relative to auxiliary frame
volatile float q[4] = { 1.0f, 0.0f, 0.0f, 0.0f };	// quaternion of sensor frame relative to auxiliary frame


// EJM added for adaptive beta gain
volatile float gyroRateChange = 1.0f;
volatile float accelRateChange = 0.0f;
#define maxBeta		0.10f
#define min(x,y) ((x<y)?(x):(y))

//---------------------------------------------------------------------------------------------------
// Function declarations

float invSqrt(float x);

//====================================================================================================
// Functions

//---------------------------------------------------------------------------------------------------
// AHRS algorithm update

void MadgwickAHRSupdate(float gx, float gy, float gz, float ax, float ay, float az, float mx, float my, float mz) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float hx, hy;
	float _2q0mx, _2q0my, _2q0mz, _2q1mx, _2bx, _2bz, _4bx, _4bz, _2q0, _2q1, _2q2, _2q3;
	//float 2q0q2, _2q2q3;
	float q0q0, q0q1, q0q2, q0q3, q1q1, q1q2, q1q3, q2q2, q2q3, q3q3;
	float _8bx, _8bz;
	// Use IMU algorithm if magnetometer measurement invalid (avoids NaN in magnetometer normalisation)
	if((mx == 0.0f) && (my == 0.0f) && (mz == 0.0f)) {
		MadgwickAHRSupdateIMU(gx, gy, gz, ax, ay, az);
		return;
	}

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
	qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
	qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
	qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

	gyroRateChange = qDot1*qDot1 + qDot2*qDot2 + qDot3*qDot3 + qDot4*qDot4;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		accelRateChange = ax*ax + ay*ay + az*az;
		recipNorm = invSqrt(accelRateChange);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Normalise magnetometer measurement
		recipNorm = invSqrt(mx * mx + my * my + mz * mz);
		mx *= recipNorm;
		my *= recipNorm;
		mz *= recipNorm;

		// Auxiliary variables to avoid repeated arithmetic
		_2q0mx = 2.0f * q[0] * mx;
		_2q0my = 2.0f * q[0] * my;
		_2q0mz = 2.0f * q[0] * mz;
		_2q1mx = 2.0f * q[1] * mx;
		_2q0 = 2.0f * q[0];
		_2q1 = 2.0f * q[1];
		_2q2 = 2.0f * q[2];
		_2q3 = 2.0f * q[3];
		//_2q0q2 = 2.0f * q0 * q2;
		//_2q2q3 = 2.0f * q2 * q3;
		q0q0 = q[0] * q[0];
		q0q1 = q[0] * q[1];
		q0q2 = q[0] * q[2];
		q0q3 = q[0] * q[3];
		q1q1 = q[1] * q[1];
		q1q2 = q[1] * q[2];
		q1q3 = q[1] * q[3];
		q2q2 = q[2] * q[2];
		q2q3 = q[2] * q[3];
		q3q3 = q[3] * q[3];

		// Reference direction of Earth's magnetic field
		hx = mx * q0q0 - _2q0my * q[3] + _2q0mz * q[2] + mx * q1q1 + _2q1 * my * q[2] + _2q1 * mz * q[3] - mx * q2q2 - mx * q3q3;
		hy = _2q0mx * q[3] + my * q0q0 - _2q0mz * q[1] + _2q1mx * q[2] - my * q1q1 + my * q2q2 + _2q2 * mz * q[3] - my * q3q3;
		_2bx = sqrt(hx * hx + hy * hy);
		_2bz = -_2q0mx * q[2] + _2q0my * q[1] + mz * q0q0 + _2q1mx * q[3] - mz * q1q1 + _2q2 * my * q[3] - mz * q2q2 + mz * q3q3;
		_4bx = 2.0f * _2bx;
		_4bz = 2.0f * _2bz;
		_8bx = 2.0f * _4bx;
		_8bz = 2.0f * _4bz;


		// Gradient decent algorithm corrective step
		s0= -_2q2*(2.0f*(q1q3 - q0q2) - ax)    +   _2q1*(2.0f*(q0q1 + q2q3) - ay)   +  -_4bz*q[2]*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   +   (-_4bx*q[3]+_4bz*q[1])*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)    +   _4bx*q[2]*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
		s1= _2q3*(2.0f*(q1q3 - q0q2) - ax) +   _2q0*(2.0f*(q0q1 + q2q3) - ay) +   -4.0f*q[1]*(2.0f*(0.5 - q1q1 - q2q2) - az)    +   _4bz*q[3]*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)   + (_4bx*q[2]+_4bz*q[0])*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)   +   (_4bx*q[3]-_8bz*q[1])*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);             
		s2= -_2q0*(2.0f*(q1q3 - q0q2) - ax)    +     _2q3*(2.0f*(q0q1 + q2q3) - ay)   +   (-4.0f*q[2])*(2.0f*(0.5 - q1q1 - q2q2) - az) +   (-_8bx*q[2]-_4bz*q[0])*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(_4bx*q[1]+_4bz*q[3])*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q[0]-_8bz*q[2])*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
		s3= _2q1*(2.0f*(q1q3 - q0q2) - ax) +   _2q2*(2.0f*(q0q1 + q2q3) - ay)+(-_8bx*q[3]+_4bz*q[1])*(_4bx*(0.5 - q2q2 - q3q3) + _4bz*(q1q3 - q0q2) - mx)+(-_4bx*q[0]+_4bz*q[2])*(_4bx*(q1q2 - q0q3) + _4bz*(q0q1 + q2q3) - my)+(_4bx*q[1])*(_4bx*(q0q2 + q1q3) + _4bz*(0.5 - q1q1 - q2q2) - mz);
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * (1.0f / sampleFreq);
	q[1] += qDot2 * (1.0f / sampleFreq);
	q[2] += qDot3 * (1.0f / sampleFreq);
	q[3] += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// IMU algorithm update

void MadgwickAHRSupdateIMU(float gx, float gy, float gz, float ax, float ay, float az) {
	float recipNorm;
	float s0, s1, s2, s3;
	float qDot1, qDot2, qDot3, qDot4;
	float _2q0, _2q1, _2q2, _2q3, _4q0, _4q1, _4q2 ,_8q1, _8q2, q0q0, q1q1, q2q2, q3q3;

	// Rate of change of quaternion from gyroscope
	qDot1 = 0.5f * (-q[1] * gx - q[2] * gy - q[3] * gz);
	qDot2 = 0.5f * (q[0] * gx + q[2] * gz - q[3] * gy);
	qDot3 = 0.5f * (q[0] * gy - q[1] * gz + q[3] * gx);
	qDot4 = 0.5f * (q[0] * gz + q[1] * gy - q[2] * gx);

	// EJM: Adaptive "beta" gain...
	gyroRateChange = qDot1*qDot1 + qDot2*qDot2 + qDot3*qDot3 + qDot4*qDot4;
	if (gyroRateChange > maxBeta)
		beta = min(gyroRateChange, maxBeta);
	else if (gyroRateChange > beta)
		beta = gyroRateChange;
	else if (beta > betaDef)
		beta -= (maxBeta-betaDef) / sampleFreq;
	else
		beta = betaDef;

	// Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
	if(!((ax == 0.0f) && (ay == 0.0f) && (az == 0.0f))) {

		// Normalise accelerometer measurement
		accelRateChange = ax*ax + ay*ay + az*az;
		recipNorm = invSqrt(accelRateChange);
		ax *= recipNorm;
		ay *= recipNorm;
		az *= recipNorm;   

		// Auxiliary variables to avoid repeated arithmetic
		_2q0 = 2.0f * q[0];
		_2q1 = 2.0f * q[1];
		_2q2 = 2.0f * q[2];
		_2q3 = 2.0f * q[3];
		_4q0 = 4.0f * q[0];
		_4q1 = 4.0f * q[1];
		_4q2 = 4.0f * q[2];
		_8q1 = 8.0f * q[1];
		_8q2 = 8.0f * q[2];
		q0q0 = q[0] * q[0];
		q1q1 = q[1] * q[1];
		q2q2 = q[2] * q[2];
		q3q3 = q[3] * q[3];

		// Gradient decent algorithm corrective step
		s0 = _4q0 * q2q2 + _2q2 * ax + _4q0 * q1q1 - _2q1 * ay;
		s1 = _4q1 * q3q3 - _2q3 * ax + 4.0f * q0q0 * q[1] - _2q0 * ay - _4q1 + _8q1 * q1q1 + _8q1 * q2q2 + _4q1 * az;
		s2 = 4.0f * q0q0 * q[2] + _2q0 * ax + _4q2 * q3q3 - _2q3 * ay - _4q2 + _8q2 * q1q1 + _8q2 * q2q2 + _4q2 * az;
		s3 = 4.0f * q1q1 * q[3] - _2q1 * ax + 4.0f * q2q2 * q[3] - _2q2 * ay;
		recipNorm = invSqrt(s0 * s0 + s1 * s1 + s2 * s2 + s3 * s3); // normalise step magnitude
		s0 *= recipNorm;
		s1 *= recipNorm;
		s2 *= recipNorm;
		s3 *= recipNorm;

		// Apply feedback step
		qDot1 -= beta * s0;
		qDot2 -= beta * s1;
		qDot3 -= beta * s2;
		qDot4 -= beta * s3;
	}

	// Integrate rate of change of quaternion to yield quaternion
	q[0] += qDot1 * (1.0f / sampleFreq);
	q[1] += qDot2 * (1.0f / sampleFreq);
	q[2] += qDot3 * (1.0f / sampleFreq);
	q[3] += qDot4 * (1.0f / sampleFreq);

	// Normalise quaternion
	recipNorm = invSqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
	q[0] *= recipNorm;
	q[1] *= recipNorm;
	q[2] *= recipNorm;
	q[3] *= recipNorm;
}

//---------------------------------------------------------------------------------------------------
// Fast inverse square-root
// See: http://en.wikipedia.org/wiki/Fast_inverse_square_root

float invSqrt(float x) {
	float halfx = 0.5f * x;
	float y = x;
	int32_t i;
	memcpy(&i, &y, sizeof(y));
	i = 0x5f3759df - (i>>1);
	memcpy(&y, &i, sizeof(y));
	y = y * (1.5f - (halfx * y * y));
	return y;
}

//====================================================================================================
// END OF CODE
//====================================================================================================
