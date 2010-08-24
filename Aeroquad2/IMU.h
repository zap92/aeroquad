#ifndef __IMU_H__
#define __IMU_H__

#include "SubSystem.h"
#include "HardwareComponent.h"

#include "SerialComs.h"

class IMUHardware : public AnalogInHardwareComponent
{
	protected:
		enum {IMUGyroX = 0, IMUGyroY, IMUGyroZ, IMUAcclX, IMUAcclY, IMUAcclZ};

	public:
		IMUHardware(const unsigned int inputCount);		
		
		virtual void initialize();				
		virtual void process(const unsigned long currentTime)	;						
		
		virtual void calibrateZero();
		
		const float* currentReadings();
};

class OilpanIMU : public IMUHardware
{	
	public:
		OilpanIMU();
		
		virtual void initialize();
		virtual const int readRawValue(const unsigned int channel);		
};

class SuplimentalYawSource : public HardwareComponent
{
	protected:
		int _lastReadings[3];
		
	public:
		SuplimentalYawSource();
		
		virtual void initialize();	
		virtual void process(const unsigned long currentTime);
		
		const int* currentReadings();	
};

class HMC5843Compass : public SuplimentalYawSource
{
	public:
		virtual void initialize();
		virtual void process(const unsigned long currentTime);
};
	

class IMUFilter
{
	protected:
		typedef enum { IMUFilterAxisX = 0, IMUFilterAxisY, IMUFilterAxisZ } IMUFilterAxis;
		typedef enum { IMUFilterRoll = 0, IMUFilterPitch, IMUFilterYaw } IMUFilterAttitude;
		
		unsigned long _lastProcessTime;
		unsigned int  _deltaTime;
		
		bool _firstPass;
		
		float _accelRaw[3];
		float _gyroRaw[3];
		int _suplimentalRaw[3];

		float _processedRateInRadians[3];
		float _processedRateInDegrees[3];		
		float _processedAngleInRadians[3];
		float _processedAngleInDegrees[3];
		
		const float _correctYawReading(const float rollAngle, const float pitchAngle);		
		void _calculateRawAngles(float *rollAngle, float *pitchAngle, float* yawAngle);
		
	public:
		IMUFilter();	
		
		virtual void initialize();
		virtual void filter(const unsigned long currentTime);
		
		void setCurrentReadings(const float *currentReadings);
		void setSuplimentalYawReadings(const int *currentReadings);
		
		//Accessors for the processed values
		const float rollRateInDegrees();		
		const float pitchRateInDegrees();		
		const float yawRateInDegrees();
				
		const float rollRateInRadians();		
		const float pitchRateInRadians();		
		const float yawRateInRadians();
								
		const float rollAngleInDegrees();		
		const float pitchAngleInDegrees();
		const float yawAngleInDegrees();
						
		const float rollAngleInRadians();		
		const float pitchAngleInRadians();
		const float yawAngleInRadians();
};

//Working
//Just a simple Accel only filter.
class SimpleAccellOnlyIMUFilter : public IMUFilter
{
	public:
		SimpleAccellOnlyIMUFilter();
		
		virtual void initialize()	;	
		virtual void filter(const unsigned long currentTime);		
};

//Working
//Based on the code by RoyB at: http://www.rcgroups.com/forums/showpost.php?p=12082524&postcount=1286    
class ComplimentryMUFilter : public IMUFilter
{
	private:
		float timeConstantCF;
		
		float previousAngle_roll;
		float newAngle_roll;
		float newRate_roll;
		float filterTerm0_roll;
		float filterTerm1_roll;
		float filterTerm2_roll;
		
		float previousAngle_pitch;
		float newAngle_pitch;
		float newRate_pitch;
		float filterTerm0_pitch;
		float filterTerm1_pitch;
		float filterTerm2_pitch;	
		
		float previousAngle_yaw;
		float newAngle_yaw;
		float newRate_yaw;
		float filterTerm0_yaw;
		float filterTerm1_yaw;
		float filterTerm2_yaw;					

	public:
		ComplimentryMUFilter();
				
		virtual void initialize();
		virtual void filter(const unsigned long currentTime);		
};


//Not working yet.
//Just returns 0's
class DCMIMUFilter : public IMUFilter
{
	private:
		float _dt;
		
		float _DCMMatrix[3][3];		
		float _updateMatrix[3][3];
		float _temporaryMatrix[3][3];
		
		float _errorRollPitch[3];
		float _errorYaw[3];
		
		float _omegaVector[3]; //Corrected Gyro_Vector data
		float _omegaP[3];//Omega Proportional correction
		float _omegaI[3];//Omega Integrator
		float _omega[3];
		
		float _kpRollPitch;
		float _kiRollPitch;
		float _kpYaw;
		float _kiYaw;
		
		float _accelVector[3];
		float _gyroVector[3];
		
		float _accelWeight;
		
		void _matrixUpdate();				
		void _normalize();
		void _driftCorrection();
		void _eulerAngles();
		
	public:
		DCMIMUFilter();
		
		virtual void initialize();
		
		virtual void filter(const unsigned long currentTime);
};

//Working 
//Based on code at: http://gluonpilot.com
class KalmanIMUFilter : public IMUFilter
{
	private:
		struct Gyro1DKalman
		{
			/* These variables represent our state matrix x */
			float x_angle,
			      x_bias;
	
			/* Our error covariance matrix */
			float P_00,
			      P_01,
			      P_10,
			      P_11;	
	
			/* 
			 * Q is a 2x2 matrix of the covariance. Because we
			 * assuma the gyro and accelero noise to be independend
			 * of eachother, the covariances on the / diagonal are 0.
			 *
			 * Covariance Q, the process noise, from the assumption
			 *    x = F x + B u + w
			 * with w having a normal distribution with covariance Q.
			 * (covariance = E[ (X - E[X])*(X - E[X])' ]
			 * We assume is linair with dt
			 */
			float Q_angle, Q_gyro;
			/*
			 * Covariance R, our observation noise (from the accelerometer)
			 * Also assumed to be linair with dt
			 */
			float R_angle;
		};
	
		struct Gyro1DKalman _rollFilterData;
		struct Gyro1DKalman _pitchFilterData;	
	
	// Initializing the struct
		void _init(struct Gyro1DKalman *filterdata, float Q_angle, float Q_gyro, float R_angle);	
		void _predict(struct Gyro1DKalman *filterdata, const float dotAngle, const float dt);
		float _update(struct Gyro1DKalman *filterdata, const float angle_m);
		
	public:
		KalmanIMUFilter();
		virtual void initialize();
		virtual void filter(const unsigned long currentTime);	
};


//Not working yet
//Based on the code at: http://code.google.com/p/gluonpilot/source/browse/trunk/Firmware/rtos_pilot/ahrs_simple_quaternion.c
/*class QuaternionIMUFilter : public IMUFilter
{
	private:
		static double pitch_rad, roll_rad;
		static double pitch_acc, roll_acc;
		static double q[4];
		static double w; 		// speed along z axis
		
		static double p_bias;	// in rad/sec
		static double q_bias;

		void quaternionNormalize(double *q);
		void quaternionFromAttitude (const double roll, const double pitch, const double yaw, double* q);
		double quaternionToRoll (const double* q);
		double quaternionToPitch(const double* q);
		double quaternionToYaw(const double* q);
		 


	public:
		QuaternionIMUFilter();
	
		void initialize();
		void filter(const unsigned long currentTime);
		
};*/

class IMU : public SubSystem
{
	private:
		IMUHardware *_imuHardware;
		SuplimentalYawSource *_suplimentalYawSource;
		IMUFilter *_imuFilter;
		
		float _altitudeRateOfChange;
		float _lastAltitude;
		float _lastAltitudeTime;
		
	public:
		typedef enum { ArduPilotOilPan } HardwareType;
		typedef enum { SimpleAccellOnly = 0, Complimentry, DCM, Kalman, Quaternion } FilterType;
		typedef enum { HMC5843 } SuplimentalYawSourceType;
			
		IMU();
		
		virtual void initialize(const unsigned int frequency, const unsigned int offset = 0);
		virtual void process(const unsigned long currentTime);
		
		void setHardwareType(const HardwareType hardwareType);		
		void setSuplimentalYawSource(const SuplimentalYawSourceType yawSourceType);		
		void setFilterType(const FilterType filterType);
		
		void calibrateZero();
				
		//Accessors for the processed values
		const float rollRateInDegrees();
		const float pitchRateInDegrees();
		const float yawRateInDegrees();
		
		const float rollRateInRadians();
		const float pitchRateInRadians();
		const float yawRateInRadians();
		
		const float rollAngleInDegrees();
		const float pitchAngleInDegrees();
		const float yawAngleInDegrees();
		
		const float rollAngleInRadians();
		const float pitchAngleInRadians();
		const float yawAngleInRadians();
		
		const float altitudeInMeters();
		const float altitudeRateInMeters();
};

extern IMU imu;

const ArduinoShellCallback::callbackReturn _calibrateIMU(ArduinoShell &shell, const int argc, const char* argv[]);
const ArduinoShellCallback::callbackReturn _monitorIMU(ArduinoShell &shell, const int argc, const char* argv[]);

#endif //#ifndef __IMU_H__