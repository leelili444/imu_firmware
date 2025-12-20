#include "ins_task.h"

// Initialize algorithms
FusionOffset offset;
FusionAhrs ahrs;
FusionEuler euler;


// Define calibration (replace with actual calibration data if available)
const FusionMatrix gyroscopeMisalignment = {
	    .array = {
	        {1.0f, 0.0f, 0.0f},
	        {0.0f, 1.0f, 0.0f},
	        {0.0f, 0.0f, 1.0f}
	    }
	};
const FusionVector gyroscopeSensitivity = {{1.0f, 1.0f, 1.0f}};
const FusionVector gyroscopeOffset = {{0.0f, 0.0f, 0.0f}};
const FusionMatrix accelerometerMisalignment = {
	    .array = {
	        {1.0f, 0.0f, 0.0f},
	        {0.0f, 1.0f, 0.0f},
	        {0.0f, 0.0f, 1.0f}
	    }
	};
const FusionVector accelerometerSensitivity = {{1.0f, 1.0f, 1.0f}};
const FusionVector accelerometerOffset = {{0.0f, 0.0f, 0.0f}};
const FusionMatrix softIronMatrix = {
	    .array = {
	        {1.0f, 0.0f, 0.0f},
	        {0.0f, 1.0f, 0.0f},
	        {0.0f, 0.0f, 1.0f}
	    }
	};
const FusionVector hardIronOffset = {{0.0f, 0.0f, 0.0f}};



void InitializePose(void){
	FusionOffsetInitialise(&offset, SAMPLE_RATE);
	FusionAhrsInitialise(&ahrs);
    // Set AHRS algorithm settings
	const FusionAhrsSettings settings = {
	        .convention = FusionConventionNwu,
	        .gain = 0.5f,
	        .gyroscopeRange = 2000.0f, /* replace this with actual gyroscope range in degrees/s */
	        .accelerationRejection = 10.0f,
	        .magneticRejection = 10.0f,
	        .recoveryTriggerPeriod = 5 * SAMPLE_RATE, /* 5 seconds */
	};
	FusionAhrsSetSettings(&ahrs, &settings);
}
void GetPose(ICM42688P_Data_t *imudata){
	// Acquire latest sensor data
	FusionVector gyroscope = {{imudata->gyro_x,imudata->gyro_y,imudata->gyro_z}}; //  degrees/s
	FusionVector accelerometer = {{imudata->accel_x,imudata->accel_y,imudata->accel_z}}; //g
    // Apply calibration
    gyroscope = FusionCalibrationInertial(gyroscope, gyroscopeMisalignment, gyroscopeSensitivity, gyroscopeOffset);
    accelerometer = FusionCalibrationInertial(accelerometer, accelerometerMisalignment, accelerometerSensitivity, accelerometerOffset);
    // Update gyroscope offset correction algorithm
    gyroscope = FusionOffsetUpdate(&offset, gyroscope);
    // Calculate delta time (in seconds) to account for gyroscope sample clock error
    // Update gyroscope AHRS algorithm
    if( offset.gyroscopeOffset.axis.z!=0){
    	FusionAhrsUpdateNoMagnetometer(&ahrs, gyroscope, accelerometer, 1e-6*imudata->dt);
    // Print algorithm outputs
    	euler = FusionQuaternionToEuler(FusionAhrsGetQuaternion(&ahrs));
    }
    //delCNT = GetTimeUS_TIM()-preCNT;
  }
