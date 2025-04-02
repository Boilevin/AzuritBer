/*
  // IMU (GY-88)
  //  Gyro:    MPU6050
  //  Compass: HMC5883L

  How to use:

	IMU.begin();
	while (true){
	  if (IMU.needGyroCal())) {
	    IMU.startGyroCalibration();
    }
	  IMU.run();
		float yaw = IMU.getYaw();
	}

*/


#ifndef IMU_H
#define IMU_H

#include "helper_3dmath.h"
//#include "adafruit/Adafruit_Sensor.h"
//#include "adafruit/Adafruit_BNO055.h"
//#include "adafruit/imumaths.h"


// Sensor sensitivities
#define QUAT_SENS   1073741824.0
#define ACCEL_SENS  16384.0
#define GYRO_SENS   16.375


enum IMUMode { IMU_MODE_COM_TILT, IMU_MODE_COM_FLAT } ;
typedef enum IMUMode IMUMode;

enum IMUState { IMU_RUN, IMU_CAL_GYRO, IMU_CAL_COM } ;
typedef enum IMUState IMUState;


struct point_int_t {
  int16_t x;
  int16_t y;
  int16_t z;
};
typedef struct point_int_t point_int_t;

struct point_long_t {
  long x;
  long y;
  long z;
};
typedef struct point_long_t point_long_t;

struct point_float_t {
  float x;
  float y;
  float z;
};
typedef struct point_float_t point_float_t;

struct ypr_t {
  float yaw;
  float pitch;
  float roll;
};
typedef struct ypr_t ypr_t;

class IMUClass {
  public:
    float fusionPI(float w, float a, float b);
    float distance180(float x, float w);
    float rotate360(float x);
    float scale180(float v);
    float CompassGyroOffset;           // compass yaw ofs (for gyro correction)

    IMUState state;
    // ---- mpu -------------------------------------------------
    ypr_t ypr;              // yaw, pitch, roll   robot yaw/pitch/roll
    VectorFloat gravity;    // x, y, z            gravity vector
    point_float_t gyro;     // current gyro
    point_float_t com;      // current compass (calibrated)
    point_float_t acc;      // current acceleration sensor
    IMUMode mode;           // IMU mode
    point_float_t gyroSum;
    int gyroSumCount;
    float accXmin;
    float accXmax;
    float gyroZlowpass;
    boolean enabled;
    boolean isRotating;
    boolean verboseOutput;
    boolean useGyro;
    float gyroBiasDpsMax;
    //bb5

    float timeToAdjustGyroToCompass;









    unsigned long lastAHRSTime;
    float gyroAccYaw;
    float ecartYprCompass;
    float nextTimeAdjustYaw;
    boolean calibrationAvail;
    // --------- compass state --------------------------

    point_float_t comLast;
    point_float_t comMin; // compass sensor data (raw)
    point_float_t comMax; // compass sensor data (raw)
    point_float_t comTilt; // compass sensor data (tilt corrected)
    point_float_t comOfs;
    point_float_t comScale;
    float comYaw;         // compass heading (radiant, raw)
    boolean useComCalibration;
    // calibrate compass sensor
    void calibComStartStop();
    void calibComUpdate();
    void calibComQMC5883Update();

    boolean newMinMaxFound();
    void calibGyro();
    // ----------------------
    /*  // ---- compass ------
      boolean calibFound;
      //adafruit_bno055_offsets_t calibData; // BNO055 calibration
      point_float_t comR; // raw compass (uncalibrated)
      point_float_t comAcc; // compass acceleration sensor
      float comRoll; // compass roll
      float comPitch; // compass pitch
       point_float_t comLast;
       point_float_t comMin; // compass sensor data (raw)
       point_float_t comMax; // compass sensor data (raw)
       point_float_t comTilt; // compass sensor data (tilt corrected)
       point_float_t comOfs;
       point_float_t comScale;

      point_float_t comAccMin; // compass acceleration sensor seen min
      point_float_t comAccMax; // compass acceleration sensor seen max
      long gyroBias[3];    // gyro bias
      long accelCal[3];   // accel calibration
      float comCalA_1[9]; // compass calibration A_1
      float comCalB[3];   // compass calibration B
      point_float_t comAccOfs; // compass acceleration sensor ofs
      point_float_t comAccScale;  // compass acceleration sensor scale
         boolean useComAccCalibration;
      int calibComAccAxisCounter ;
      float yawAtGyroCalibrationTime;
      unsigned long timeAtGyroCalibrationTime;
      float statsYawMax;
      float statsYawMin;
      int statsGyroCalibrationTimeMax;
    */

    void meansensors();
    void calibration();



    void begin();
    void run();
    float getYaw();
    boolean needGyroCal();
    boolean needCompassCal();
    void startGyroCalibration();
    void calibrateAcceleration();
    void startCompassCalibration();
    void stopCompassCalibration();
    void loadCalib();
    void saveCalib();
    void deleteCompassCalib();
    void deleteAccelGyroCalib();

    void calibrateAccel();
    void runSelfTest(boolean accelSelfTest = true);
    void setNextMode();
  protected:
    int calibrationTime;

    Quaternion q;           // w, x, y, z         quaternion container
    unsigned long nextBeepTime;
    unsigned long nextComTime;
    unsigned long nextInfoTime;
    unsigned long nextGyroCalTime;
    unsigned long gyroCalStartTime;
    unsigned long gyroCalStopTime;
    unsigned long comMinMaxTimeout;

    boolean foundNewMinMax;
    boolean calibrateGyro();

    void printPt(point_float_t p);
    void loadSaveCalib(boolean readflag);
    void initSensors();
    //void readCompassMPU9150();
    void readCompassHMC5883();
    // void readAccelerationADXL345B();
    //void readCompassBNO055();
    //void readCompassCMPS11();

    void printCalib();
    //void initAccelerationADXL345B();
    //void initCompassHMC5833L();

    void readHMC5883L();
    void readQMC5883L();

};


extern IMUClass IMU;

#endif
