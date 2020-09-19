/*
  License
  Copyright (c) 2013-2017 by Alexander Grau

  Private-use only! (you need to ask for a commercial-use)

  The code is open: you can modify it under the terms of the
  GNU General Public License as published by the Free Software Foundation,
  either version 3 of the License, or (at your option) any later version.

  The code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with this program.  If not, see <http://www.gnu.org/licenses/>.

  Private-use only! (you need to ask for a commercial-use)
  /************************************************************
  MPU9250_DMP_Quaternion
  Quaternion example for MPU-9250 DMP Arduino Library
  Jim Lindblom @ SparkFun Electronics
  original creation date: November 23, 2016
  https://github.com/sparkfun/SparkFun_MPU9250_DMP_Arduino_Library

  The MPU-9250's digital motion processor (DMP) can calculate
  four unit quaternions, which can be used to represent the
  rotation of an object.

  This exmaple demonstrates how to configure the DMP to
  calculate quaternions, and prints them out to the serial
  monitor. It also calculates pitch, roll, and yaw from those
  values.

  Development environment specifics:
  Arduino IDE 1.6.12
  SparkFun 9DoF Razor IMU M0

*/

#include "imu.h"
#include "SparkFunMPU9250-DMP.h"
#include "mower.h"
//#include "i2c.h"
#include "robot.h"

//#include "flashmem.h"


MPU9250_DMP imu;


#define ADDR 600
#define MAGIC 6
//#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)

void IMUClass::begin() {
  if (!robot.imuUse) return;
  Console.println(F("--------------------------------------------------------------------------"));
  Console.println(F("--------------------------------- START IMU ------------------------------"));
  Console.println(F("--------------------------------------------------------------------------"));
  if (imu.begin() != INV_SUCCESS)
  {
    while (1)
    {
      Console.println("Unable to communicate with MPU-9250");
      Console.println("Check connections, and try again.");
      delay(5000);
    }
  }
  else {
    Console.println("Connected with MPU-9250");
  }

  imu.dmpBegin(DMP_FEATURE_6X_LP_QUAT | // Enable 6-axis quat
               DMP_FEATURE_GYRO_CAL, // Use gyro calibration
               20);
  // Set DMP FIFO rate to 20 Hz because imu is read each 50 ms
  // DMP_FEATURE_6X_LP_QUAT is used to not read the compass
  // DMP_FEATURE_GYRO_CAL is used to automaticly calibrate the gyro when no move for 8 secondes

  nextTimeAdjustYaw = millis();
  Console.println("Wait 9 secondes for GYRO calibration");
  delay(9000);
  // read the AccelGyro and the CompassHMC5883 to find the initial CompassYaw
  int uu=0;
  for (uu = 0; uu < 11; uu++) {
    run();
    Console.print("Initial GYRO/ACCELL Yaw :");
    Console.print(imu.yaw * 180 / PI) ;
    Console.print(" Pitch : ");
    Console.print(imu.pitch * 180 / PI) ;
    Console.print(" Roll : ");
    Console.println(imu.roll * 180 / PI);
    delay(100);
  }



  Console.println("Yaw Pitch and Roll need to be near 0.00 if calibration is OK :");





  if (robot.CompassUse) {
    Console.print(F("  Compass Yaw: "));
    Console.print(comYaw);
    CompassGyroOffset = distancePI(ypr.yaw, comYaw);
    Console.print(F("  Diff between compass and accelGyro in Radian and Deg"));
    Console.print(CompassGyroOffset);
    Console.print(" / ");
    Console.println(CompassGyroOffset * 180 / PI);
  }
  else
  {
    Console.println("Compass is not used");
    CompassGyroOffset = 0;
  }




  Console.println(F("--------------------------------- IMU READY ------------------------------"));
}



float IMUClass::distance180(float x, float w)
{
  float d = scale180(w - x);
  if (d < -180) d = d + 2 * 180;
  else if (d > 180) d = d - 2 * 180;
  return d;
}

//bb
float IMUClass::scale180(float v)
{
  float d = v;
  while (d < 0) d += 2 * 180;
  while (d >= 2 * 180) d -= 2 * 180;
  if (d >= 180) return (-2 * 180 + d);
  else if (d < -180) return (2 * 180 + d);
  else return d;
}
float IMUClass::rotate360(float x)
{
  x += 360;
  if (x >= 360) x -= 360;
  if (x < 0) x += 360;
  return x;
}



void IMUClass::run() {
  if (!robot.imuUse)  return;

  if (state == IMU_CAL_COM) { //don't read the MPU9250 if compass calibration
    calibComUpdate();
    return;
  }
  //-------------------read the mpu9250 DMP  --------------------------------
  // Check for new data in the FIFO
  if ( imu.fifoAvailable() )
  {

    // Use dmpUpdateFifo to update the ax, gx, mx, etc. values
    if (imu.dmpUpdateFifo() == INV_SUCCESS)
    {
      // computeEulerAngles can be used -- after updating the
      // quaternion values -- to estimate roll, pitch, and yaw
      imu.computeEulerAngles();

    }
  }

  //bber4
  //filter to avoid bad reading
  /*
    if ((abs(imu.pitch) - abs(ypr.pitch)) > 0.3490)
    {
      Console.print("Last pitch : ");
      Console.print(ypr.pitch);
      Console.print(" Actual pitch : ");
      Console.println(imu.pitch);
      Console.println("pitch change 20 deg in less than 50 ms ????????? value is skip");
    }
    else
    {
      ypr.pitch = imu.pitch ;
    }

    if ((abs(imu.roll) - abs(ypr.roll)) > 0.3490)
    {
      Console.print("Last roll : ");
      Console.print(ypr.roll);
      Console.print(" Actual roll : ");
      Console.println(imu.roll);
      Console.println("roll change 20 deg in less than 50 ms ????????? value is skip");
    }
    else
    {
      ypr.roll = imu.roll;
    }
  */
  ypr.pitch = imu.pitch ;
  ypr.roll = imu.roll ;
  gyroAccYaw = imu.yaw;  // the Gyro Yaw very accurate but drift


  if (robot.CompassUse) {
    // ------------------put the CompassHMC5883 value into comYaw-------------------------------------
    //readHMC5883L();

    comTilt.x =  com.x  * cos(ypr.pitch) + com.z * sin(ypr.pitch);
    comTilt.y =  com.x  * sin(ypr.roll)         * sin(ypr.pitch) + com.y * cos(ypr.roll) - com.z * sin(ypr.roll) * cos(ypr.pitch);
    comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);
    comYaw = scalePI( atan2(comTilt.y, comTilt.x)  ); // the compass yaw not accurate but reliable
  }
  else
  {
    CompassGyroOffset = 0;
  }

  // / CompassGyroOffset=distancePI( scalePI(ypr.yaw-CompassGyroOffset), comYaw);
  ypr.yaw = scalePI(gyroAccYaw + CompassGyroOffset) ;

}



void IMUClass::loadSaveCalib(boolean readflag) {
  /*
    int addr = ADDR;
    short magic = MAGIC;
    if (readflag) Console.println(F("Load Calibration"));
    else Console.println(F("Save Calibration"));
    eereadwrite(readflag, addr, magic); // magic
    //accelgyro offset
    eereadwrite(readflag, addr, ax_offset);
    eereadwrite(readflag, addr, ay_offset);
    eereadwrite(readflag, addr, az_offset);
    eereadwrite(readflag, addr, gx_offset);
    eereadwrite(readflag, addr, gy_offset);
    eereadwrite(readflag, addr, gz_offset);
    //compass offset
    eereadwrite(readflag, addr, comOfs);
    eereadwrite(readflag, addr, comScale);
    Console.print(F("Calibration address Start = "));
    Console.println(ADDR);
    Console.print(F("Calibration address Stop = "));
    Console.println(addr);
  */
}


void IMUClass::printPt(point_float_t p) {
  Console.print(p.x);
  Console.print(",");
  Console.print(p.y);
  Console.print(",");
  Console.println(p.z);
}
void IMUClass::printCalib() {
  /*
    Console.println(F("-------- IMU CALIBRATION  --------"));
    Console.print("ACCEL GYRO MPU6050 OFFSET ax: ");
    Console.print(ax_offset);
    Console.print(" ay: ");
    Console.print(ay_offset);
    Console.print(" az: ");
    Console.print(az_offset);
    Console.print(" gx: ");
    Console.print(gx_offset);
    Console.print(" gy: ");
    Console.print(gy_offset);
    Console.print(" gz: ");
    Console.println(gz_offset);
    Console.print("COMPASS OFFSET X.Y.Z AND SCALE X.Y.Z   --> ");
    Console.print(F("comOfs="));
    printPt(comOfs);
    Console.print(F("comScale="));
    printPt(comScale);
    Console.println(F("."));
  */
}

void IMUClass::loadCalib() {
  /*
    short magic = 0;
    int addr = ADDR;
    eeread(addr, magic);
    if (magic != MAGIC) {
    Console.println(F("IMU error: no calib data"));
    ax_offset = 376;
    ay_offset = -1768;
    az_offset = 1512;
    gx_offset = 91;
    gy_offset = 12;
    gz_offset = -2;
    comOfs.x = comOfs.y = comOfs.z = 0;
    comScale.x = comScale.y = comScale.z = 2;
    useComCalibration = false;
    return;
    }
    calibrationAvail = true;
    useComCalibration = true;
    Console.println(F("IMU: found calib data"));
    loadSaveCalib(true);
  */
}

void IMUClass::saveCalib() {
  loadSaveCalib(false);
}


void IMUClass::deleteCompassCalib() {
  /*
    int addr = ADDR;
    eewrite(addr, (short)0); // magic
    comOfs.x = comOfs.y = comOfs.z = 0;
    comScale.x = comScale.y = comScale.z = 2;
    Console.println("Compass calibration deleted");
  */
}
void IMUClass::deleteAccelGyroCalib() {
  /*
    int addr = ADDR;
    eewrite(addr, (short)0); // magic
    ax_offset = ay_offset = az_offset = 0;
    gx_offset = gy_offset = gz_offset = 0;


    Console.println("AccelGyro calibration deleted ");
  */
}


// calculate gyro offsets
void IMUClass::calibGyro() {
  /*
    Console.println("Reading sensors for first time... without any offset");
    watchdogReset();
    // meansensors();
    watchdogReset();
    Console.print("Reading ax: ");
    Console.print(mean_ax);
    Console.print(" ay: ");
    Console.print(mean_ay);
    Console.print(" az: ");
    Console.print(mean_az);
    Console.print(" gx: ");
    Console.print(mean_gx);
    Console.print(" gy: ");
    Console.print(mean_gy);
    Console.print(" gz: ");
    Console.println(mean_gz);

    Console.println("\nCalculating offsets...");
    watchdogReset();
    //calibration();
    watchdogReset();
    //meansensors();
    watchdogReset();
    /*
    Console.print("FINISHED reading Value with new offset,If all is OK need to be close 0 exept the az close to 16384");
    Console.print(" New reading ax: ");
    Console.print(mean_ax);
    Console.print(" ay: ");
    Console.print(mean_ay);
    Console.print(" az: ");
    Console.print(mean_az);
    Console.print(" gx: ");
    Console.print(mean_gx);
    Console.print(" gy: ");
    Console.print(mean_gy);
    Console.print(" gz: ");
    Console.println(mean_gz);
    watchdogReset();
    Console.print("THE NEW OFFSET ax: ");
    Console.print(ax_offset);
    Console.print(" ay: ");
    Console.print(ay_offset);
    Console.print(" az: ");
    Console.print(az_offset);
    Console.print(" gx: ");
    Console.print(gx_offset);
    Console.print(" gy: ");
    Console.print(gy_offset);
    Console.print(" gz: ");
    Console.println(gz_offset);
    watchdogReset();
    saveCalib();
  */

}


void IMUClass::calibComStartStop() {
  /*
    while ((!robot.RaspberryPIUse) && (Console.available())) Console.read(); //use to stop the calib
    if (state == IMU_CAL_COM) {
    // stop
    Console.println(F("com calib completed"));
    calibrationAvail = true;
    float xrange = comMax.x - comMin.x;
    float yrange = comMax.y - comMin.y;
    float zrange = comMax.z - comMin.z;
    comOfs.x = xrange / 2 + comMin.x;
    comOfs.y = yrange / 2 + comMin.y;
    comOfs.z = zrange / 2 + comMin.z;
    comScale.x = xrange;
    comScale.y = yrange;
    comScale.z = zrange;
    //bber18
    if ((comScale.x == 0) || (comScale.y == 0) || (comScale.z == 0)) { //jussip bug found div by 0 later
      Console.println(F("*********************ERROR WHEN CALIBRATE : VALUE ARE TOTALY WRONG CHECK IF COMPASS IS NOT PERTURBATE **************"));
      comOfs.x = comOfs.y = comOfs.z = 0;
      comScale.x = comScale.y = comScale.z = 2;
    }
    saveCalib();
    printCalib();
    useComCalibration = true;
    state = IMU_RUN;


    } else {
    // start
    Console.println(F("com calib..."));
    Console.println(F("rotate sensor 360 degree around all three axis until NO new data are coming"));
    watchdogReset();
    foundNewMinMax = false;
    useComCalibration = false;
    state = IMU_CAL_COM;
    comMin.x = comMin.y = comMin.z = 9999;
    comMax.x = comMax.y = comMax.z = -9999;
    }
  */
}
void IMUClass::calibComUpdate() {
  /*
    comLast = com;
    delay(20);
    readHMC5883L();
    watchdogReset();
    boolean newfound = false;
    if ( (abs(com.x - comLast.x) < 10) &&  (abs(com.y - comLast.y) < 10) &&  (abs(com.z - comLast.z) < 10) ) {
    if (com.x < comMin.x) {
      comMin.x = com.x;
      newfound = true;
    }
    if (com.y < comMin.y) {
      comMin.y = com.y;
      newfound = true;
    }
    if (com.z < comMin.z) {
      comMin.z = com.z;
      newfound = true;
    }
    if (com.x > comMax.x) {
      comMax.x = com.x;
      newfound = true;
    }
    if (com.y > comMax.y) {
      comMax.y = com.y;
      newfound = true;
    }
    if (com.z > comMax.z) {
      comMax.z = com.z;
      newfound = true;
    }
    if (newfound) {
      foundNewMinMax = true;
      watchdogReset();
      Console.print("x:");
      Console.print(comMin.x);
      Console.print(",");
      Console.print(comMax.x);
      Console.print("\t  y:");
      Console.print(comMin.y);
      Console.print(",");
      Console.print(comMax.y);
      Console.print("\t  z:");
      Console.print(comMin.z);
      Console.print(",");
      Console.print(comMax.z);
      Console.println("\t");
    }
    }
  */
}
