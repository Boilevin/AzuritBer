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

*/

#include "imu.h"
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"
#include "mower.h"
#include "i2c.h"
#include "robot.h"
//#include "buzzer.h"
#include "flashmem.h"


///////////////////////////////////   CONFIGURATION FOR ACCEL GYRO MPU6050 CALIBRATION  /////////////////////////////
//Change this 3 variables if you want to fine tune the skecth to your needs.
int buffersize = 1000;   //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone = 8;   //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone = 1;   //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)



MPU6050 mpu(0x69);


bool blinkState = false;
float nextTimeLoop;
// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer
//for accelgyro calib
int16_t ax, ay, az, gx, gy, gz;
int mean_ax, mean_ay, mean_az, mean_gx, mean_gy, mean_gz, state = 0;
int ax_offset, ay_offset, az_offset, gx_offset, gy_offset, gz_offset;

// orientation/motion vars
//Quaternion q;           // [w, x, y, z]         quaternion container
//VectorInt16 aa;         // [x, y, z]            accel sensor measurements
//VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
//VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
//VectorFloat gravity;    // [x, y, z]            gravity vector
//float euler[3];         // [psi, theta, phi]    Euler angle container
float yprtest[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
//uint8_t teapotPacket[14] = { '$', 0x02, 0, 0, 0, 0, 0, 0, 0, 0, 0x00, 0x00, '\r', '\n' };


#define ADDR 600
#define MAGIC 6
#define HMC5883L (0x1E)          // HMC5883L compass sensor (GY-80 PCB)

void IMUClass::begin() {
  if(!robot.imuUse) return;
  //initialisation of CompassHMC5833L
  Console.println(F("--------------------------------- IMU INITIALISATION -------------------"));
  uint8_t data = 0;
  //while (true) {
    I2CreadFrom(HMC5883L, 10, 1, &data, 1);
    Console.print(F("COMPASS HMC5883L ID NEED TO BE 72 IF ALL IS OK ------>  ID="));
    Console.println(data);
    if (data != 72) Console.println(F("COMPASS HMC5883L FAIL"));
    delay(1000);
  //}
  comOfs.x = comOfs.y = comOfs.z = 0;
  comScale.x = comScale.y = comScale.z = 2;
  loadCalib();
  printCalib();
  useComCalibration = true;
  I2CwriteTo(HMC5883L, 0x00, 0x70);  // config A:  8 samples averaged, 75Hz frequency, no artificial bias.
  I2CwriteTo(HMC5883L, 0x01, 0x20);  // config B: gain
  I2CwriteTo(HMC5883L, 0x02, 00);  // mode: continuous
  delay(2000); //    wait 2 second before doing the first reading
  
  //initialisation of MPU6050
  mpu.initialize();
  Console.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));
  Console.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  
  mpu.setXAccelOffset(ax_offset); //-1929
  mpu.setYAccelOffset(ay_offset); //471
  mpu.setZAccelOffset(az_offset); // 1293
  mpu.setXGyroOffset(gx_offset);//-1
  mpu.setYGyroOffset(gy_offset);//-3
  mpu.setZGyroOffset(gz_offset);//-2
/*
  mpu.setXAccelOffset(-1929); //-1929
  mpu.setYAccelOffset(471); //471
  mpu.setZAccelOffset(1293); // 1293
  mpu.setXGyroOffset(-1);//-1
  mpu.setYGyroOffset(-3);//-3
  mpu.setZGyroOffset(-2);//-2
 */ 
  CompassGyroOffset=0;

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    Console.print(F("Enabling DMP...  "));
    mpu.setDMPEnabled(true);
    packetSize = mpu.dmpGetFIFOPacketSize();
    Console.print(F("Packet size "));
    Console.println(packetSize);
  }
  else  Console.println(F("DMP Initialization failed "));
  nextTimeAdjustYaw = millis();
  Console.println(F("Wait 3 secondes to stabilize the Drift"));
  delay(3000); // wait 3 sec to help DMP stop drift
// read the AccelGyro and the CompassHMC5883 to find the initial CompassYaw
  
  run();
  Console.print(F("AccelGyro Yaw: "));
  Console.print(ypr.yaw);
  Console.print(F("  Compass Yaw: "));
  Console.print(comYaw);
  
  CompassGyroOffset=distancePI(ypr.yaw, comYaw);
  Console.print(F("  Diff between compass and accelGyro in Radian and Deg"));
  Console.print(CompassGyroOffset);
  Console.print(" / ");
  Console.println(CompassGyroOffset*180/PI);
  
  Console.println(F("--------------------------------- IMU READY ------------------------------"));
}

// weight fusion (w=0..1) of two radiant values (a,b)
float IMUClass::fusionPI(float w, float a, float b)
{
  float c;
  if ((b >= PI / 2) && (a <= -PI / 2)) {
    c = w * a + (1.0 - w) * (b - 2 * PI);
  } else if ((b <= -PI / 2) && (a >= PI / 2)) {
    c = w * (a - 2 * PI) + (1.0 - w) * b;
  } else c = w * a + (1.0 - w) * b;
  return scalePI(c);
}

// first-order complementary filter
// newAngle = angle measured with atan2 using the accelerometer
// newRate = angle measured using the gyro
// looptime = loop time in millis()
float Complementary2(float newAngle, float newRate, int looptime, float angle) {
  float k = 10;
  float dtc2 = float(looptime) / 1000.0;
  float x1 = (newAngle -   angle) * k * k;
  float y1 = dtc2 * x1 + y1;
  float x2 = y1 + (newAngle -   angle) * 2 * k + newRate;
  angle = dtc2 * x2 + angle;
  return angle;
}

// scale setangle, so that both PI angles have the same sign
float scalePIangles(float setAngle, float currAngle) {
  if ((setAngle >= PI / 2) && (currAngle <= -PI / 2)) return (setAngle - 2 * PI);
  else if ((setAngle <= -PI / 2) && (currAngle >= PI / 2)) return (setAngle + 2 * PI);
  else return setAngle;
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

void IMUClass::calibration() {
  ax_offset = -mean_ax / 8;
  ay_offset = -mean_ay / 8;
  az_offset = (16384 - mean_az) / 8;

  gx_offset = -mean_gx / 4;
  gy_offset = -mean_gy / 4;
  gz_offset = -mean_gz / 4;
  while (1) {
    int ready = 0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Console.println("Wait ...");

    if (abs(mean_ax) <= acel_deadzone) ready++;
    else ax_offset = ax_offset - mean_ax / acel_deadzone;

    if (abs(mean_ay) <= acel_deadzone) ready++;
    else ay_offset = ay_offset - mean_ay / acel_deadzone;

    if (abs(16384 - mean_az) <= acel_deadzone) ready++;
    else az_offset = az_offset + (16384 - mean_az) / acel_deadzone;

    if (abs(mean_gx) <= giro_deadzone) ready++;
    else gx_offset = gx_offset - mean_gx / (giro_deadzone + 1);

    if (abs(mean_gy) <= giro_deadzone) ready++;
    else gy_offset = gy_offset - mean_gy / (giro_deadzone + 1);

    if (abs(mean_gz) <= giro_deadzone) ready++;
    else gz_offset = gz_offset - mean_gz / (giro_deadzone + 1);

    if (ready == 6) break;
  }
}
void IMUClass::meansensors() {
  long i = 0, buff_ax = 0, buff_ay = 0, buff_az = 0, buff_gx = 0, buff_gy = 0, buff_gz = 0;

  while (i < (buffersize + 101)) {  //default buffersize=1000
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    if (i > 100 && i <= (buffersize + 100)) { //First 100 measures are discarded
      buff_ax = buff_ax + ax;
      buff_ay = buff_ay + ay;
      buff_az = buff_az + az;
      buff_gx = buff_gx + gx;
      buff_gy = buff_gy + gy;
      buff_gz = buff_gz + gz;
    }
    if (i == (buffersize + 100)) {
      mean_ax = buff_ax / buffersize;
      mean_ay = buff_ay / buffersize;
      mean_az = buff_az / buffersize;
      mean_gx = buff_gx / buffersize;
      mean_gy = buff_gy / buffersize;
      mean_gz = buff_gz / buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}










void IMUClass::run() {
if(!robot.imuUse)  return;
if(devStatus != 0) return;
  if (state == IMU_CAL_COM) { //don't read the MPU6050 if compass calibration
    calibComUpdate();
    return;
  }
  //-------------------read the mpu6050 DMP into yprtest array--------------------------------
  mpu.resetFIFO();
  while (fifoCount < packetSize) {  //leave time to the DMP to fill the FIFO
    fifoCount = mpu.getFIFOCount();
  }
  while (fifoCount >= packetSize) {  //Immediatly read the fifo and verify that it's not filling during reading
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
  }
  if (fifoCount != 0) {
    //Console.println("////// the DMP fill the fifo during the reading IMU value are skip //////////////");
    return;  ///the DMP fill the fifo during the reading , all the value are false but without interrupt it's the only way i have find to make it work ???certainly i am wrong
  }
  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(yprtest, &q, &gravity);
  gyroAccYaw = yprtest[0];  // the Gyro Yaw very accurate but drift
  ypr.pitch = yprtest[1];
  ypr.roll = yprtest[2];

  // ------------------put the CompassHMC5883 value into comYaw-------------------------------------
  readHMC5883L();
  //tilt compensed yaw ????????????
  comTilt.x =  com.x  * cos(ypr.pitch) + com.z * sin(ypr.pitch);
  comTilt.y =  com.x  * sin(ypr.roll)         * sin(ypr.pitch) + com.y * cos(ypr.roll) - com.z * sin(ypr.roll) * cos(ypr.pitch);
  comTilt.z = -com.x  * cos(ypr.roll)         * sin(ypr.pitch) + com.y * sin(ypr.roll) + com.z * cos(ypr.roll) * cos(ypr.pitch);
  comYaw = scalePI( atan2(comTilt.y, comTilt.x)  ); // the compass yaw not accurate but reliable
  //last traitement to verify drift
  /*
    Console.print ("IMU.YAW ");
    Console.print (gyroAccYaw);
    Console.print (" Compass.YAW ");
    Console.print (comYaw);
    Console.print (" firstStartCompassYaw ");
    Console.println (firstStartCompassYaw);
    //Console.print (" stopYawCompass ");
    
  */
 
// / CompassGyroOffset=distancePI( scalePI(ypr.yaw-CompassGyroOffset), comYaw);
 ypr.yaw = scalePI(gyroAccYaw + CompassGyroOffset) ;    
  

  
  /*
    if (millis() > timeToAdjustGyroToCompass) {
      Console.println(ypr.yaw);
      //firstStartCompassYaw = comYaw - gyroAccYaw;
      timeToAdjustGyroToCompass = millis() + 200;
    }
*/
  
}

void IMUClass::readHMC5883L() {
  uint8_t buf[6];
  if (I2CreadFrom(HMC5883L, 0x03, 6, (uint8_t*)buf) != 6) {
    //errorCounter++;
    return;
  }
  // scale +1.3Gauss..-1.3Gauss  (*0.00092)
  float x = (int16_t) (((uint16_t)buf[0]) << 8 | buf[1]);
  float y = (int16_t) (((uint16_t)buf[4]) << 8 | buf[5]);
  float z = (int16_t) (((uint16_t)buf[2]) << 8 | buf[3]);
  if (useComCalibration) {
    x -= comOfs.x;
    y -= comOfs.y;
    z -= comOfs.z;
    x /= comScale.x * 0.5;
    y /= comScale.y * 0.5;
    z /= comScale.z * 0.5;
    com.x = x;
    com.y = y;
    com.z = z;
  } else {
    com.x = x;
    com.y = y;
    com.z = z;
  }

}


void IMUClass::loadSaveCalib(boolean readflag) {
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
}


void IMUClass::printPt(point_float_t p) {
  Console.print(p.x);
  Console.print(",");
  Console.print(p.y);
  Console.print(",");
  Console.println(p.z);
}
void IMUClass::printCalib() {
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
}

void IMUClass::loadCalib() {
  short magic = 0;
  int addr = ADDR;
  eeread(addr, magic);
  if (magic != MAGIC) {
    Console.println(F("IMU error: no calib data"));
    ax_offset=376;
    ay_offset=-1768;
    az_offset=1512;
    gx_offset=91;
    gy_offset=12;
    gz_offset=-2;
    return;
  }
  calibrationAvail = true;
  Console.println(F("IMU: found calib data"));
  loadSaveCalib(true);
}

void IMUClass::saveCalib() {
  loadSaveCalib(false);
}


void IMUClass::deleteCompassCalib() {
  int addr = ADDR;
  eewrite(addr, (short)0); // magic
  comOfs.x = comOfs.y = comOfs.z = 0;
  comScale.x = comScale.y = comScale.z = 2;
  Console.println("Compass calibration deleted");
}
void IMUClass::deleteAccelGyroCalib() {
  int addr = ADDR;
  eewrite(addr, (short)0); // magic
  ax_offset = ay_offset = az_offset = 0;
  gx_offset = gy_offset = gz_offset = 0;
  mpu.setXAccelOffset(1); //-1929
  mpu.setYAccelOffset(1); //471
  mpu.setZAccelOffset(1); // 1293
  mpu.setXGyroOffset(1);//-1
  mpu.setYGyroOffset(1);//-3
  mpu.setZGyroOffset(1);//-2
 
  Console.println("AccelGyro calibration deleted ");
}


// calculate gyro offsets
void IMUClass::calibGyro() {

    Console.println("Reading sensors for first time... without any offset");
    meansensors();
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
     delay(500);
    Console.println("\nCalculating offsets...");
    calibration();
    delay(500);
    meansensors();
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
    saveCalib();
}


void IMUClass::calibComStartStop() {
  while ((!robot.RaspberryPIUse) && (Console.available())) Console.read();
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
    saveCalib();
    printCalib();
    useComCalibration = true;
    state = IMU_RUN;
    // completed sound
    robot.setBeeper(2000,30,0,160,0);
    
  } else {
    // start
    Console.println(F("com calib..."));
    Console.println(F("rotate sensor 360 degree around all three axis until NO new data are coming"));
    foundNewMinMax = false;
    useComCalibration = false;
    state = IMU_CAL_COM;
    comMin.x = comMin.y = comMin.z = 9999;
    comMax.x = comMax.y = comMax.z = -9999;
  }
}
void IMUClass::calibComUpdate() {
  comLast = com;
  delay(20);
  readHMC5883L();
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
}
