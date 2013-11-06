/*
  AeroQuad v3.0.1 - February 2012
  www.AeroQuad.com
  Copyright (c) 2012 Ted Carancho.  All rights reserved.
  An Open Source Arduino based multicopter.
 
  This program is free software: you can redistribute it and/or modify 
  it under the terms of the GNU General Public License as published by 
  the Free Software Foundation, either version 3 of the License, or 
  (at your option) any later version. 

  This program is distributed in the hope that it will be useful, 
  but WITHOUT ANY WARRANTY; without even the implied warranty of 
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the 
  GNU General Public License for more details. 

  You should have received a copy of the GNU General Public License 
  along with this program. If not, see <http://www.gnu.org/licenses/>. 
*/

#include <math.h>

#ifndef _AEROQUAD_ACCELEROMETER_LSM303DLHC_H_
#define _AEROQUAD_ACCELEROMETER_LSM303DLHC_H_

#include <Accelerometer.h>


#define ACCEL_ADDRESS 0x19

void initializeAccel() {
  while (accelInitialized == false) {
      Wire.beginTransmission(ACCEL_ADDRESS);
      Wire.write(0x20);
      Wire.endTransmission();

      Wire.requestFrom(ACCEL_ADDRESS, 1);
      byte out = Wire.read();
      if (out!=0) {        
        // Enable Accelerometer      
        // Normal power mode, all axes enabled
        Wire.beginTransmission(ACCEL_ADDRESS);
        Wire.write(0x20);
        Wire.write(0b01100111); //200hz 0x67 //0x57  // 0x27 = 0b00100111
        //Wire.write(0b10010111); // 1.344Khz
        Wire.endTransmission();   
        Wire.beginTransmission(ACCEL_ADDRESS);
        Wire.write(0x23);
        Wire.write(0b00011000); // +/- 4G //0x08  // DLHC: enable high resolution mode
        Wire.endTransmission();
        accelInitialized = true;
      } else { Serial.println("Accel error cannot connect"); }
  }//while

  while (accelCalibrated == false) {
    calibrateAccel();  
  }

}
  
void calibrateAccel()  {
  /* we take a bunch of samples, even them out and offset for what is down. */
  

  int calibrationsamples = 128;
  int findZero[128];
  int diff = 0;
  int accelraw[3];

  bool moveTrigger = false;
  for (byte axis = 0; axis <= 2; axis++) {
    //sample data for this axis
    for (int i=0; i < calibrationsamples; i++) {

        Wire.beginTransmission(ACCEL_ADDRESS);
        Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the accelerometer to do slave-transmit subaddress updating.
        Wire.endTransmission();  
        Wire.requestFrom(ACCEL_ADDRESS, 6);
        byte xla = Wire.read();
        byte xha = Wire.read();
        byte yla = Wire.read();
        byte yha = Wire.read();
        byte zla = Wire.read();
        byte zha = Wire.read();
        int16_t accelx =  ((int16_t)(xha << 8 | xla)) >> 4;
        int16_t accely =  ((int16_t)(yha << 8 | yla)) >> 4;
        int16_t accelz =  ((int16_t)(zha << 8 | zla)) >> 4;
        
        accelraw[0] = (int) accelx;
        accelraw[1] = (int) accely;
        accelraw[2] = (int) accelz;
        
        findZero[i] = accelraw[axis]; 
        
        delayMicroseconds(3000);
    }

    //find average.
    double axisavg = calculateAverage(findZero, calibrationsamples);
    accelZero[axis] = axisavg;
    //Serial.println(accelZero[axis]);
  }

  //work out rotation vector (how much the gyro is offcenter)

  double realdown[3] = { 0.0, 0.0, -1.0 };
  double accelZeroVec[3] = { accelZero[0], accelZero[1], accelZero[2] };
  normalize(accelZeroVec);
  accelzerodiff = angle( accelZeroVec, realdown );  
  accelzerodiffcross[3];
  cross(accelzerodiffcross, accelZeroVec, realdown);
  normalize(accelzerodiffcross);  

  
  /*
  Serial.println("Acceleromter Calibration values")
  Serial.println(accelzerodiffcross[0])
  Serial.println(accelzerodiffcross[1])
  Serial.println(accelzerodiffcross[2])
  Serial.println(accelzerodiff)
  */

  rotate(accelZero, accelzerodiffcross, accelzerodiff);    


  
  accelCalibrated = true;
}

void measureAccel() {
  if (accelInitialized == false) { Serial.print("Warning Accelerometer not initialized."); }
  if (accelCalibrated == false) { Serial.print("Warning Accelerometer not calibrated."); }

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the accelerometer to do slave-transmit subaddress updating.
  Wire.endTransmission();  
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  int accelraw[3];
  accelraw[0] = ((int16_t)(xha << 8 | xla)) >> 4;
  accelraw[1] = ((int16_t)(yha << 8 | yla)) >> 4;
  accelraw[2] = ((int16_t)(zha << 8 | zla)) >> 4;

  double accelrawd[3];
  accelrawd[0] = (double) accelraw[0];
  accelrawd[1] = (double) accelraw[1];
  accelrawd[2] = (double) accelraw[2];

  ///////////////////////////////////////////////
  //calibration offset Rouan
  accelzerodiffcross = { -0.0975, 0.9952, 0.0};
  accelzerodiff = 0.033;
  ///////////////////////////////////////////////

  rotate(accelrawd, accelzerodiffcross, accelzerodiff);  //correct for physical alignment.

  accel[0] = accelrawd[0];
  accel[1] = accelrawd[1];
  accel[2] = accelrawd[2];

  //meterPerSecSec[0] = accnew0 * accelScaleFactor[0] + runTimeAccelBias[0];
  //meterPerSecSec[1] = accnew1 * accelScaleFactor[1] + runTimeAccelBias[1];  
  //meterPerSecSec[2] = accnew2 * accelScaleFactor[2] + runTimeAccelBias[2];
}

void measureAccelSum() {

  Wire.beginTransmission(ACCEL_ADDRESS);
  Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the accelerometer to do slave-transmit subaddress updating.
  Wire.endTransmission();  
  Wire.requestFrom(ACCEL_ADDRESS, 6);
  byte xla = Wire.read();
  byte xha = Wire.read();
  byte yla = Wire.read();
  byte yha = Wire.read();
  byte zla = Wire.read();
  byte zha = Wire.read();

  // combine high and low bytes, then shift right to discard lowest 4 bits (which are meaningless)
  // GCC performs an arithmetic right shift for signed negative numbers, but this code will not work
  // if you port it to a compiler that does a logical right shift instead.
  long accnew0 = ((int16_t)(xha << 8 | xla)) >> 4;
  long accnew1 = ((int16_t)(yha << 8 | yla)) >> 4;
  long accnew2 = ((int16_t)(zha << 8 | zla)) >> 4;  
  
  accelSample[0] += accnew0;
  accelSample[1] += accnew1;  
  accelSample[2] += accnew2;
  accelSampleCount++;
}

void evaluateMetersPerSec() {
	
  for (byte axis = 0; axis <= 2; axis++) {
    meterPerSecSec[axis] = (accelSample[axis] / accelSampleCount) * accelScaleFactor[axis] + runTimeAccelBias[axis];
	accelSample[axis] = 0;
  }
  accelSampleCount = 0;		
}

void computeAccelBias() {
  
  for (int samples = 0; samples < SAMPLECOUNT; samples++) {
    measureAccelSum();
    delayMicroseconds(2500);
  }

  for (byte axis = 0; axis < 3; axis++) {
    meterPerSecSec[axis] = (float(accelSample[axis])/SAMPLECOUNT) * accelScaleFactor[axis];
    accelSample[axis] = 0;
  }
  accelSampleCount = 0;

  runTimeAccelBias[0] = -meterPerSecSec[0];
  runTimeAccelBias[1] = -meterPerSecSec[1];
  runTimeAccelBias[2] = -9.8065 - meterPerSecSec[2];

  accelOneG = fabs(meterPerSecSec[2] + runTimeAccelBias[2]);
}



#endif
