/*  AeroQuad v3.0.1 - June 2012
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

  October 2013 - Implemented on L3GD20 by Rouan van der Ende

*/


#ifndef _DRONE_GYROSCOPE_L3GD20_H_
#define _DRONE_GYROSCOPE_L3GD20_H_

#include <Gyroscope.h>
#include <Wire.h>

#define GYRO_ADDRESS 0x6b
#define GYRO_WHOAMI_REG 0x0F
#define GYRO_IDENTITY 0xD4
#define GYRO_CTRL_REG1 0x20
#define GYRO_CTRL_REG2 0x21
#define GYRO_CTRL_REG3 0x22
#define GYRO_CTRL_REG4 0x23
#define GYRO_CTRL_REG5 0x24
#define GYRO_RATE_HZ 760;
// Axis inversion: -1 = invert, 1 = don't invert
int gyroAxisInversionFactor[3] = {1,-1,-1};


void initializeGyro() {
  while (gyroInitialized == false) {
    Wire.beginTransmission(GYRO_ADDRESS);
    Wire.write(GYRO_WHOAMI_REG);
    Wire.endTransmission();  
    Wire.requestFrom(GYRO_ADDRESS, 1);  
    byte i2cid = Wire.read();

    if (i2cid == GYRO_IDENTITY)
    {
      //success
      Wire.beginTransmission(GYRO_ADDRESS);
      Wire.write(GYRO_CTRL_REG1);   
      Wire.write(0xFF);       //page 31/32 datasheet. 760hz
      Wire.endTransmission();    
      gyroInitialized = true;  
    } else { Serial.println("Gyro error cannot connect"); }
  }  

  while (gyroCalibrated == false) {
    calibrateGyro();  
  }  
  
}
  


void readGyroRaw() { 
  Wire.beginTransmission(GYRO_ADDRESS);    
  Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the gyro to do slave-transmit subaddress updating.
  Wire.endTransmission();
  Wire.requestFrom(GYRO_ADDRESS, 6);
  byte xlg,xhg, ylg, yhg, zlg, zhg;
  xlg = Wire.read();
  xhg = Wire.read();
  ylg = Wire.read();
  yhg = Wire.read();
  zlg = Wire.read();
  zhg = Wire.read();
  int gyroraw[3];
  gyroraw[0] = (int16_t)(xhg << 8 | xlg);
  gyroraw[1] = (int16_t)(yhg << 8 | ylg);
  gyroraw[2] = (int16_t)(zhg << 8 | zlg);
}

void measureGyro() {
    Wire.beginTransmission(GYRO_ADDRESS);    
    Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the gyro to do slave-transmit subaddress updating.
    Wire.endTransmission();
    Wire.requestFrom(GYRO_ADDRESS, 6);
    byte xlg,xhg, ylg, yhg, zlg, zhg;
    xlg = Wire.read();
    xhg = Wire.read();
    ylg = Wire.read();
    yhg = Wire.read();
    zlg = Wire.read();
    zhg = Wire.read();
    int gyroraw[3];
    gyroraw[0] = (int16_t)(xhg << 8 | xlg);
    gyroraw[1] = (int16_t)(yhg << 8 | ylg);
    gyroraw[2] = (int16_t)(zhg << 8 | zlg);

    //apply calibration zero
    gyro[0] = (double) gyroraw[0] - gyroZero[0];
    gyro[1] = (double) gyroraw[1] - gyroZero[1];
    gyro[2] = (double) gyroraw[2] - gyroZero[2];
}


void measureGyroSum() {
 
}


void evaluateGyroRate() {
  
}




void calibrateGyro() {
  int calibrationsamples = 128;
  int findZero[calibrationsamples];
  int diff = 0;
  int gyroraw[3];

  bool moveTrigger = false;
  for (byte axis = 0; axis <= 2; axis++) 
  {
    //collect samples for axis
    for (int i=0; i < calibrationsamples; i++)
    {
      Wire.beginTransmission(GYRO_ADDRESS);    
      Wire.write(0x28 | (1 << 7)); // assert the MSB of the address to get the gyro to do slave-transmit subaddress updating.
      Wire.endTransmission();
      Wire.requestFrom(GYRO_ADDRESS, 6);
      while (Wire.available() < 6);  //wait till available  
      byte xlg,xhg, ylg, yhg, zlg, zhg;
      xlg = Wire.read();
      xhg = Wire.read();
      ylg = Wire.read();
      yhg = Wire.read();
      zlg = Wire.read();
      zhg = Wire.read();
      gyroraw[0] = (int16_t)(xhg << 8 | xlg);
      gyroraw[1] = (int16_t)(yhg << 8 | ylg);
      gyroraw[2] = (int16_t)(zhg << 8 | zlg);

      
      findZero[i] = gyroraw[axis]; 
      delayMicroseconds(1315); //760hz
    }
    
    //find average.
    double axisdrift = calculateAverage(findZero, calibrationsamples);
    if (abs(axisdrift) > 300) { 
      //bomb if we think the gyro was moved. Not just drift.
      moveTrigger = true; 
      serialStatus("Gyro Calibration failed, moved."); 
    } else {
      gyroZero[axis] = axisdrift; 
    }     
  }

  //There was no problem.
  if (moveTrigger == false) { gyroCalibrated = true; }
}

#endif