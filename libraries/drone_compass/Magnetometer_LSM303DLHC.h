/*
  AeroQuad v3.0 - April 2011
  www.AeroQuad.com 
  Copyright (c) 2011 Ted Carancho.  All rights reserved.
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


#ifndef _AEROQUAD_MAGNETOMETER_LSM303DLHC_H_
#define _AEROQUAD_MAGNETOMETER_LSM303DLHC_H_

#include "Arduino.h"
#include "Compass.h"


#define COMPASS_ADDRESS 0x1E
#define COMPASS_IDENTITY 0x10

void readSpecificMag(float *rawMag);

void initializeMagnetometer() 
{
  while (compassInitialized == false) 
  {
    Wire.beginTransmission(COMPASS_ADDRESS);
    Wire.write(0x0);
    Wire.endTransmission();       
    Wire.requestFrom(COMPASS_ADDRESS, 1);  
    byte i2cid = Wire.read();

    if (i2cid == COMPASS_IDENTITY) 
    {
      //success
      Wire.beginTransmission(COMPASS_ADDRESS);
      Wire.write(0x02);
      Wire.write(0x00);   // Continuous conversion mode  
      Wire.endTransmission();     

      Wire.beginTransmission(COMPASS_ADDRESS);
      Wire.write(0x01);
      Wire.write(0b00100000);   // +/- 1.3 Gauss gain //page 37
      Wire.endTransmission();  
      compassInitialized = true;  
      measureMagnetometer(0.0, 0.0);  // Assume 1st measurement at 0 degrees roll and 0 degrees pitch
    }    
  }  
}

void measureMagnetometerRaw() {
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(COMPASS_ADDRESS, 6);
  byte xhm, xlm, zhm, zlm, yhm, ylm;
  xhm = Wire.read();
  xlm = Wire.read();
  zhm = Wire.read();
  zlm = Wire.read();
  yhm = Wire.read();
  ylm = Wire.read();

  // combine high and low bytes
  int compassx = (int16_t)(xhm << 8 | xlm);
  int compassy = (int16_t)(yhm << 8 | ylm);
  int compassz = (int16_t)(zhm << 8 | zlm);

  compass[0] = compassx;
  compass[1] = compassy;
  compass[2] = compassz;
}


void measureMagnetometer(float roll, float pitch) {
    
  Wire.beginTransmission(COMPASS_ADDRESS);
  Wire.write(0x03);
  Wire.endTransmission();

  Wire.requestFrom(COMPASS_ADDRESS, 6);
  byte xhm, xlm, zhm, zlm, yhm, ylm;
  xhm = Wire.read();
  xlm = Wire.read();
  zhm = Wire.read();
  zlm = Wire.read();
  yhm = Wire.read();
  ylm = Wire.read();

  // combine high and low bytes
  int compassx = (int16_t)(xhm << 8 | xlm);
  int compassy = (int16_t)(yhm << 8 | ylm);
  int compassz = (int16_t)(zhm << 8 | zlm);

  rawMag[0] = compassx;
  rawMag[1] = compassy;
  rawMag[2] = compassz;

  measuredMagX = rawMag[0] + magBias[0];
  measuredMagY = rawMag[1] + magBias[1];
  measuredMagZ = rawMag[2] + magBias[2];
  
  measuredMag[0] = measuredMagX;
  measuredMag[1] = measuredMagY;
  measuredMag[2] = measuredMagZ;
  
  float cosRoll =  cos(roll);
  float sinRoll =  sin(roll);
  float cosPitch = cos(pitch);
  float sinPitch = sin(pitch);

  float magX = (float)measuredMagX * cosPitch + (float)measuredMagY * sinRoll * sinPitch + (float)measuredMagZ * cosRoll * sinPitch;
  float magY = (float)measuredMagY * cosRoll - (float)measuredMagZ * sinRoll;
  float tmp  = sqrt(magX * magX + magY * magY);
   
  hdgX = magX / tmp;
  hdgY = -magY / tmp;
}

#endif
