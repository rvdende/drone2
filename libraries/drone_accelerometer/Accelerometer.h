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

#ifndef _AEROQUAD_ACCELEROMETER_H_
#define _AEROQUAD_ACCELEROMETER_H_

#include "Arduino.h"

#define SAMPLECOUNT 400.0

bool accelInitialized = false;  
bool accelCalibrated = false;    

double accelZero[3] = {0.0,0.0,0.0}; 
double accelzerodiff;  
double accelzerodiffcross[3];


float accelScaleFactor[3] = {0.0,0.0,0.0};
float runTimeAccelBias[3] = {0, 0, 0};
float accelOneG = 0.0;
float meterPerSecSec[3] = {0.0,0.0,0.0};
long accelSample[3] = {0,0,0};
byte accelSampleCount = 0;

double accelDataRateHz = 200.0;        //how many samples per second
double accelDataRateSec = 1.0/200.0;     //optimal time between samples, related to above
  
void initializeAccel();
void calibrateAccel();
void measureAccel();
void measureAccelSum();
void evaluateMetersPerSec();
void computeAccelBias();
  
#endif