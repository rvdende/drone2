#ifndef _DRONE_GYROSCOPE_H_
#define _DRONE_GYROSCOPE_H_
#include "Arduino.h"


//ALL OF THESE SHOULD BE SET DURING THE initializeGyro() FUNCTION
bool gyroInitialized = false;  
bool gyroCalibrated = false;                                    
float gyroRate[3] = {0.0,0.0,0.0};

double gyroZero[3] = {0.0,0.0,0.0};		//calibration offsets //important!

double gyroDataRateHz = 760.0;				//how many samples per second
double gyroDataRateSec = 1.0/760.0;			//optimal time between samples, related to above

long  gyroSample[3] = {0,0,0};
float gyroScaleFactor = 0.0;
float gyroHeading = 0.0;
unsigned long gyroLastMesuredTime = 0;
byte gyroSampleCount = 0;



void measureGyroSum();
void evaluateGyroRate();
void initializeGyro();
void measureGyro();
void calibrateGyro();
void readGyroTemp();



#endif