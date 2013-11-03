#ifndef _AEROQUAD_MOTORS_H_
#define _AEROQUAD_MOTORS_H_

#include "Arduino.h"
//#include "FlightConfigType.h"

#define MOTOR1 0
#define MOTOR2 1
#define MOTOR3 2
#define MOTOR4 3
#define MOTOR5 4
#define MOTOR6 5
#define MOTOR7 6
#define MOTOR8 7
#define MINCOMMAND 1000
#define MAXCOMMAND 2000


volatile byte numberOfMotors = 4;
int motorCommand[8] = {0,0,0,0,0,0,0,0};  // LASTMOTOR not know here, so, default at 8 @todo : Kenny, find a better way
  
void initializeMotors(byte numbers = 4);
void writeMotors();
void commandAllMotors(int command);

void pulseMotors(byte nbPulse) {
  for (byte i = 0; i < nbPulse; i++) {
    commandAllMotors(MINCOMMAND + 100);
    delay(250);
    commandAllMotors(MINCOMMAND);
    delay(250);
  }
}


#endif