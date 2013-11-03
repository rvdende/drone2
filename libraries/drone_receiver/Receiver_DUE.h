#ifndef _DRONE_RECEIVER_DUE_H_
#define _DRONE_RECEIVER_DUE_H_

#include "Arduino.h"

#define RISING_EDGE 1
#define FALLING_EDGE 0
#define MINONWIDTH 950
#define MAXONWIDTH 2075
#define MINOFFWIDTH 12000
#define MAXOFFWIDTH 24000

//SET YOUR PINS! TO MATCH RECIEVER CHANNELS
#define CHAN1PIN 44
#define CHAN2PIN 46
#define CHAN3PIN 48
#define CHAN4PIN 45
#define CHAN5PIN 47
#define CHAN6PIN 49
#define CHAN7PIN 50 //not used at the moment
#define CHAN8PIN 51 //not used at the moment

volatile int PPMt[16]; // unvalidated input
volatile int PPM[16];
volatile int PPMch = 255;
volatile int PPMlast=0;
volatile int pwmLast[8];


void pwmHandler(int ch, int pin) {
  int cv = TC0->TC_CHANNEL[1].TC_CV;
  if (digitalRead(pin)) {
    pwmLast[ch] = cv;
  } else {
    cv = (cv - pwmLast[ch]) / 42;
    if (cv>950 && cv<2075) {
      PPM[ch] = cv;
    }
  }
}

void ch1Handler() { pwmHandler(1, CHAN1PIN); }
void ch2Handler() { pwmHandler(2, CHAN2PIN); }
void ch3Handler() { pwmHandler(3, CHAN3PIN); }
void ch4Handler() { pwmHandler(4, CHAN4PIN); }
void ch5Handler() { pwmHandler(5, CHAN5PIN); }
void ch6Handler() { pwmHandler(6, CHAN6PIN); }
void ch7Handler() { pwmHandler(7, CHAN7PIN); } //not used at the moment
void ch8Handler() { pwmHandler(8, CHAN8PIN); } //not used at the moment


void initializeReceiver() {
  pmc_enable_periph_clk(ID_TC1);
  // use timer just for timing reference
  TC_Configure(TC0, 1, TC_CMR_TCCLKS_TIMER_CLOCK1);
  TC_Start(TC0,1);
  attachInterrupt(CHAN1PIN,ch1Handler,CHANGE);
  attachInterrupt(CHAN2PIN,ch2Handler,CHANGE);
  attachInterrupt(CHAN3PIN,ch3Handler,CHANGE);
  attachInterrupt(CHAN4PIN,ch4Handler,CHANGE);
  attachInterrupt(CHAN5PIN,ch5Handler,CHANGE);
  attachInterrupt(CHAN6PIN,ch6Handler,CHANGE);
  attachInterrupt(CHAN7PIN,ch7Handler,CHANGE);
  attachInterrupt(CHAN8PIN,ch8Handler,CHANGE);
}

int getRawChannelValue(byte channel) {  
  return PPM[channel];
}



#endif



