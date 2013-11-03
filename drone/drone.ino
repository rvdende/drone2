#include "UserConfiguration.h"  // Edit this file first before uploading to the drone


#include <Wire.h>               // Needed for I2C sensors





//how fast we correct gyro drift. see orientationDrift()
//based on accelerometer (feeling which way is down)
//This should be fine as long as your gyro drift speed is low enough to not overpower this.

//////////////////////////////////////

                              
double accelgain = 0.05;      // higher corrects drift faster. too high with above drag value too low will 
                              // overpower gyro and update waaay too slow
                              // higher values introduce more vibrational noise. 
                              // lower makes it less locked to world orientation

double compassgain = 0.05;

//////////////////////////////////////////

int armed = 0;     // ready to fly? see takeoff in api.ino
int pid = 0;       
int gyrostate;     // is gyro working
int compassstate;  // is accel/magnetometer working
int barostate;     // is altimeter working




// These get updated as fast as possible
// SENSORS
#define SAMPLELENGTH 3     //15 Samples at 760hz works out to 0.02sec maximum lag

int recievercounter = 0;
int gyrocounter = 0;
int accelcounter = 0;
int compasscounter = 0;
int orientationcounter = 0;

double gyro[3]         = {0,0,0};   
double gyro0[SAMPLELENGTH];
double gyro1[SAMPLELENGTH];
double gyro2[SAMPLELENGTH];

//double accel[3]        = {0,0,0};              
double accel0[SAMPLELENGTH];
double accel1[SAMPLELENGTH];
double accel2[SAMPLELENGTH];

double compass[3]      = {0,0,0};              // L3GD20_read(); to update
double compass0[SAMPLELENGTH];
double compass1[SAMPLELENGTH];
double compass2[SAMPLELENGTH];






// VECTORS / SELF
double forward[3]    = {   0.0, 1.0, 0.0};        // forward vector for drone, north at start
double up[3]         = {   0.0, 0.0, 1.0};        // up vector for drone, skywards at start
double right[3]      = {   1.0, 0.0, 0.0};        // right vector for drone, for consistency and easier calculations.
double arm0[3]       = {  -1.0, 1.0, 0.0};
double arm1[3]       = {  -1.0,-1.0, 0.0};

double accelvec[3]   = {   0.0, 0.0,-1.0};
double compassvec[3] = {   0.0, -1.0,-1.0};

double forwardtest[3]= {   0.0, 1.0, 0.0};
double uptest[3]     = {   0.0, 0.0, 1.0};
double righttest[3]  = {   1.0, 0.0, 0.0};

double motorthrottles[4] = {0,0,0,0};
double userthrottle = 1000.0;              //from the HUD, this should control altitude indirectly. W/S key in HUD
double userpitch = 0;
double userroll = 0;
double useryaw = 0;

double hardlimit = 1060.0;                //absolute max motor speed. this even overrides stability control.

double wantedheading[3] = {0.0,1.0,0.0};  //this should come from the HUD/user
double headingdiff = 0.0;

//WORLD
double xaxis[3] = {  1.0,0.0,0.0};
double yaxis[3] = {  0.0,1.0,0.0};
double zaxis[3] = {  0.0,0.0,1.0};

long prevtime = micros();
long nowtime, deltatime;
double deltatimeseconds = 0.001;
int counter = 0;

/**********************************************************************
  LOAD
*********************************************************************/

#include <droneMath.h>
#include "orientation.h"  
#include "stabilisation.h"  

#ifdef API
  #include <aJSON.h>
  #include "api.h"    
#endif  

#ifdef ACCEL_LSM303DLHC
  #include <Accelerometer_LSM303DLHC.h>
#endif  
#ifdef GYRO_L3GD20 
  #include <Gyroscope_L3GD20.h>
#endif
#ifdef COMPASS_LSM303DLHC
  #include <Magnetometer_LSM303DLHC.h>
#endif
#ifdef RECEIVER_JSON
  #include <Receiver_JSON.h>
#endif
#ifdef RECEIVER_DUE
  #include <Receiver_DUE.h>
#endif
#ifdef RECEIVER_DUE_SBUS
  #include <Receiver_DUE_SBUS.h>
#endif
#ifdef MOTOR_DUE
  #include <Motors_DUE.h>
#endif

/**********************************************************************
  SETUP
*********************************************************************/

void setup() {
  pinMode(13, OUTPUT);
  pinMode(20,INPUT);
  pinMode(21,INPUT);
  delay(1000);
  Wire.begin();
  #ifdef SERIAL
    Serial.begin(115200);
  #endif

  serialStatus("initializing motors"); 
  initializeMotors();

  serialStatus("initializing gyro"); 
  initializeGyro();  //zero calibration is now part of initialization process.

  serialStatus("initializing compass"); 
  initializeMagnetometer();

  serialStatus("initializing reciever"); 
  initializeReceiver();   
}

/**********************************************************************  
  LOOP
*********************************************************************/

void loop() {  
  digitalWrite(13, armed);
  //INPUT API
  /*
  #ifdef API
    while (serial_stream.available()) 
    {
      aJsonObject *msg = aJson.parse(&serial_stream);
      apiMessage(msg);  //see api.ino
      aJson.deleteItem(msg);  
    } 
  #endif  


  */
  if (newReceiver()) {
    //testreciever(); //print receiver to serial. useful for setting trims.

    if (getRawChannelValue(1) > 0) {    //if signal      
      if (getRawChannelValue(5) > 1500) {

        //FIRST ARMING, DO LOCK OF MAGNETIC NORTH.
        if (northlocked == false) {
          doNorthLock = true;
        }

        armed = 1;
        digitalWrite(13, 1);
      } else { 
        armed = 0;
        digitalWrite(13, 0);
      }
    }
  }

  
  if (newAccel()) { }  
  if (newGyro()) { }
  if (newOrientationUpdate()) { }
  
  #ifdef API
    newJSONOUT();
  #endif

  //############################
  // UNCOMMENT FOR DEBUG INFO!
  //############################
  // printtests();
  //############################
  
  counter++;  
}

/**********************************************************************
  HELPER FUNCTIONS
*********************************************************************/

void serialStatus(char* message) {
  #ifdef SERIAL
    #ifdef API
      Serial.print("{\"status\" : \"");
      Serial.print(message);
      Serial.println("\"}"); 
    #else
      Serial.println(message);  //just plain messages to serial
    #endif     
  #endif           
}


/**********************************************************************
  TIMER FUNCTIONS
*********************************************************************/

// RECIEVER

//This function gives us true or false and limits true to 100hz. 
//Its a template to limit certain things at certain Hz.
unsigned long timerReceiver;
bool newReceiver() {
 if(abs(micros() - timerReceiver) >= 0.01*1000000) // 0.01sec is 100hz. We times by million because micros() is in millionths
 {
  recievercounter++;
  timerReceiver = micros();
  return true;
 } else {
  return false;
 }
}

unsigned long timerJSONOUT;
bool newJSONOUT() {
 if(abs(micros() - timerJSONOUT) >= 0.02*1000000) // 50hz
  {    
    double delllta = (double) abs(micros() - timerJSONOUT) / 1000000;
    timerJSONOUT = micros();   
    Serial.print("{\"f\":["); 
    Serial.print(forward[0], 4);
    Serial.print(",");
    Serial.print(forward[1], 4);  
    Serial.print(",");
    Serial.print(forward[2], 4);  
    Serial.print("], \"u\":[");
    Serial.print(up[0], 4); 
    Serial.print(",");
    Serial.print(up[1], 4);   
    Serial.print(",");
    Serial.print(up[2], 4);   
    Serial.print("],\"accel\":[");
    Serial.print(accelvec[0], 4); 
    Serial.print(",");
    Serial.print(accelvec[1], 4);   
    Serial.print(",");
    Serial.print(accelvec[2], 4);   
    Serial.print("],\"compass\":[");
    Serial.print(compassvec[0]); 
    Serial.print(",");
    Serial.print(compassvec[1]);   
    Serial.print(",");
    Serial.print(compassvec[2]);   
    Serial.print("],\"motorthrottles\":[");
    Serial.print(motorCommand[0]); 
    Serial.print(",");
    Serial.print(motorCommand[1]);   
    Serial.print(",");
    Serial.print(motorCommand[2]);       
    Serial.print(",");
    Serial.print(motorCommand[3]);   
    Serial.print("], \"headingdiff\": \"");
    Serial.print(headingdiff, 4); 
    Serial.print("\",");
    Serial.print("\"arm0\":[");
    Serial.print(arm0[0], 4); 
    Serial.print(",");
    Serial.print(arm0[1], 4); 
    Serial.print(",");
    Serial.print(arm0[2], 4);    
    Serial.print("], \"arm1\":[");
    Serial.print(arm1[0], 4); 
    Serial.print(",");
    Serial.print(arm1[1], 4); 
    Serial.print(",");
    Serial.print(arm1[2], 4);  
    Serial.print("],\"t\":");
    Serial.print(delllta);   
    Serial.println("}");   
  }
}


/////////////////////////////////////////////
// ACCEL (WELL NOT JUST ACCEL, WE DO COMPASS AND DRIFT CORRECTIONS TOO)
unsigned long timerAccel;
bool newAccel() {
 if(abs(micros() - timerAccel) >= 0.01*1000000) 
  {
    accelcounter++;
    timerAccel = micros();    

    measureAccel();  //get readings from your accelerometer
    
    pushShiftArray(accel0, SAMPLELENGTH, accel[0]); //push into array;
    pushShiftArray(accel1, SAMPLELENGTH, accel[1]); //push into array;
    pushShiftArray(accel2, SAMPLELENGTH, accel[2]); //push into array;
    accel[0] = calculateAverageDouble(accel0, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    accel[1] = calculateAverageDouble(accel1, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    accel[2] = calculateAverageDouble(accel2, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    
    measureMagnetometerRaw();

    pushShiftArray(compass0, SAMPLELENGTH, compass[0]); //push into array;
    pushShiftArray(compass1, SAMPLELENGTH, compass[1]); //push into array;
    pushShiftArray(compass2, SAMPLELENGTH, compass[2]); //push into array;
    compass[0] = calculateAverageDouble(compass0, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    compass[1] = calculateAverageDouble(compass1, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    compass[2] = calculateAverageDouble(compass2, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.

    orientationDrift(accel[0],accel[1],accel[2],compass[0],compass[1],compass[2]); 

    return true;
  } else {
    return false;  
  }  
}


/////////////////////////////////////////////
// GYRO

// This function only samples the gyro at its datasheet (and I2C register settings) rate. 760hz in my case.
unsigned long timerGyro;
bool newGyro() {
 if(abs(micros() - timerGyro) >= gyroDataRateSec*1000000) //abs for when micros() rolls over. gyroDataRateSec is set in Gyroscope.h and your Gyroscope_XXXXX.h
  {
    gyrocounter++;

    //double deltatimeseconds = (double) abs(micros() - timerGyro) / 1000000.0;

    timerGyro = micros();
    measureGyro();  //get readings from your gyro.         
    pushShiftArray(gyro0, SAMPLELENGTH, gyro[0]); //push into array;
    pushShiftArray(gyro1, SAMPLELENGTH, gyro[1]); //push into array;
    pushShiftArray(gyro2, SAMPLELENGTH, gyro[2]); //push into array;
    gyro[0] = calculateAverageDouble(gyro0, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    gyro[1] = calculateAverageDouble(gyro1, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.
    gyro[2] = calculateAverageDouble(gyro2, SAMPLELENGTH); //calculates new moving average of last samples. This should clean the noise.

    //if (deltatimeseconds > 0.1) { return false; }

    //Serial.print(deltatimeseconds, 5);
    
    return true;
  } else {
    return false;  
  }  
}

/////////////////////////////////////////////
// ORIENTATION
unsigned long timerOrientation;
bool newOrientationUpdate() {
 if(abs(micros() - timerOrientation) >= 0.01*1000000) //abs for when micros() rolls over. gyroDataRateSec is set in Gyroscope.h and your Gyroscope_XXXXX.h  
  {
    orientationcounter++;
    double deltatimeseconds = (double) abs(micros() - timerOrientation) / 1000000.0;
    timerOrientation = micros();  
    //Serial.println(deltatimeseconds,5);

    orientationUpdate(gyro[0], gyro[1], gyro[2], deltatimeseconds);
  
    userthrottle = getRawChannelValue(3);
    userpitch = getRawChannelValue(1)-1500;
    userroll = getRawChannelValue(2)-1500;
    useryaw = getRawChannelValue(4)-1500;    

    double pidoutA = 0;
    double pidoutB = 0;
    if (deltatimeseconds < 0.5) {
      pidoutA = pid_A_calcPID(arm0[2], 0.0, deltatimeseconds);  //BLUE  
      pidoutB = pid_B_calcPID(arm1[2], 0.0, deltatimeseconds);  //RED      
    }
    
    //do proportional control. SEE stabilisation.ino and api.ino
    if (armed == 1) {
        
        
        //double pidoutC = pid_C_calcPID(headingdiff, 0.0, deltatimeseconds);  //GREEN          

        
        /*motorCommand[0] = userthrottle+pidoutA+pidoutC;
        motorCommand[1] = userthrottle+pidoutB-pidoutC;
        motorCommand[2] = userthrottle-pidoutA+pidoutC; 
        motorCommand[3] = userthrottle-pidoutB-pidoutC;
        */
        motorCommand[0] = userthrottle + pidoutA - userpitch - userroll + useryaw;
        motorCommand[1] = userthrottle + pidoutB - userpitch + userroll - useryaw; 
        motorCommand[2] = userthrottle - pidoutA + userpitch + userroll + useryaw;
        motorCommand[3] = userthrottle - pidoutB + userpitch - userroll - useryaw;
        writeMotors();
      } else {
        clearPID();
        //RAW control.
        motorCommand[0] = userthrottle - userpitch - userroll + useryaw;
        motorCommand[1] = userthrottle - userpitch + userroll - useryaw; 
        motorCommand[2] = userthrottle + userpitch + userroll + useryaw;
        motorCommand[3] = userthrottle + userpitch - userroll - useryaw;
        writeMotors();
    }



      
    return true;
  } else {
    return false;  
  }  
}

/**********************************************************************
  SERIAL DEBUG PRINT FUNCTIONS

  these print out data to serial so you might see whats going on.

*********************************************************************/
unsigned long timer;
void printtests() {
 if((millis() - timer) > 1000) //1sec
  {
    timer = millis();  
    Serial.print(" OrientationPSec: ");
    Serial.print(orientationcounter); 
    Serial.print(" AccelPSec: ");
    Serial.print(accelcounter); 
    Serial.print(" GyroPSec: ");
    Serial.print(gyrocounter);
    Serial.print(" RecieverPSec: ");
    Serial.print(recievercounter); 
    Serial.println();      

    

    //testreciever(); 
    //testaccel();
    //testgyro();   

    orientationcounter = 0;
    recievercounter = 0;
    gyrocounter = 0;
    accelcounter = 0;
    compasscounter = 0;        
  }
}


void testreciever() {
    Serial.print(" CH1: ");
    Serial.print(getRawChannelValue(1));  //ELEV UP/DOWN
    Serial.print(" CH2: ");
    Serial.print(getRawChannelValue(2));  //AILER ROLL LEFT/RIGHT
    Serial.print(" CH3: ");
    Serial.print(getRawChannelValue(3));  //THROTTLE
    Serial.print(" CH4: ");
    Serial.print(getRawChannelValue(4));  //RUDDER TURN LEFT/RIGHT
    Serial.print(" CH5: ");
    Serial.print(getRawChannelValue(5));  //EXTRA SWITCH G
    Serial.print(" CH6: ");
    Serial.print(getRawChannelValue(6));  //EXTRA DIAL VR
    Serial.print(" CH7: ");
    Serial.print(getRawChannelValue(7));
    Serial.print(" CH8: ");
    Serial.print(getRawChannelValue(8));
    Serial.println();
  
}

void testaccel() {
    Serial.print(" ax: ");
    Serial.print(accel[0],4);
    Serial.print(" ay: ");
    Serial.print(accel[1],4);
    Serial.print(" az: ");
    Serial.print(accel[2],4);
    Serial.println();
}


void testgyro() {
  /*
  DEBUG gyro raw array before averaging
  for (int x = 0; x < SAMPLELENGTH; x++) {
    Serial.print(gyro0[x], 1);
    Serial.print(" ");
  }
  */  
  Serial.print(" gx: ");
  Serial.print(gyro[0],4);
  Serial.print(" gy: ");
  Serial.print(gyro[1],4);
  Serial.print(" gz: ");
  Serial.print(gyro[2],4);  
  Serial.println();
}

void apiJSONout() {
  
}