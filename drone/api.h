#ifndef _DRONE_API_H_
#define _DRONE_API_H_


aJsonStream serial_stream(&Serial);

/* ================================================================== */
void apiMessage(aJsonObject *msg) 
{
  aJsonObject *cmdparm = aJson.getObjectItem(msg, "cmd");
  if (cmdparm) 
  {
    String cmd = cmdparm->valuestring;
    ///////////////////////////////////////////////////////////////
    if (cmd == "takeoff") {               //prepares quad for takeoff, arms motors and spins up to idle
      armed = 1;                          //global state switch. see firmware.ino      
      //movementArmMotors();                //Warning, few second wait. This arms the ESCs
      //movementSetMotors(1055.0, 1055.0, 1055.0, 1055.0);    //idle speed. might need calibration.
    }
    ///////////////////////////////////////////////////////////////
    if (cmd == "land") {                  //should be run on landing    
      //estop();                            //stops motors. see movement.ino      
      armed = 0;                          //global state switch. see firmware.ino
    }
    ///////////////////////////////////////////////////////////////
    if (cmd == "pidon") {                 //switches stability control on
      pid = 1;

    }
    ///////////////////////////////////////////////////////////////    
    if (cmd == "pidoff") {                 //switches stability control off
      pid = 0;
    }
    ///////////////////////////////////////////////////////////////
    if (cmd == "throttle") {   
      aJsonObject *val = aJson.getObjectItem(msg, "val");
      if (val) {  
        char* tempval = val->valuestring;
        //int tempval = val->valueint;
        userthrottle = atof(tempval);   //ascii to double
      }
    }
    ///////////////////////////////////////////////////////////////
    if (cmd == "hardlimit") {   
      aJsonObject *val = aJson.getObjectItem(msg, "val");
      if (val) {  
        char* tempval = val->valuestring;
        hardlimit = atof(tempval);   //ascii to double
      }
    }
    ///////////////////////////////////////////////////////////////    
    if (cmd == "pidset") {  
      //this is more for tuning... 
      double newPgain = 0.0;
      double newIgain = 0.0;
      double newDgain = 0.0;      
      //clearPID(); //clears accumulated error. important, else flip.
      aJsonObject *valP = aJson.getObjectItem(msg, "P");
      if (valP) {
          char* tempp = valP->valuestring;
          newPgain = atof(tempp);
      }        
      aJsonObject *valI = aJson.getObjectItem(msg, "I");    
      if (valI) {
          char* tempi = valI->valuestring;
          newIgain = atof(tempi);
      }
      aJsonObject *valD = aJson.getObjectItem(msg, "D");        
      if (valD) {
          char* tempd = valD->valuestring;
          newDgain = atof(tempd);
      }              
      //stabilisationSetPIDgains(newPgain, newIgain, newDgain);    
      Serial.println("{ \"status\" : \"NEW PID SET\"}");
    }    
///////////////////////////////////////////////////////////////////////////////////////////////////
  }
}
/* ================================================================== */

#endif