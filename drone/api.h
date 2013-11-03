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
      stabilisationSetPIDgains(newPgain, newIgain, newDgain);    
      Serial.println("{ \"status\" : \"NEW PID SET\"}");
    }    
///////////////////////////////////////////////////////////////////////////////////////////////////
  }
}
/* ================================================================== */

#endif