// MUST HAVE THIS IN MAIN FILE
//  double forward[3] = {0.0,1.0,0.0};   // forward vector for drone, north at start
//  double up[3] = {0.0,0.0,1.0};        // up vector for drone, skywards at start


void orientationUpdate(double gx, double gy, double gz, double deltatimesec) {

  if (gx != gx) { return; }
  if (gy != gy) { return; }
  if (gz != gz) { return; }
  if (deltatimesec != deltatimesec) { return; }
  if (deltatimesec > 0.1) { return; }
  double rate = 2.0*PI;    

  gx = gx/32768.0*250.0/360.0*deltatimesec;
  gy = gy/32768.0*250.0/360.0*deltatimesec;
  gz = gz/32768.0*250.0/360.0*deltatimesec;

  //we get 2 bytes per axis, 2 bytes is -32768 to 32768 range. We have 250dps range set on the gyro.
  //we divide by 360 to get from dps (degrees per second) to radians and times by fraction of second.

  rotate(up,      forward,   gy*rate);    
  rotate(forward, up,       -gz*rate);    
  cross(right, forward, up);  //generate new right from forward and up
  normalize(right); 
  rotate(up,      right,    -gx*rate);
  rotate(forward, right,    -gx*rate);
  normalize(up); 
  normalize(forward);   
  //we only have 2 arms, opposite sides are just inverse
  copy(arm0, forward);
  copy(arm1, forward);

  //SET UP YOUR QUAD SO THE ARMS line up with your quad's arms.
  rotate(arm0, up, PI/4*-3);    //white (front)
  rotate(arm1, up, PI/-4);  //red (left side)
  
  //CALCULATE WANTED HEADING AND CURRENT HEADING DIFF  
  double currentheading[3] = {forward[0], forward[1], forward[2]};
  currentheading[2] = 0.0;
  normalize(currentheading);

  if (headingAlignmentDone == false) { //heading not locked yet
    if (northAlignmentDone == true)  //once compass is snapped to north 
    {
      wantedheading = {currentheading[0],currentheading[1],currentheading[2]};  //we save current heading as the one we want to keep
      headingAlignmentDone = true; //done. so this only runs once.
    }  
  }
  

  if (headingAlignmentDone == true) { //heading is locked
    //CALCULATE headingdiff THE AMOUNT WE ARE OFF TARGET in YAW
    double headingcross[3];
    cross(headingcross, currentheading, wantedheading);
    headingcross[0] = 0.0;
    headingcross[1] = 0.0;  
    normalize(headingcross);      
    headingdiff = angle(currentheading, wantedheading) * headingcross[2];
    if (headingdiff != headingdiff) {
      headingdiff = 0.0;
    }
  }  
}

/* ################################################################ */

//compass calibration 
//double m_min[3] = {-707.0, -717.0, -461.0};
//double m_max[3] = {-109.0,   70.0,  123.0};

double compassmin[3] = {-538.0, -614.0, -455.0};
double compassmax[3] = {78.0,   67.0,  116.0};



void orientationDrift(double ax,double ay,double az,double mx,double my,double mz) {
  accelvec = { ax,  -ay,  az};
  normalize(accelvec);
  
  compassvec = {(double) mx, (double) my, (double) mz};  
  compassvec[0] = ((compassvec[0]-compassmin[0]) / (compassmax[0] - compassmin[0])) - 0.5;
  compassvec[1] = ((compassvec[1]-compassmin[1]) / (compassmax[1] - compassmin[1])) - 0.5;
  compassvec[2] = ((compassvec[2]-compassmin[2]) / (compassmax[2] - compassmin[2])) - 0.5;  
  
  normalize(compassvec);
  compassvec[0] *= -1;
  //compassvec[1] *= -1;
  compassvec[2] *= -1;  
	/*compassmin[0] = min(compassmin[0], mx);
	compassmin[1] = min(compassmin[1], my);
	compassmin[2] = min(compassmin[2], mz);
	compassmax[0] = max(compassmax[0], mx);
	compassmax[1] = max(compassmax[1], my);
	compassmax[2] = max(compassmax[2], mz);*/

  //compassvec[0] = (((double) mx - m_min[0]) / (m_max[0] - m_min[0]) * 2) - 1.0;
  //compassvec[1] = (((double) my - m_min[1]) / (m_max[1] - m_min[1]) * 2) - 1.0;
  //compassvec[2] = (((double) mz - m_min[2]) / (m_max[2] - m_min[2]) * 2) - 1.0;
  //compassvec[1] *= -1.0;
  //normalize(compassvec);
  
  forwardtest   = { 0.0, 1.0, 0.0};
  uptest        = { 0.0, 0.0, 1.0};
  righttest     = { 1.0, 0.0, 0.0};
  
  //THIS SHOULD SNAP TO ORIENTATION
  double fdiff = angle( forward, forwardtest );  
  double fcross[3];
  cross(fcross, forwardtest, forward);
  normalize(fcross);  
  rotate(forwardtest, fcross, fdiff);    //APPLY 1/2
  rotate(uptest,      fcross, fdiff);    //APPLY 1/2
  rotate(righttest,   fcross, fdiff);    //APPLY 1/2
  double udiff = angle( up, uptest );               
  double ucross[3];
  cross(ucross, uptest, up);
  normalize(ucross);  
  rotate(forwardtest, ucross, udiff);    //APPLY 2/2, NOW ALLIGNED  
  rotate(uptest,      ucross, udiff);    //APPLY 2/2, NOW ALLIGNED  
  rotate(righttest,   ucross, udiff);    //APPLY 2/2, NOW ALLIGNED   

  
  rotate(accelvec,  fcross, fdiff);      //APPLY 1/2
  rotate(accelvec,  ucross, udiff);      //APPLY 2/2, NOW ALLIGNED 
  rotate(compassvec,  fcross, fdiff);      //APPLY 1/2
  rotate(compassvec,  ucross, udiff);      //APPLY 2/2, NOW ALLIGNED 
  
  /// CORRECT DRIFT USING ANGLE BETWEEN GRAVITY AND ACCELEROMETER VECTOR ATTACHED TO DRONE ORIENTATION
  double gravity[3] = {0.0,0.0, -1.0};
  double gdiff = angle(accelvec, gravity);  //down drift error angle
  double gcross[3];
  cross(gcross, accelvec, gravity);
  normalize(gcross);
  
  rotate(forward, gcross, gdiff*accelgain);
  rotate(up, gcross, gdiff*accelgain);  
  rotate(accelvec, gcross, gdiff*accelgain); 
  rotate(compassvec, gcross, gdiff*accelgain);   
  
  /// CALCULATE MAGNETIC DRIFT FIX
  compassvec[2] = 0.0;
  normalize(compassvec);

  
  //rotate everything so that north stays north.
    double truenorthcross[3];


    cross(truenorthcross, compassvec, truenorth);
    normalize(truenorthcross);

    double northdiff = angle(compassvec, truenorth);   
       
    if (northAlignmentDone == false) {
      if (accelcounter > SAMPLELENGTH) {
        //snap to north.
        rotate(forward,    truenorthcross, northdiff);
        rotate(up,         truenorthcross, northdiff);   
        northAlignmentDone = true;  
      }      
    } else {
      rotate(forward,    truenorthcross, northdiff*compassgain);
      rotate(up,         truenorthcross, northdiff*compassgain);     
    }
    
  

}



