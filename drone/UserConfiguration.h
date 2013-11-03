/****************************************************************************
 ************************* Hardware Configuration ***************************
 ****************************************************************************
  Uncomment to select which hardware you are using.
  Please note in most cases only select one.
  If your does not exist, add it in the correct folder and code blocks.
                                                                          */

/****************************************************************************
MAIN PROCESSOR 
Which board you are using                                             
                                                                          */

//#define PROC_UNO        // For Arduino UNO AVR 
#define PROC_DUE        // For Arduino DUE ARM 32bit based boards.

/**************************************************************************
Options																	*/

#define SERIAL  //to enable serial logging
#define API		//extends serial into a bidirection JSON api

/****************************************************************************
RECEIVER 
To get signals from your radio remote unit                                */

//#define RECEIVER_SERIAL
#define RECEIVER_DUE
//#define RECEIVER_DUE_SBUS //under development, unreliable. do not use!


/****************************************************************************
MOTOR CONTROL 
Defines where and how motor signals should be used                    */

//#define MOTOR_UNO
//#define MOTOR_I2C
#define MOTOR_DUE 				// supports up to 8 motors
								// PINS 34,36,38,40,9,8,7,6

/****************************************************************************
Flight Configuration
                                                                          */
#define quadXConfig             // 0,1,2,3 MOTORS. in X layout
//#define quadPlusConfig        // 0,1,2,3 MOTORS. in + layout
//#define hexPlusConfig
//#define hexXConfig      
//#define triConfig
//#define quadY4Config
//#define hexY6Config
//#define octoX8Config
//#define octoPlusConfig		    // EXPERIMENTAL: not completely re-tested
//#define octoXConfig			      // EXPERIMENTAL: not completely re-tested


// *******************************************************************************************************************************
// Sensors - Uncomment all you have connected.
// *******************************************************************************************************************************
// Gyro
#define GYRO_L3GD20            //pololu miniIMU

// Accelerometer
#define ACCEL_LSM303DLHC       //pololu miniIMU 

// Magnetometer compass
#define COMPASS_LSM303DLHC         //pololu miniIMU 

// Barometric 
#define ALTITUDE_BMP085

// Ultrasonic (downward to ground)
#define ALTITUDE_SRF02_ULTRASONIC

// GPS
#define GPS_ULTIMATEGPS


// *******************************************************************************************************************************
// Battery Monitor Options
// For more information on how to setup the Battery Monitor please refer to http://aeroquad.com/showwiki.php?title=Battery+Monitor
// *******************************************************************************************************************************
//#define BattMonitor			  // Enables Battery monitor
//#define BattMonitorAutoDescent  // NEED BattMonitor defined. If you want the craft to auto descent when the battery reaches the alarm voltage
//#define POWERED_BY_VIN          // NEED BattMonitor defined. Uncomment this if your v2.x shield is powered directly by the Vin/Gnd of the arduino
//
// Advanced configuration. Please refer to the wiki for instructions.
//#define BattCustomConfig DEFINE_BATTERY(0,A4,51.8,0,A3,180.3,0)
