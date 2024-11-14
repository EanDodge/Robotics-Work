/**********************************************************************;
* Project           : [Intro-Robotics] ATR TurtleBot Nano 
* Program name      : ATR_TurtleBot_Nano_Init.ino
* Author            : Jong-Hoon Kim
* Date created      : 04/23/2024
* Purpose           : Inititalization of a Simple Rescue Robot 
* Revision History  :
* Date        Author      Ref    Revision (Date in MMDDYYYY format) 
* MM/DD/YYYY
*
*********************************************************************/
/*********************************************************************
*   Instructional Note:  
*           This code is for a RPI-Pico W board                      
***********************************************************************/

#ifndef ATR_TurtleBot_Nano_H
#define ATR_TurtleBot_Nano

// Motor control pins
#define RightMotor_E1  8    // GP8
#define RightMotor_E2  9   // GP9
#define LeftMotor_E1  14   // GP14
#define LeftMotor_E2  15   // GP15
#define RightMotor_INT2  10  // GP10
#define RightMotor_INT1  11  // GP11
#define LeftMotor_INT1  12   // GP12
#define LeftMotor_INT2  13   // GP13



// LED pins
#define Led_1  16 // led 1
#define Led_2  17 // led 2
#define Led_3  18 // led 3

// Bottom IR sensor
#define IRSensor_1  26  // Analog pin 0 (pin 26)
#define IRSensor_2  27  // Analog pin 1 (pin 27)


#define LEFT  0
#define RIGHT 1
#define PREV  0
#define NOW   1
#define LINEAR  0
#define ANGULAR 1

#define INTERVAL_MOTOR_TEST       5000
#define INTERVAL_SONAR_TEST       500
#define INTERVAL_IMU_TEST         2000
#define INTERVAL_DEBUG_PRINT      2000

#define MIN_SONAR_ANGLE      30
#define MID_SONAR_ANGLE      90
#define MAX_SONAR_ANGLE      150
#define STRID_SONAR_ANGLE     5

#define ODOM_MSG                         "o"  // pose x, y, z, speed liner, angular
#define CMD_MSG                          "c"
#define GENERAL_MSG                      "g"
#define TORQUE_MSG                       "t"

#define MESSAGE_BUFFER_SIZE              64

#endif // ATR_TurtleBot_Nano