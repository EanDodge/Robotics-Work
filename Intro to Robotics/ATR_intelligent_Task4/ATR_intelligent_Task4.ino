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
#include "ATR_TurtleBot_Nano.h"
#include "SimpleDeadReckoning.h"
#include "DRV8833.h"
#include <Arduino.h>
#include <U8g2lib.h>
#include <Wire.h>
#include <MPU6050_light.h>
#include <Servo.h>
#include <WiFi.h>
#include <WiFiUdp.h>

#ifndef STASSID
//#define STASSID "ATR_Lab"
#define STASSID "RRL_Kent"
#define STAPSK "ksu_robotics"
#endif

unsigned int localPort = 8888;  // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged\r\n";        // a string to send back

// WIFI setup
WiFiUDP Udp;
// Create an instance of the DRV8833:
DRV8833 driver = DRV8833();
// OLED setup
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/ U8X8_PIN_NONE);
// IMU setup
MPU6050 mpu(Wire);

SimpleDeadReckoning mySDR( 122.0, 9.8, 60, 0);   // encoder values per one rotation,  wheel radius, distance between two wheels, unit (cm)

int irValue1 = 0; int irValue2 = 0; 
long odomVal_R = 0; long odomVal_L = 0;
int cnt = 0;

long counterLeftWheel[2] = {0,0}; long counterRightWheel[2] = {0,0};

int motorSpeed[2] = {0,0};   // current left/right motor speed {range -255 ~ + 255}
float cmd_vel[2] = {0.0, 0.0};    // LINEAR 0, ANGULAR 1
float odom_pose[3] = {0.0, 0.0, 0.0};
float odom_vel[2] = {0.0, 0.0};
int controlMode = 0;         // manual control = 0, line following = 1,

long cTime = 0;     // current time of each iteration 
long lTime = 0;     // localization interval
long mTime = 0;     // motor interval
long sTime = 0;     // sonar interval
long iTime = 0;     // IMU interval
long dTime = 0;     // debug print interval
unsigned long timer = 0;



int prevLeftCLK, prevLeftDT, nowLeftCLK, nowLeftDT = 0;
int prevRightCLK, prevRightDT, nowRightCLK, nowRightDT = 0;
long lCounter, rCounter = 0;
float cTheta, xLocation, yLocation = 0;

float thetaOffset = 0.0;


long enError_R = 0;long enError_L = 0;

void setup() {
  // put your setup code here, to run once:
  initPins();
  initComm();
  initSensors();
  initDisplay();
  initMotor();
  initLocalization();

  attachInterrupt(digitalPinToInterrupt(LeftMotor_E1), checkLeftEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(LeftMotor_E2), checkLeftEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RightMotor_E1), checkRightEncoder, CHANGE); 
  attachInterrupt(digitalPinToInterrupt(RightMotor_E2), checkRightEncoder, CHANGE); 
}


void loop() {
  // put your main code here, to run repeatedly:
  if (((cnt++)%100)==0){
      irValue1 = analogRead(IRSensor_1); irValue2 = analogRead(IRSensor_2); 
//    Serial.println(cnt);
//    Serial.print(lCounter);  Serial.print("["); Serial.print(enError_L);  Serial.print("]  : "); Serial.print(rCounter); Serial.print(" ["); Serial.print(enError_R);  Serial.print("]  = "); 
    // Serial.print(digitalRead(LeftMotor_E1));  Serial.print("  : ");  Serial.print(digitalRead(LeftMotor_E2)); Serial.print(" = "); 
    // Serial.print(digitalRead(RightMotor_E1));  Serial.print(" : ");  Serial.println(digitalRead(RightMotor_E2));
//    displayIRValues( irValue1, irValue2); 
    displayOdom();
    runLocalization();
//    displayIRValues( lCounter, rCounter); 
  }
  runLineFollow();
//  delay(1);
//  driver.motorAForward(50); driver.motorBForward(250);
//  checkOdom();

  // prevLeftCLK = nowLeftCLK;
  // prevLeftDT = nowLeftDT;
  // nowLeftCLK = digitalRead(LeftMotor_E1);
  // nowLeftDT = digitalRead(LeftMotor_E2);
  // if ((prevLeftCLK !=nowLeftCLK ) || (prevLeftDT !=nowLeftDT)){
  //   Serial.print(nowLeftCLK); Serial.print(":"); Serial.println(nowLeftDT);
  // }

}

void checkOdom(){
  checkLeftEncoder();
  checkRightEncoder();
}

void checkLeftEncoder(){
  prevLeftCLK = nowLeftCLK;
  prevLeftDT = nowLeftDT;
  nowLeftCLK = digitalRead(LeftMotor_E1);
  nowLeftDT = digitalRead(LeftMotor_E2);


  if ((prevLeftCLK == 0) && (prevLeftDT == 0)){
      if ((nowLeftCLK == 0) && (nowLeftDT == 1)){
        lCounter--;//lCounter++;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 0)){
        lCounter++;//lCounter--;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
        enError_L++;
      } 
  }else if ((prevLeftCLK == 0) && (prevLeftDT == 1)){
      if ((nowLeftCLK == 0) && (nowLeftDT == 0)){
        lCounter++; //lCounter--;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
        lCounter--; //lCounter++;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
        enError_L++;
      }   
  }else if ((prevLeftCLK == 1) && (prevLeftDT == 0)){
      if ((nowLeftCLK == 0) && (nowLeftDT == 0)){
        lCounter--; //lCounter++;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 1)){
        lCounter++; //lCounter--;
      }else if ((nowLeftCLK == 0) && (nowLeftDT == 1)){
        enError_L++;
      } 
  }else if ((prevLeftCLK == 1) && (prevLeftDT == 1)){
      if ((nowLeftCLK == 0) && (nowLeftDT == 1)){
        lCounter++; //lCounter--;
      }else if ((nowLeftCLK == 1) && (nowLeftDT == 0)){
        lCounter--; //lCounter++;
      }else if ((nowLeftCLK == 0) && (nowLeftDT == 0)){
        enError_L++;
      } 
   }
}
// 0:0
// 0:1
// 1:1
// 1:0


void checkRightEncoder(){
  prevRightCLK = nowRightCLK;
  prevRightDT = nowRightDT;
  nowRightCLK = digitalRead(RightMotor_E1);
  nowRightDT = digitalRead(RightMotor_E2);
  if ((prevRightCLK == 0) && (prevRightDT == 0)){
      if ((nowRightCLK == 0) && (nowRightDT == 1)){
        rCounter++;
      }else if ((nowRightCLK == 1) && (nowRightDT == 0)){
        rCounter--;
      }else if ((nowRightCLK == 1) && (nowRightDT == 1)){
        enError_R++;
      } 
  }else if ((prevRightCLK == 0) && (prevRightDT == 1)){
      if ((nowRightCLK == 0) && (nowRightDT == 0)){
        rCounter--;
      }else if ((nowRightCLK == 1) && (nowRightDT == 1)){
        rCounter++;
      }else if ((nowRightCLK == 1) && (nowRightDT == 0)){
        enError_R++;
      }   
  }else if ((prevRightCLK == 1) && (prevRightDT == 0)){
      if ((nowRightCLK == 0) && (nowRightDT == 0)){
        rCounter++;
      }else if ((nowRightCLK == 1) && (nowRightDT == 1)){
        rCounter--;
      }else if ((nowRightCLK == 0) && (nowRightDT == 1)){
        enError_R++;
      } 
  }else if ((prevRightCLK == 1) && (prevRightDT == 1)){
      if ((nowRightCLK == 0) && (nowRightDT == 1)){
        rCounter--;
      }else if ((nowRightCLK == 1) && (nowRightDT == 0)){
        rCounter++;
      }else if ((nowRightCLK == 0) && (nowRightDT == 0)){
        enError_R++;
      } 
   }
}



void runLocalization(){
  cTime = millis ();
//  recieved_py();
  
  mpu.update();
  cTheta = mpu.getAngleZ()- thetaOffset;
  mySDR.updateLocation(lCounter, rCounter, cTheta);
  Serial.print(cTheta);
  Serial.print(", path ");
  Serial.print(mySDR.getXLocation());
  Serial.print(" ");
  Serial.println(mySDR.getYLocation());
//  delay(100);
  
  //motor checking
//  analogWrite(PIN_MOTOR_RIGHT_ENA, 255);   digitalWrite( PIN_MOTOR_RIGHT_IN1, HIGH);    digitalWrite( PIN_MOTOR_RIGHT_IN2 , LOW);
//  analogWrite(PIN_MOTOR_LEFT_ENB, 255);   digitalWrite( PIN_MOTOR_LEFT_IN3, HIGH);    digitalWrite( PIN_MOTOR_LEFT_IN4 , LOW);
//
//  //Servo checking
//  myServo.write(100);


//  //IR Line sensor checking
//  Serial.print( digitalRead(PIN_IR_LEFT)); Serial.print(" "); Serial.print(digitalRead(PIN_IR_RIGHT)); Serial.print("][");
//  //Encoder checking
//  Serial.print( lCounter); Serial.print(" = ");
//  Serial.print( digitalRead(PIN_ENCODER_LEFT_CLK)); Serial.print(" "); Serial.print(digitalRead(PIN_ENCODER_LEFT_DT));Serial.print("][");
//  Serial.print( rCounter); Serial.print(" = ");
//  Serial.print( digitalRead(PIN_ENCODER_RIGHT_CLK)); Serial.print(" "); Serial.print(digitalRead(PIN_ENCODER_RIGHT_DT));Serial.print("][");


//  //Sonar Checking
//  digitalWrite(PIN_SONAR_PING, LOW);
//  delayMicroseconds(2);
//  digitalWrite(PIN_SONAR_PING, HIGH);
//  delayMicroseconds(5);
//  digitalWrite(PIN_SONAR_PING, LOW);
//  Serial.print(pulseIn(PIN_SONAR_ECHO, HIGH,5000 ) / 29 / 2 ); Serial.print("]");
// 
 
//  //IMU checking
//  Serial.print("[");
//  mpu.Execute();
//  Serial.print(mpu.GetAngX());
//  Serial.print("  ");
//  Serial.print(mpu.GetAngY());
//  Serial.print("  ");
//  Serial.print(mpu.GetAngZ());
//  Serial.println("]");


//  //Bluetooth Serial checking
//  while (Serial.available() > 0){
//    char a = Serial.read();
//    Serial3.write((char)a);  
//  }
//  while (Serial3.available() > 0){
//    char b = Serial3.read();
//    Serial.write((char)b);  
//  }
//  
//  delay(30);
//  
    // String odom_temp[8] = {String(odom_pose[0]), String(odom_pose[1]), String(odom_pose[2]), String(odom_vel[0]), String(odom_vel[2])};
    // send_py(odom_temp, 5, ODOM_MSG);  
}





void runLineFollow(){
  if ((irValue1 > 100) && (irValue2 > 100)) { // both are in a black line
    driver.motorAForward(100); driver.motorBForward(100);
  }else if ((irValue1 > 100) && (irValue2 <= 100)) { // left is in a block line
    driver.motorAReverse(100); driver.motorBForward(100);
  }else if ((irValue1 <= 100) && (irValue2 > 100)) { // left is in a block line
    driver.motorAForward(100); driver.motorBReverse(100);
  }else{                                             // both are out of a black line 
    driver.motorAStop(); driver.motorBStop();
  }
}




void initMotor(){
  // Attach a motor to the input pins:
  driver.attachMotorA(RightMotor_INT1, RightMotor_INT2);
  driver.attachMotorB(LeftMotor_INT1, LeftMotor_INT2);
}

void initSensors(){
  //IMU test
  Wire.begin();
  
  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while(status!=0){ } // stop everything if could not connect to MPU6050
  
  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets(); // gyro and accelero
  Serial.println("Done!\n");


}

void initDisplay(){
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  displayMessage("TEST CODE"); 
}

void initComm(){
  Serial.begin(115200);
  while (!Serial) delay(10); // will pause Zero, Leonardo, etc until serial console opens
  Serial.println("Serial port inititialized");

   WiFi.begin(STASSID, STAPSK);
   while (WiFi.status() != WL_CONNECTED) {
      Serial.print('.');
      delay(500);
   }
   Serial.print("Connected! IP address: ");
   Serial.println(WiFi.localIP());
  // Serial.print("UDP server on port %d\n", localPort);
   Udp.begin(localPort);
}


void initLocalization(){
  thetaOffset= mpu.getAngleZ();

  mySDR.setXLocation(500.0);
  mySDR.setYLocation(500.0);  
}

void initPins(){
  //encoder pins
  pinMode(RightMotor_E1, INPUT_PULLUP); pinMode(RightMotor_E2, INPUT_PULLUP); 
  pinMode(LeftMotor_E1, INPUT_PULLUP); pinMode(LeftMotor_E2, INPUT_PULLUP);   
  // pinMode(RightMotor_E1, INPUT); pinMode(RightMotor_E2, INPUT); 
  // pinMode(LeftMotor_E1, INPUT); pinMode(LeftMotor_E2, INPUT);   
  // Initialize motor control pins as outputs
  pinMode(RightMotor_INT1, OUTPUT);  pinMode(RightMotor_INT2, OUTPUT);
  pinMode(LeftMotor_INT1, OUTPUT);  pinMode(LeftMotor_INT2, OUTPUT);
  // Initialize LED pins as outputs
  pinMode(Led_1, OUTPUT);  pinMode(Led_2, OUTPUT);  pinMode(Led_3, OUTPUT);  
  // Initialize IR sensor pins as inputs
  pinMode(IRSensor_1, INPUT);   pinMode(IRSensor_2, INPUT);
}

void displayMessage(const char *message) {
  u8g2.clearBuffer();  u8g2.drawStr(0, 20, message);
  u8g2.sendBuffer();
}

void displayIRValues(int value1, int value2) {
  u8g2.clearBuffer(); u8g2.setCursor(0, 10); u8g2.print("IR Sensor 1: "); u8g2.print(value1);
  u8g2.setCursor(0, 20); u8g2.print("IR Sensor 2: "); u8g2.print(value2); 
  u8g2.setCursor(0, 30); u8g2.print("Motor = "); u8g2.print(cnt); 
  u8g2.setCursor(0, 40); u8g2.print("IP = ");  //u8g2.print(WiFi.localIP());
  u8g2.setCursor(0, 50); u8g2.print("Odom_R="); 
  u8g2.sendBuffer();
}

void displayOdom() {
  u8g2.clearBuffer(); 
  u8g2.setCursor(0, 10); u8g2.print(irValue1);u8g2.print(" : ");u8g2.print(irValue2);
  u8g2.setCursor(0, 20); u8g2.print(rCounter);u8g2.print(" : ");u8g2.print(lCounter);
  u8g2.setCursor(0, 30); u8g2.print(enError_R);u8g2.print(" : ");u8g2.print(enError_L);
  u8g2.setCursor(0, 40); u8g2.print(cTheta);u8g2.print(" = "); u8g2.print(mySDR.getXLocation());u8g2.print(" : ");u8g2.print(mySDR.getYLocation());
  u8g2.sendBuffer();
}