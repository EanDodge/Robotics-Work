/**********************************************************************;
* Project           : [Intro-Robotics] ATR TurtleBot Nano 
* Program name      : ATR_TurtleBot_Nano_One_Wheel_Speed.ino
* Author            : Jong-Hoon Kim
* Date created      : 04/25/2024
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



#define ATR_PROTOCOL_MAX_BUF_SIZE 64
#define TICK_PER_DISTANCE .013  //2724
#ifndef STASSID
#define STASSID "RRL_Kent"
//#define STASSID "Ean's iPhone"
#define STAPSK "ksu_robotics"
#endif
unsigned int localPort = 8888;  // local port to listen on

// buffers for receiving and sending data
char packetBuffer[UDP_TX_PACKET_MAX_SIZE + 1];  // buffer to hold incoming packet,
char ReplyBuffer[] = "acknowledged\r\n";        // a string to send back

// WIFI setup
WiFiUDP Udp;
char msgBuf[ATR_PROTOCOL_MAX_BUF_SIZE];
int msgBufPnt = 0;
SimpleDeadReckoning mySDR(2700, 1, 6, 0);  // encoder values per one rotation,  wheel radius, distance between two wheels, unit (cm)
MPU6050 mpu(Wire);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R2, /* reset=*/U8X8_PIN_NONE);


// control variable
int controlMode = 0;                                 // manual control = 0, line following = 1, stright motion test = 2
int targetLinearSpeed = 45, targetAngularSpeed = 0;  //

int leftMotorPower, RightMotorPower = 0;

//robot status
float pose[3] = { 0.0, 0.0, 0.0 };  // x , y, theta
//float cmd_vel[3] = {0.0,0.0,0.0};  // linear, angular,
int currentLinearSpeed, currentAngularSpeed = 0;

// control inputs
int joystick[3] = { 0, 0, 0 };  // angular(x), linear (y), sw value

// event time variable
long cTime = 0;              // current time of each iteration
long mTime, mTime_Prev = 0;  // motor interval
long sTime, sTime_Prev = 0;  // speed check
long oTime = 0;              // OLED display Time
long dTime = 0;              // Seiral Debug time
unsigned long timer = 0;
double distanceCovered = 0;
int xStartPos = 500;
int yStartPos = 500;


// odom inputs
int prevLeftCLK, prevLeftDT, nowLeftCLK, nowLeftDT = 0;
int prevRightCLK, prevRightDT, nowRightCLK, nowRightDT = 0;
// odometry
long lCounter, rCounter, lCounter_Prev, rCounter_Prev = 0;
// odometry error count
long enError_R = 0;
long enError_L = 0;
// wheel speed and motor power
int lSpeed, rSpeed, lPower = 200, rPower = 200;
float cTheta, xLocation, yLocation = 0;

float thetaOffset = 0.0;

// Create an instance of the DRV8833:
DRV8833 driver = DRV8833();

int flag = 1;
int corner = 0;
int angle = 90;
void setup() {
  // put your setup code here, to run once:
  initOdom();
  initPins();
  initComm();
  initSensors();
  initDisplay();
  initMotor();
  initLocalization();
  delay(2000);
}

void loop() {
  // put your main code here, to run repeatedly:
  cTime = micros();  // current time of this iteration

  checkInputs();
  checkSpeed();
  runLocalization();
  distanceCovered = sqrt(pow((xStartPos - mySDR.getXLocation()), 2) + pow(yStartPos - mySDR.getYLocation(), 2));
  if (flag == 1) {
    updateMotor();
  }
  if (distanceCovered >= 40) {
    flag = -1;
    xStartPos = mySDR.getXLocation();
    yStartPos = mySDR.getYLocation();
  }
  if (flag == -1 && corner < 4) {
    updateMotorRight();
    if (cTheta <= thetaOffset - angle) {
      ++corner;
      flag *= -1;
      angle += 90;
       //thetaOffset += 90;
    }
  }
  if (corner == 4){
    targetLinearSpeed = 0;
  }

//updateLocalization();
//updateMotor();
//updateDisplay();
displayOdom();
}


void initPins() {
  //encoder pins
  pinMode(RightMotor_E1, INPUT_PULLUP);
  pinMode(RightMotor_E2, INPUT_PULLUP);
  pinMode(LeftMotor_E1, INPUT_PULLUP);
  pinMode(LeftMotor_E2, INPUT_PULLUP);
  // Initialize motor control pins as outputs
  pinMode(RightMotor_INT1, OUTPUT);
  pinMode(RightMotor_INT2, OUTPUT);
  pinMode(LeftMotor_INT1, OUTPUT);
  pinMode(LeftMotor_INT2, OUTPUT);
  // Initialize LED pins as outputs
  pinMode(Led_1, OUTPUT);
  pinMode(Led_2, OUTPUT);
  pinMode(Led_3, OUTPUT);
  // Initialize IR sensor pins as inputs
  pinMode(IRSensor_1, INPUT);
  pinMode(IRSensor_2, INPUT);
}

void initOdom() {
  // adding interrupt rutines on motor encoder pins
  attachInterrupt(digitalPinToInterrupt(LeftMotor_E1), checkLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(LeftMotor_E2), checkLeftEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightMotor_E1), checkRightEncoder, CHANGE);
  attachInterrupt(digitalPinToInterrupt(RightMotor_E2), checkRightEncoder, CHANGE);
}
void initLocalization() {
  thetaOffset = mpu.getAngleZ();

  mySDR.setXLocation(500.0);
  mySDR.setYLocation(500.0);
}
void runLocalization() {
  //cTime = millis ();
  //  recieved_py();

  mpu.update();
  cTheta = mpu.getAngleZ() - thetaOffset;
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
void initSensors() {
  //IMU test
  Wire.begin();

  byte status = mpu.begin();
  Serial.print(F("MPU6050 status: "));
  Serial.println(status);
  while (status != 0) {}  // stop everything if could not connect to MPU6050

  Serial.println(F("Calculating offsets, do not move MPU6050"));
  delay(1000);
  // mpu.upsideDownMounting = true; // uncomment this line if the MPU6050 is mounted upside-down
  mpu.calcOffsets();  // gyro and accelero
  Serial.println("Done!\n");
}

void initMotor() {
  // Attach a motor to the input pins:
  driver.attachMotorB(LeftMotor_INT1, LeftMotor_INT2);
  driver.attachMotorA(RightMotor_INT1, RightMotor_INT2);
}

void checkSpeed() {
  if (cTime > sTime) {
    lSpeed = ((abs(lCounter) - abs(lCounter_Prev)) * TICK_PER_DISTANCE) * (1000000 / (cTime - sTime_Prev));
    lCounter_Prev = lCounter;
    rSpeed = ((abs(rCounter) - abs(rCounter_Prev)) * TICK_PER_DISTANCE) * (1000000 / (cTime - sTime_Prev));
    rCounter_Prev = rCounter;
    sTime_Prev = cTime;
    sTime = cTime + 100000;
  }
}

// 0:0 ==> // 0:1 ==> // 1:1 ==> // 1:0
void checkLeftEncoder() {
  prevLeftCLK = nowLeftCLK;
  prevLeftDT = nowLeftDT;
  nowLeftCLK = digitalRead(LeftMotor_E1);
  nowLeftDT = digitalRead(LeftMotor_E2);
  if ((prevLeftCLK == 0) && (prevLeftDT == 0)) {
    if ((nowLeftCLK == 0) && (nowLeftDT == 1)) {
      lCounter--;  //lCounter++;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 0)) {
      lCounter++;  //lCounter--;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 1)) {
      enError_L++;
    }
  } else if ((prevLeftCLK == 0) && (prevLeftDT == 1)) {
    if ((nowLeftCLK == 0) && (nowLeftDT == 0)) {
      lCounter++;  //lCounter--;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 1)) {
      lCounter--;  //lCounter++;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 1)) {
      enError_L++;
    }
  } else if ((prevLeftCLK == 1) && (prevLeftDT == 0)) {
    if ((nowLeftCLK == 0) && (nowLeftDT == 0)) {
      lCounter--;  //lCounter++;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 1)) {
      lCounter++;  //lCounter--;
    } else if ((nowLeftCLK == 0) && (nowLeftDT == 1)) {
      enError_L++;
    }
  } else if ((prevLeftCLK == 1) && (prevLeftDT == 1)) {
    if ((nowLeftCLK == 0) && (nowLeftDT == 1)) {
      lCounter++;  //lCounter--;
    } else if ((nowLeftCLK == 1) && (nowLeftDT == 0)) {
      lCounter--;  //lCounter++;
    } else if ((nowLeftCLK == 0) && (nowLeftDT == 0)) {
      enError_L++;
    }
  }
}
void checkRightEncoder() {
  prevRightCLK = nowRightCLK;
  prevRightDT = nowRightDT;
  nowRightCLK = digitalRead(RightMotor_E1);
  nowRightDT = digitalRead(RightMotor_E2);
  if ((prevRightCLK == 0) && (prevRightDT == 0)) {
    if ((nowRightCLK == 0) && (nowRightDT == 1)) {
      rCounter++;  //lCounter++;
    } else if ((nowRightCLK == 1) && (nowRightDT == 0)) {
      rCounter--;  //lCounter--;
    } else if ((nowRightCLK == 1) && (nowRightDT == 1)) {
      enError_R++;
    }
  } else if ((prevRightCLK == 0) && (prevRightDT == 1)) {
    if ((nowRightCLK == 0) && (nowRightDT == 0)) {
      rCounter--;  //lCounter--;
    } else if ((nowRightCLK == 1) && (nowRightDT == 1)) {
      rCounter++;  //lCounter++;
    } else if ((nowRightCLK == 1) && (nowRightDT == 1)) {
      enError_R++;
    }
  } else if ((prevRightCLK == 1) && (prevRightDT == 0)) {
    if ((nowRightCLK == 0) && (nowRightDT == 0)) {
      rCounter++;  //lCounter++;
    } else if ((nowRightCLK == 1) && (nowRightDT == 1)) {
      rCounter--;  //lCounter--;
    } else if ((nowRightCLK == 0) && (nowRightDT == 1)) {
      enError_R++;
    }
  } else if ((prevRightCLK == 1) && (prevRightDT == 1)) {
    if ((nowRightCLK == 0) && (nowRightDT == 1)) {
      rCounter--;  //lCounter--;
    } else if ((nowRightCLK == 1) && (nowRightDT == 0)) {
      rCounter++;  //lCounter++;
    } else if ((nowRightCLK == 0) && (nowRightDT == 0)) {
      enError_R++;
    }
  }
}

void updateMotor() {

  if (cTime > mTime) {
    if (targetLinearSpeed > lSpeed) {
      lPower += 1;
    } else if (targetLinearSpeed < lSpeed) {
      lPower -= 1;
    }

    if (targetLinearSpeed > rSpeed) {
      rPower += 1;
    } else if (targetLinearSpeed < rSpeed) {
      rPower -= 1;
    }
    if (targetLinearSpeed == 0) {
      lPower = 0;
      rPower = 0;
    }
    driver.motorBForward(lPower);
    driver.motorAForward(rPower);
    mTime = cTime + 100000;
  }
}

void updateMotorRight() {

  if (cTime > mTime) {
    if (targetLinearSpeed > lSpeed) {
      lPower += 1;
    } else if (targetLinearSpeed < lSpeed) {
      lPower -= 1;
    }

    if (targetLinearSpeed > rSpeed) {
      rPower += 1;
    } else if (targetLinearSpeed < rSpeed) {
      rPower -= 1;
    }
    if (targetLinearSpeed == 0) {
      lPower = 0;
      rPower = 0;
    }
    driver.motorBForward(lPower);
    driver.motorAReverse(rPower);
    mTime = cTime + 100000;
  }
}


void initDisplay() {
  u8g2.begin();
  u8g2.setFont(u8g2_font_ncenB08_tr);
  displayMessage("TEST CODE");
}
void initComm() {
  Serial.begin(115200);
  while (!Serial) delay(10);  // will pause Zero, Leonardo, etc until serial console opens
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

void checkInputs() {
  checkSerialInput();
  //  checkUDPInput();
  //  checkButtonInput();
}

void checkSerialInput() {
  // serial message input checking
  while (Serial.available() > 0) {
    char tmpChar = Serial.read();
    if (msgBufPnt >= ATR_PROTOCOL_MAX_BUF_SIZE) {
      Serial.println("Message Overflow");
      if ((tmpChar != '\n') || (tmpChar != '\r')) {
        msgBuf[0] = tmpChar;
        msgBufPnt = 1;
      }
    } else {
      if ((tmpChar == '\n') || (tmpChar == '\r')) {
        msgBuf[msgBufPnt] = '\0';
        checkMessage();
        msgBufPnt = 0;
      } else {
        msgBuf[msgBufPnt] = tmpChar;
        msgBufPnt++;
      }
    }
  }
}

bool checkMessage() {  // Serial message checking
                       //  Serial.println(msgBuf);
  char *p = msgBuf;
  String str;
  int cnt = 0;
  // while ((str = strtok_r(p, ";", &p)) != NULL) // delimiter is the semicolon
  str = strtok_r(p, ";", &p);  // delimiter is the semicolon
  Serial.println(str);
  if (str == "Keyboard") {
    Serial.print("Key ");
    while ((str = strtok_r(p, ";", &p)) != NULL) {
      if (cnt == 0) {  //joy x value
        if (str == "w") {
          targetLinearSpeed += 1;
        } else if (str == "a") {
          targetAngularSpeed -= 1;
        } else if (str == "s") {
          targetLinearSpeed += 1;
        } else if (str == "d") {
          targetAngularSpeed -= 1;
        } else if (str == "0") {
          controlMode = 0;
        } else if (str == "1") {
          controlMode = 1;
        } else if (str == "2") {
          controlMode = 2;
        } else if (str == " ") {
          targetAngularSpeed = 0;
          targetLinearSpeed = 0;
        }
        Serial.println(str);
      }
      cnt++;
    }
  } else if (str == "Joystick") {
    Serial.println("Joy ");
    while ((str = strtok_r(p, ";", &p)) != NULL) {
      if (cnt == 0) {  //joy x value
        joystick[0] = str.toInt();
      } else if (cnt == 1) {  //joy y value
        joystick[1] = str.toInt();
      } else if (cnt == 2) {  //joy sw value
        joystick[2] = str.toInt();
      }
      cnt++;
    }
    Serial.print(joystick[0]);
    Serial.print(" ");
    Serial.print(joystick[1]);
    Serial.print(" ");
    Serial.println(joystick[2]);
  } else if (str == "Status") {
    Serial.print("Control Mode: ");
    Serial.print(controlMode);
    Serial.print(", targetAngularSpeed: ");
    Serial.print(targetAngularSpeed);
    Serial.print(", targetLinearSpeed:");
    Serial.print(targetLinearSpeed);
    Serial.print(", X:");
    Serial.print(pose[0]);
    Serial.print(", Y:");
    Serial.print(pose[1]);
    Serial.print(", Orientation:");
    Serial.println(pose[3]);
  } else {
    while ((str = strtok_r(p, ";", &p)) != NULL) {
      Serial.print("No Standard Protocol[");
      Serial.print(str);
      Serial.println("]");
    }
  }

  return 0;
}






void updateLocalization() {
}


void updateDisplay() {
  // debugging purpose ==> send a message to Serial

  if (cTime > dTime) {
    Serial.print("Target Speed:");
    Serial.print(targetLinearSpeed);
    Serial.print(", Current Left Motor Speed:");
    Serial.print(lSpeed);
    Serial.print(", Current right Motor Speed:");
    Serial.print(rSpeed);
    Serial.print(",Current Left Motor Power:");
    Serial.print(lPower);
    Serial.print(",Current Right Motor Power:");
    Serial.println(rPower);
    dTime = cTime + 100000;
  }
  // to OLED
}
void displayMessage(const char *message) {
  u8g2.clearBuffer();
  u8g2.drawStr(0, 20, message);
  u8g2.sendBuffer();
}
void displayOdom() {
  u8g2.clearBuffer();
  //u8g2.setCursor(0, 10); u8g2.print(irValue1);u8g2.print(" : ");u8g2.print(irValue2);
  u8g2.setCursor(0, 20);
  u8g2.print(rCounter);
  u8g2.print(" : ");
  u8g2.print(lCounter);
  u8g2.setCursor(0, 30);
  u8g2.print(enError_R);
  u8g2.print(" : ");
  u8g2.print(enError_L);
  u8g2.setCursor(0, 40);
  u8g2.print(cTheta);
  u8g2.print(" = ");
  u8g2.print(mySDR.getXLocation());
  u8g2.print(" : ");
  u8g2.print(mySDR.getYLocation());
  u8g2.sendBuffer();
}
