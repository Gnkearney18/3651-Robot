#include <Arduino.h>
#include <Wire.h>
#include <vl53l4cd_class.h>
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>

#include <Servo.h>

#define DEV_I2C Wire
#define SerialPort Serial

#define LedPin LED_BUILTIN

VL53L4CD sensor_vl53l4cd_sat(&DEV_I2C, A1);

#include "MPU9250.h"
MPU9250 IMU(Wire, 0x68);

Servo servo;
int status;

int bLouta = 22;
int bLoutb = 23;
int pwmbL = 8;

int bRouta = 25;
int bRoutb = 24;
int pwmbR = 7;

int fLouta = 28;
int fLoutb = 29;
int pwmfL = 9;


int fRouta = 26;
int fRoutb = 27;
int pwmfR = 6;

int servoPin = 3;
int button = 40;

VL53L4CD_Result_t results;

double last[] = {0,0,0,0};
double current[] = {0,0,0,0};
long startTime[] = {0,0,0,0}; 
long stopTime[] = {0,0,0,0};
double distance[] = {0,0,0,0};
int prevState[] = {0,0,0,0};
int currentState[] = {0,0,0,0};
long period[] = {0,0,0,0};

double speedMeasured[] = {0,0,0,0}; //in rotations per second

double wheelCircumference = 3.14159 *2.5; //in inches

double distanceTraveled[] = {0,0,0,0};



void setup() {
  Serial.begin(115200);
  
  setPinModes();
  setupSensor();
  
  servo.attach(servoPin);
  servo.write(90);
  updateSensors();

  Serial.println("Press Button to Begin");
  while(digitalRead(button) == 0){} 
}

void loop() {  
//
//  forwardAidedGyro();
//
////  forward(255);
//  updateSpeeds();
  go();

  delay(30);
//
  servo.write(90);

  turnAround();

  servo.write(180);

  delay(30);
  
  back();
  while(1){}

  
}

int turns[4];
int distances[4];


void go() {

  for (int i = 0; i < 2; i++) {
    
    Serial.println("Starting");
    servo.write(90);
    
    updateSensors();
  
    int distanceAway = results.distance_mm;
    forward(255);
    updateSpeeds();

    zeroDistance();
    int offsetSensors = 0;
    while (distanceAway > 300 || distanceAway == 0){
      
     forwardAidedGyro();
     updateSpeeds();
          
      offsetSensors = offsetSensors + 1;
      
      if (offsetSensors > 100) {
        updateSensors();
        distanceAway = results.distance_mm;
        offsetSensors = 0;
      }
    }
  
    servo.write(180);
    forward(0);
    delay(150);
    
    distances[i] = distanceTraveled[i];
    Serial.println(distanceTraveled[i]);
    
    if (i != 1) {
      Serial.println("checking left");
      delay(300);
      updateSensors();
      delay(100);
      int left = results.distance_mm;
    
    
    
      servo.write(0);
      Serial.println("checking right");
      delay(150);
      updateSensors();
      delay(100);
      
      int right = results.distance_mm;
    
      if (left > right) {
        turnLeft(90);
        turns[i] = -1;
      } else {
        turnRight(90);
        turns[i] = 1;
      }
    }
  }

}

void zeroDistance() {
    for (int i = 0; i < 4; i++) {
    distanceTraveled[i] = 0;
  }
}

void turnAround() {
  IMU.begin();
  turnRight(90);
  IMU.begin();
  delay(3);
  turnRight(90);
}


void back() {

  for (int i = 1; i > -1; i--){
    if (i != 1) { 
      if (turns[i] ==     -1) {
        turnRight(90);
      } else {
        turnLeft(90);
      }
    }
  
    zeroDistance();
    while (distanceTraveled[i] < distances[i]) {
      forwardAidedGyro(); 
      updateSpeeds();
    }
  
    forward(0);
  }
}


int zeroGyro = 0;
float currentAngleG = 0;
int wheelSpeedG = 255;
int previousTimeG = 0;
int currentTimeG = 0;

void forwardAidedGyro() {
  
//  forward(255);

  IMU.readSensor();
  currentTimeG = millis();

  int delta = currentTimeG - previousTimeG;
  float deltaAngle = delta * IMU.getGyroZ_rads()*.001;

  currentAngleG = currentAngleG + ((deltaAngle * 360.0)/(2*3.14159));
//  Serial.print("Current Angle: ");
//  Serial.println(currentAngleG);
  previousTimeG = currentTimeG;
//
//  Serial.print("ZeroGyro: ");
//  Serial.println(zeroGyro);
  zeroGyro = zeroGyro + 1;
  if (zeroGyro > 50) {
    zeroGyro = 0;
    currentAngleG = 0;
    
  }

//  Serial.println(currentAngleG);
  setLeftDirection(1);
  setRightDirection(1);
  directionCorrection();
}

int factor = 5;
void directionCorrection() {
  int right;
  if (wheelSpeedG + factor*currentAngleG > 255) {
    right = 255;
  } else {
    right = wheelSpeedG + factor*currentAngleG;
  }
  int left;
  if (wheelSpeedG - factor*currentAngleG > 255) {
    left = 255;
  } else {
    left = wheelSpeedG - factor*currentAngleG;
  }
//  Serial.print("Left: ");
//  Serial.print(left);
//  Serial.print(" Right: ");
//  Serial.println(right);
//  
  
  analogWrite(pwmfR, right);
  analogWrite(pwmfL, left);
  analogWrite(pwmbR, right);
  analogWrite(pwmbL, left);
  updateSpeeds();
}
  


void setPinModes() {
  pinMode(button, INPUT);
  
  pinMode(bLouta, OUTPUT);
  pinMode(bLoutb, OUTPUT);

  pinMode(bRouta, OUTPUT);
  pinMode(bRoutb, OUTPUT);

  pinMode(fLouta, OUTPUT);
  pinMode(fLoutb, OUTPUT);

  pinMode(fRouta, OUTPUT);
  pinMode(fRoutb, OUTPUT);
}

void forward(int speedWheel) {
  setSpeeds(speedWheel);
  
  setLeftDirection(1);
  setRightDirection(1);
}



void backward(int speedWheel) {
  setSpeeds(speedWheel);
  
  setLeftDirection(-1);
  setRightDirection(-1);
}

void setSpeeds(int speedWheel) {
  analogWrite(pwmfR, speedWheel);
  analogWrite(pwmfL, speedWheel);
  analogWrite(pwmbR, speedWheel);
  analogWrite(pwmbL, speedWheel);
}


void rightTurn(int speedWheel) {


  setLeftDirection(1);
  setRightDirection(-1);

  setSpeeds(speedWheel);
}


void leftTurn(int speedWheel) {
  setSpeeds(speedWheel);

  setLeftDirection(-1);
  setRightDirection(1);
}

void setLeftDirection(int directionInt) {
  if (directionInt == 1) {
    digitalWrite(fLouta, HIGH);
    digitalWrite(fLoutb, LOW);

    digitalWrite(bLouta, HIGH);
    digitalWrite(bLoutb, LOW);
  } else {
    digitalWrite(fLouta, LOW);
    digitalWrite(fLoutb, HIGH);

    digitalWrite(bLouta, LOW);
    digitalWrite(bLoutb, HIGH);
  }
}

void setRightDirection(int directionInt) {
  if (directionInt == 1) {
    digitalWrite(fRouta, HIGH);
    digitalWrite(fRoutb, LOW);

    digitalWrite(bRouta, HIGH);
    digitalWrite(bRoutb, LOW);
  } else {
    digitalWrite(fRouta, LOW);
    digitalWrite(fRoutb, HIGH);

    digitalWrite(bRouta, LOW);
    digitalWrite(bRoutb, HIGH);
  }
}


void setupSensor() {
  SerialPort.println("Starting...");

  // Initialize I2C bus.
  DEV_I2C.begin();

  status = IMU.begin();
  if (status < 0) {
    Serial.println("IMU initialization unsuccessful");
    Serial.println("Check IMU wiring or try cycling power");
    Serial.print("Status: ");
    Serial.println(status);
    while(1) {}
  }

  // Configure VL53L4CD satellite component.
  sensor_vl53l4cd_sat.begin();

  // Switch off VL53L4CD satellite component.
  sensor_vl53l4cd_sat.VL53L4CD_Off();

  //Initialize VL53L4CD satellite component.
  sensor_vl53l4cd_sat.InitSensor();

  // Program the highest possible TimingBudget, without enabling the
  // low power mode. This should give the best accuracy
  sensor_vl53l4cd_sat.VL53L4CD_SetRangeTiming(200, 0);

  // Start Measurements
  sensor_vl53l4cd_sat.VL53L4CD_StartRanging();
}

void updateSensors(){
  IMU.readSensor();
  
  int statusUpdate = updateSensor();
  while(statusUpdate == -1) {
    statusUpdate = updateSensor();
    delay(30);
  }
}
int updateSensor() {
  
  uint8_t NewDataReady = 0;
  uint8_t status;
  char report[64];

  int i = 0;
  do {
    status = sensor_vl53l4cd_sat.VL53L4CD_CheckForDataReady(&NewDataReady);
    updateSpeeds();
    i = i + 1;
    if (i > 10) {
      return -1;
    }
  } while (!NewDataReady);

  if ((!status) && (NewDataReady != 0)) {
    // (Mandatory) Clear HW interrupt to restart measurements
    sensor_vl53l4cd_sat.VL53L4CD_ClearInterrupt();

    // Read measured distance. RangeStatus = 0 means valid data
    sensor_vl53l4cd_sat.VL53L4CD_GetResult(&results);
  }
//  Serial.println(results.distance_mm);
  return 1;
} 


int fLa = 36;
int fLb = 37;

int fRa = 34;
int fRb = 35;

int bLa = 32;
int bLb = 33;

int bRa = 30;
int bRb = 31;

void updateSpeeds(){

  double fL = (digitalRead(fLa));// + digitalRead(fLb))/2;
  double fR = (digitalRead(fRa));// + digitalRead(fRb))/2;
  double bL =  (digitalRead(bLa));// + digitalRead(bLb))/2;
  double bR = (digitalRead(bRa));// + digitalRead(bRb))/2;
  
  double current[] = {fL, fR, bL, bR};
  for (int i = 0; i < 4; i++){
    currentState[i] = current[i] -last[i];

    
    if (prevState[i] <= 0) {
      if(currentState[i] > 0) {
        stopTime[i] = micros();
        period[i] = stopTime[i] - startTime[i];
        startTime[i] = micros();
          speedMeasured[i] = (6000.0/period[i]); //Rotations Per Second        
        
        distanceTraveled[i] = distanceTraveled[i] + (wheelCircumference * period[i] * 0.000001 * speedMeasured[i]);
      }
    }
    prevState[i] = currentState[i];
    last[i] = current[i];
  }
//  Serial.println();

  
  Serial.println(distanceTraveled[0]);
  
}



void turnLeft(int goal) {
  IMU.begin();
  int wheelSpeed = 255;
  int previousTime = 0;
  int currentTime = 0;
  float currentAngle = 360;
  goal = 360-goal;
  while(1) {
    IMU.readSensor();
    currentTime = millis();
  
    int delta = currentTime - previousTime;
    float deltaAngle = delta * IMU.getGyroZ_rads()*.001;
  
    currentAngle = currentAngle + ((deltaAngle * 360.0)/(2*3.14159));
    Serial.println(currentAngle);
    previousTime = currentTime;

    double diff = abs(currentAngle - goal);
    wheelSpeed = ((abs(currentAngle - goal))/goal)* 255;
    if (diff < 15) {
      wheelSpeed = 0;
      setSpeeds(wheelSpeed);
      
      return;
    } else {
      wheelSpeed = wheelSpeed + 245;
    }
    
    if (wheelSpeed > 255){
      wheelSpeed = 255;
    } 
    
    leftTurn(wheelSpeed);
  }
}

void turnRight(int goal) {
  IMU.begin();
  int wheelSpeed = 255;
  int previousTime = 0;
  int currentTime = 0;
  float currentAngle = 0;
  
  while(1) {
    IMU.readSensor();
    currentTime = millis();
  
    int delta = currentTime - previousTime;
    float deltaAngle = delta * IMU.getGyroZ_rads()*.001;
  
    currentAngle = currentAngle + ((deltaAngle * 360.0)/(2*3.14159));
    Serial.println(currentAngle);
    previousTime = currentTime;
  
    wheelSpeed = ((abs(currentAngle - goal))/goal)* 255;
    if (wheelSpeed < 15) {
      wheelSpeed = 0;
      setSpeeds(wheelSpeed);
      IMU.begin();
      return;
    } else {
      wheelSpeed = wheelSpeed + 250;
    }
    
    if (wheelSpeed > 255){
      wheelSpeed = 255;
    } 
    
    rightTurn(wheelSpeed);
    
  }
  
}
