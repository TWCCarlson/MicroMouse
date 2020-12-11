//LEFT STEPPER
#define STEP_PINL 32
#define DIR_PINL 14
#define MS1L 13

//RIGHT STEPPER
#define STEP_PINR 33
#define DIR_PINR 15
#define MS1R A1

// IR SENSORS
#define SenL A4 // Profile iii
#define SenR A2 // Profile ii
#define SenC A3 // Profile i

int stepsStrt = 420;
// 420 steps = 1 cell
int stepsSpin = 172;
// 344 steps = 180 deg
// 172 steps = 90 deg
// 86 steps = 45 deg

#include <ArduinoSort.h>

void setup() {
  //Setup
  pinMode(MS1L, OUTPUT);
  pinMode(MS1R, OUTPUT);
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS1R, HIGH);
  pinMode(DIR_PINL, OUTPUT);
  pinMode(STEP_PINL, OUTPUT);
  pinMode(DIR_PINR, OUTPUT);
  pinMode(STEP_PINR, OUTPUT);
  Serial.begin(9600);
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  Fwd(3,1);
}

void Fwd(int Cells, bool Dir){
  // Move linearly
  if (Dir == 1){
    // Forward
    digitalWrite(DIR_PINL, HIGH);
    digitalWrite(DIR_PINR, HIGH);
  } else {
    // Backward
    digitalWrite(DIR_PINL, LOW);
    digitalWrite(DIR_PINR, LOW);
  }
  for (int i = 1; i <= stepsStrt*Cells; i++){
    digitalWrite(STEP_PINR, HIGH);
    digitalWrite(STEP_PINL, HIGH);
    delay(2);
    digitalWrite(STEP_PINR, LOW);
    digitalWrite(STEP_PINL, LOW);
    delay(2);
  }
  delay(200);
}

void Rot(int Rotations, bool Dir){
  // Spin in place
  if (Dir == 1){
    //CW Spin
    digitalWrite(DIR_PINL, HIGH);
    digitalWrite(DIR_PINR, LOW);
  } else {
    //CCW Spin
    digitalWrite(DIR_PINL, LOW);
    digitalWrite(DIR_PINR, HIGH);
  }
  for (int i = 1; i <= stepsSpin*Rotations; i++){
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
  }
  delay(200);
}
