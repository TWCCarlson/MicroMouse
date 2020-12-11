//LEFT STEPPER
#define STEP_PINL A5
#define DIR_PINL 14
#define MS1L 13
#define MS2L 12

//RIGHT STEPPER
#define STEP_PINR 33
#define DIR_PINR 15
#define MS1R A1
#define MS2R 27

// IR SENSORS
#define SenL 32 // Profile iii
#define SenR A2 // Profile ii
#define SenC A3 // Profile i

// IR OUTPUTS
float AnalogIR_FWD;
float AnalogIR_RT;
float AnalogIR_LT;
float volts_FWD;
float volts_RT;
float volts_LT;
float voltsAvg_FWD;
float voltsAvg_RT;
float voltsAvg_LT;
//float array[5];
//#include <ArduinoSort.h>
//float number = array[0];
//float Mode = number;
//int count = 1;
//int countMode = 1;


// Motion constants
int stepsStrt = 170;
// 420 steps = 1 cell
int stepsSpin = 172;
// 344 steps = 180 deg
// 172 steps = 90 deg
// 86 steps = 45 deg

// Timing
int tstart;
int tend;
int tspan;

// Control Vars
float FWD_PADDING = 1.5; // cm, stopping space for forward sensor
float LT_PADDING = 3.2; // cm, minimum space between left wall and mouse
float RT_PADDING = 3.2; // cm, minimum space between right wall and mouse
// Distance -> Voltage equation found from sensor data 4th order polynomial trendlines
// Sen i / Forward formula for 1.5cm-7.5cm
float FWD_VOLT_PADDING = 0.000887*pow(FWD_PADDING, 4)-0.023476*pow(FWD_PADDING, 3)+0.245323*pow(FWD_PADDING, 2)-1.292543*FWD_PADDING+3.568144;
// Sen ii / Right formula for 1.5cm-7.5cm
float RT_VOLT_PADDING = 0.000792*pow(RT_PADDING, 4)-0.021438*pow(RT_PADDING, 3)+0.231216*pow(RT_PADDING, 2)-1.263579*RT_PADDING+3.561291;
// Sen iii / Left formula for 1.5cm-7.5cm
float LT_VOLT_PADDING = 0.001135*pow(LT_PADDING, 4)-0.027837*pow(LT_PADDING, 3)+0.271392*pow(LT_PADDING, 2)-1.357578*LT_PADDING+3.617512;
bool TurningL;
bool TurningR;
int LMicros = 0;
int RMicros = 0;
float distR;
float distL;
float distC;
float diff;
float FWD_DRIFT_TOLERANCE;
float FWD_CENTER_DIST;
int StepsTaken;

void setup() {
  // Setup
  // Microstepping Pin 12
  pinMode(MS1L, OUTPUT);
  pinMode(MS1R, OUTPUT);
  pinMode(MS2L, OUTPUT);
  pinMode(MS2R, OUTPUT);
  // Half-steps
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS1R, HIGH);
  digitalWrite(MS2L, LOW);
  digitalWrite(MS2R, LOW);
  // Step and direction pins
  pinMode(DIR_PINL, OUTPUT);
  pinMode(STEP_PINL, OUTPUT);
  pinMode(DIR_PINR, OUTPUT);
  pinMode(STEP_PINR, OUTPUT);
  // Serial plotting enable
  Serial.begin(9600);
  // Wait 5s before executing to position robot
  FWD_DRIFT_TOLERANCE = 0.3;
  FWD_CENTER_DIST = 3.9;
  delay(5000);
}

void loop() {
  // Need to use non-blocking functions to read sensor data during travel
  for (int n = 0; n < 50; n++){
      AnalogIR_RT = analogRead(SenR);
      AnalogIR_LT = analogRead(SenL);
      AnalogIR_FWD = analogRead(SenC);
      voltsAvg_FWD += AnalogIR_FWD*3.3/4095;
      voltsAvg_RT += AnalogIR_RT*3.3/4095;
      voltsAvg_LT += AnalogIR_LT*3.300/4095;
    }
    voltsAvg_RT = voltsAvg_RT/50;
    voltsAvg_LT = voltsAvg_LT/50;
    voltsAvg_FWD = voltsAvg_FWD/50;
    distR = 2.171083*pow(voltsAvg_RT, 4)-12.811088*pow(voltsAvg_RT, 3)+28.785336*pow(voltsAvg_RT, 2)-31.579924*voltsAvg_RT+17.215482;
    distL = 1.196055*pow(voltsAvg_LT, 4)-7.751272*pow(voltsAvg_LT, 3)+19.610565*pow(voltsAvg_LT, 2)-24.927116*voltsAvg_LT+15.860854;
    distC = 2.115626*pow(voltsAvg_FWD, 4)-12.927186*pow(voltsAvg_FWD, 3)+30.377943*pow(voltsAvg_FWD, 2)-34.877123*voltsAvg_FWD+19.467093;
    diff = distL - distR; // if diff>0, robot drifting right, if diff<0, robot drifting left
    Serial.print(distL);
    Serial.print(", ");
    Serial.println(distR);
    voltsAvg_FWD = 0;
    voltsAvg_RT = 0;
    voltsAvg_LT = 0;
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
  StepsTaken = 0;
  LMicros = 0;
  RMicros = 0;
  for (int i = 1; i <= stepsStrt*Cells; i++){
    // If a wall is too close in front:
    for (int n = 0; n < 15; n++){
      AnalogIR_FWD = analogRead(SenC);
      voltsAvg_FWD += AnalogIR_FWD*3.3/4095;
    }
    voltsAvg_FWD = voltsAvg_FWD/15;
    //Serial.println(voltsAvg_FWD);
    // Stop moving
    if (voltsAvg_FWD > FWD_VOLT_PADDING){
      break;
    }

    // LANE CENTERING
    // Centerline error approach
    // Take readings
    for (int n = 0; n < 15; n++){
      AnalogIR_RT = analogRead(SenR);
      AnalogIR_LT = analogRead(SenL);
      voltsAvg_RT += AnalogIR_RT*3.3/4095;
      voltsAvg_LT += AnalogIR_LT*3.3/4095;
    }
    voltsAvg_RT = voltsAvg_RT/15;
    voltsAvg_LT = voltsAvg_LT/15;
    // Use sensors' voltage->distance formula
    distR = 2.122193*pow(voltsAvg_RT, 4)-13.125209*pow(voltsAvg_RT, 3)+31.063445*pow(voltsAvg_RT, 2)-35.625104*voltsAvg_RT+19.711987;
    distL = 1.305458*pow(voltsAvg_LT, 4)-9.002361*pow(voltsAvg_LT, 3)+23.649247*pow(voltsAvg_LT, 2)-29.953079*voltsAvg_LT+18.265132+0.25;
    diff = distL - distR; // if diff>0, robot drifting right, if diff<0, robot drifting left
    // Allow up to 6mm of drift before correction

    // CENTERING CONTROL
    // 2 Parallel walls case
    // Defined by the existence of a wall in front of both sensors
    // For now say that this happens at the function minima of ~9cm
    /*if (distR <= 9 && distL <= 9) {
      if (diff > FWD_DRIFT_TOLERANCE) {
        //microstep the left wheel for a little while
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS2L, HIGH);
        //only turn 1 wheel at once
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS2R, LOW);
      } else if (diff < -FWD_DRIFT_TOLERANCE) {
        //microstep the right wheel for a little while
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS2R, HIGH);
        //only turn 1 wheel at once
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS2L, LOW);
      } else {
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS2L, LOW);
        digitalWrite(MS2R, LOW);
      }
    // 1 Wall on the left case
    } else if (distR > 9 && distL < 9) {
      //If there is only a wall on the left, try to maintain a certain distance from it of ~4cm
      if (distL <= FWD_CENTER_DIST-FWD_DRIFT_TOLERANCE && StepsTaken % 4 == 0) {
        // Too close to the left, so microstep the right wheel
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS2R, HIGH);
        // only turn 1 wheel at once
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS2L, LOW);
        LMicros++;
      } else if (distL >= FWD_CENTER_DIST+FWD_DRIFT_TOLERANCE && StepsTaken % 4 == 0) {
        // Too far from the left, so microstep the left wheel
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS2L, HIGH);
        // only turn 1 wheel at once
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS2R, LOW);
        RMicros++;
      } else {
        // Let the robot run straight
        digitalWrite(MS1R, HIGH);
        digitalWrite(MS1L, HIGH);
        digitalWrite(MS2R, LOW);
        digitalWrite(MS2L, LOW);
      }
    }
    // We lose distance by microstepping
    if (LMicros+RMicros % 8 == 1){
      i--;
    }*/
    
    /* If a wall is too close to the right:
    for (int n = 0; n < 15; n++){
      AnalogIR_RT = analogRead(SenR);
      voltsAvg_RT += AnalogIR_RT*3.3/4095;
    }
    voltsAvg_RT = voltsAvg_RT/15;
    if (voltsAvg_RT > RT_VOLT_PADDING){
      // Set right wheel to microstep 1/8ths
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      TurningL = 1;
    } else if (voltsAvg_RT < RT_VOLT_PADDING-0.08 && TurningL == 1){
      // Stop turning
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, LOW);
      TurningL = 0;
    } 
    if (TurningL == 1) {
      // Count number of microsteps taken
      LMicros++;
    }
    if (LMicros == 8) {
      i--;
      LMicros = 0;
    }
    
    // If a wall is too close to the left:
    for (int n = 0; n < 15; n++){
      AnalogIR_LT = analogRead(SenL);
      voltsAvg_LT += AnalogIR_LT*3.3/4095;
    }
    voltsAvg_LT = voltsAvg_LT/15;
    if (voltsAvg_LT > LT_VOLT_PADDING){
      // Set right wheel to microstep 1/8ths
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      TurningR = 1;
    } else if (voltsAvg_LT < LT_VOLT_PADDING-0.08 && TurningR == 1){
      // Stop turning
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, LOW);
      TurningR = 0;
    } 
    if (TurningR == 1) {
      // Count number of microsteps taken
      RMicros++;
    }
    if (RMicros == 8) {
      i--;
      RMicros = 0;
    }*/
    
    // Step forward
    digitalWrite(STEP_PINR, HIGH);
    digitalWrite(STEP_PINL, HIGH);
    delay(2);
    digitalWrite(STEP_PINR, LOW);
    digitalWrite(STEP_PINL, LOW);
    delay(2);
    StepsTaken++;
    // Single read
    //AnalogIR = analogRead(SenR);
    //volts = AnalogIR*3.3/4095;
    //Serial.println(volts);
    // Mode
    /*for (int n = 0; n < 5; n++){
      AnalogIR = analogRead(SenR);
      array[n] = AnalogIR*3.3/4095;
    }
    sortArray(array, 5);
    for (int n = 1; i<100; i++){
      if (array[n] == number){
        count++;
      } else {
        if (count > countMode){
          countMode = count;
          Mode = number;
        }
        count = 1;
        number = array[i];
      }
    }
    Serial.print(",");
    Serial.println(Mode);*/
    // Average
    AnalogIR_FWD = 0;
    voltsAvg_FWD = 0;
    for (int n = 0; n < 15; n++){
      AnalogIR_FWD = analogRead(SenC);
      voltsAvg_FWD += AnalogIR_FWD*3.3/4095;
    }
    voltsAvg_FWD = voltsAvg_FWD/15;
    //Serial.println(voltsAvg);
  }
  delay(200);
}

void Rot(int Rotations, bool Dir){
  // Reset microstep controls
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS2L, LOW);
  digitalWrite(MS1R, HIGH);
  digitalWrite(MS2R, LOW);
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
