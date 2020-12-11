// ===== Thomas W. C. Carlson =====
// EEE225 "simple" MicroMouse code
// This is the culmination of a lot of work and effort in a very short amount of time and as such has limited applicability to general maze solving and especially competitive maze solving
// This should be taken as an example of what is possible at the level of a Bachelor's in Mechanical Engineering, a novice's understanding of Electrical Engineering and Computer Science, and a PhD in googling.
// ===== END INTRODUCTION =========


// ===== INPUT VAR DECLARATION ====
// Analog readings
float AnalogIR_FWD;
float AnalogIR_RT;
float AnalogIR_LT;
// Voltage conversion storage
float volts_FWD;
float volts_RT;
float volts_LT;
// Voltage period average storage
float voltsAvg_FWD;
float voltsAvg_RT;
float voltsAvg_LT;
// Distance conversion storage
float distC;    // cm, distance between forward sensor and wall
float distR;    // cm, distance between right sensor and wall
float distL;    // cm, distance between left sensor and wall
float diff;
// ===== END INPUT VAR DEC =======



// ===== PIN DEFINITIONS ==========
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
// ===== END PIN DEFINITIONS ======



// ===== GLOBAL CONSTANTS ========
float FWD_PADDING = 2.5;    // cm, stopping distance between forward sensor and wall
float RT_PADDING = 3.2;   // cm, stopping distance between right sensor and wall
float LT_PADDING = 3.2;   // cm, stopping distance between left sensor and wall
float FWD_DRIFT_TOLERANCE = 0.3;    // cm, acceptable deviation from lane center during forward travel
float FWD_CENTER_DIST = 3.7;        // cm, distance between each wall and sensor during forward travel
int SCOUT_STEPS = 200;              // steps, empirically determined distance for checking next square walls
int STEPS_STRT = 420;               // steps, empirically determined distance for moving 1 cell center-to-center
int STEPS_SPIN = 172;               // steps, empirically determined distance for rotating 90 degrees in place
// Because the analog -> voltage readings are much less noisy than analog -> voltage -> distance readings, we prefer to set the padding around the voltage
// This also means that the accuracy interval is better. Estimate ~1.5cm-7.5cm to be "accurate" for curve generation, and 2cm-5cm to be "accurate" for in-maze use
// Distance -> Voltage equations are taken from Sen1.xlsx, Sen2.xlsx, Sen3.xlsx results of sensor calibration
// Sensor 1 is the forward sensor:
float FWD_VOLT_PADDING = 0.000887*pow(FWD_PADDING, 4)-0.023476*pow(FWD_PADDING, 3)+0.245323*pow(FWD_PADDING, 2)-1.292543*FWD_PADDING+3.568144;
// Sensor 2 is the right sensor:
float RT_VOLT_PADDING = 0.000792*pow(RT_PADDING, 4)-0.021438*pow(RT_PADDING, 3)+0.231216*pow(RT_PADDING, 2)-1.263579*RT_PADDING+3.561291;
// Sensor 3 is the left sensor:
float LT_VOLT_PADDING = 0.001135*pow(LT_PADDING, 4)-0.027837*pow(LT_PADDING, 3)+0.271392*pow(LT_PADDING, 2)-1.357578*LT_PADDING+3.617512;
// ===== END GLOBAL CONSTANTS ===


// ===== CONTROL VARIABLES ======


void setup() {
  // ==== MICROSTEPPING PINS ====
  // Refer to Pololu MP6500 microstepping chart for HIGH/LOW value relations
  pinMode(MS1L, OUTPUT);
  pinMode(MS2L, OUTPUT);
  pinMode(MS1R, OUTPUT);
  pinMode(MS2R, OUTPUT);
  // ==== STEP AND DIR PINS ====
  pinMode(DIR_PINL, OUTPUT);
  pinMode(STEP_PINL, OUTPUT);
  pinMode(DIR_PINR, OUTPUT);
  pinMode(STEP_PINR, OUTPUT);
  
  // ==== INITIAL SETTINGS ====
  // Mouse operation is much smoother using half-steps, so they should be the default:
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS1R, HIGH);
  digitalWrite(MS2L, LOW);
  digitalWrite(MS2R, LOW);
  
  // ==== DEBUG OPTIONS =======
  Serial.begin(9600);

  // ==== CODE TEST ===========
  delay(5000);
}

void loop() {
  // put your main code here, to run repeatedly:
  int randomDir;
  randomDir = random(0,2);
  int randomTurns;
  randomTurns = random(0,8);
  randomTurns = randomTurns % 4;
  Serial.print("Turn Direction: ");
  Serial.println(randomDir);
  Serial.print("Number of 90 deg rotations: ");
  Serial.println(randomTurns);
  Fwd_Unknown();
  delay(1000);
  Rot(randomDir,randomTurns);
  delay(1000);
}




// ============================
// ===== SENSOR FUNCTIONS =====
// ============================

void GetVoltageFWD() {
  // Sample 50 times and get an average distance between forward sensor and a wall
  for (int n = 0; n < 50; n++) {
    // Read the analog value
    AnalogIR_FWD = analogRead(SenC);
    // Convert to voltage
    voltsAvg_FWD += AnalogIR_FWD * 3.3 / 4095;
  }
  // Take the average value
  voltsAvg_FWD = voltsAvg_FWD / 50;
  // Debug
  //Serial.println(voltsAvg_FWD);
}

void GetVoltageRT() {
  // Sample 50 times and get an average distance between forward sensor and a wall
  for (int n = 0; n < 50; n++) {
    // Read the analog value
    AnalogIR_RT = analogRead(SenR);
    // Convert to voltage
    voltsAvg_RT += AnalogIR_RT * 3.3 / 4095;
  }
  // Take the average value
  voltsAvg_RT = voltsAvg_RT / 50;
  // Debug
  //Serial.println(voltsAvg_RT);
}

void GetVoltageLT() {
  // Sample 50 times and get an average distance between forward sensor and a wall
  for (int n = 0; n < 50; n++) {
    // Read the analog value
    AnalogIR_LT = analogRead(SenL);
    // Convert to voltage
    voltsAvg_LT += AnalogIR_LT * 3.3 / 4095;
  }
  // Take the average value
  voltsAvg_LT = voltsAvg_LT / 50;
  // Debug
  //Serial.println(voltsAvg_LT);
}

void VoltageToDistFWD() {
  // Convert the voltage of the forward sensor to a distance in cm
  // Using Voltage -> Distance formula from calibration in Excel
  // Fwd Sensor for Mouse 1 uses Sen_1.xlsx
  distC = 2.115626*pow(voltsAvg_FWD, 4)-12.927186*pow(voltsAvg_FWD, 3)+30.377943*pow(voltsAvg_FWD, 2)-34.877123*voltsAvg_FWD+19.467093;
}

void VoltageToDistRT() {
  // Convert the voltage of the forward sensor to a distance in cm
  // Using Voltage -> Distance formula from calibration in Excel
  // Fwd Sensor for Mouse 1 uses Sen_2.xlsx
  distR = 2.171083*pow(voltsAvg_RT, 4)-12.811088*pow(voltsAvg_RT, 3)+28.785336*pow(voltsAvg_RT, 2)-31.579924*voltsAvg_RT+17.215482;
}

void VoltageToDistLT() {
  // Convert the voltage of the forward sensor to a distance in cm
  // Using Voltage -> Distance formula from calibration in Excel
  // Fwd Sensor for Mouse 1 uses Sen_3.xlsx
  distL = 1.196055*pow(voltsAvg_LT, 4)-7.751272*pow(voltsAvg_LT, 3)+19.610565*pow(voltsAvg_LT, 2)-24.927116*voltsAvg_LT+15.860854;
  
}

void DisplaySensorVoltageReads() {
  // For debugging
  // Smoother representation of input reads
  // Read each sensor
  GetVoltageFWD();
  GetVoltageRT();
  GetVoltageLT();
  // Display results
  Serial.print(voltsAvg_LT);
  Serial.print(", ");
  Serial.print(voltsAvg_FWD);
  Serial.print(", ");
  Serial.println(voltsAvg_RT);
}

void DisplaySensorDistReads() {
  // For debugging
  // Read each sensor
  GetVoltageFWD();
  GetVoltageRT();
  GetVoltageLT();
  // Convert read to 
  VoltageToDistFWD();
  VoltageToDistRT();
  VoltageToDistLT();
  // Display results
  Serial.print(distL);
  Serial.print(", ");
  Serial.print(distC);
  Serial.print(", ");
  Serial.println(distR);
}

int DetectWallToLT() {
  int WallExistsLT;
  // Check if there is a wall to the left
  GetVoltageLT();
  VoltageToDistLT();
  if (distL >= 9) {
    // Then there is no wall
    WallExistsLT = 0;
  } else {
    // Then there is a wall
    WallExistsLT = 1;
  }
  return WallExistsLT;
}

int DetectWallToRT() {
  int WallExistsRT;
  // Check if there is a wall to the right
  GetVoltageRT();
  VoltageToDistRT();
  if (distR >= 9) {
    // Then there is no wall
    WallExistsRT = 0;
  } else {
    // Then there is a wall
    WallExistsRT = 1;
  }
  return WallExistsRT;
}

int DetectWallFWD() {
  int WallExistsFWD;
  // Check if there is a wall in front
  GetVoltageFWD();
  VoltageToDistFWD();
  if (distR >= 9) {
    // Then there is no wall
    WallExistsFWD = 0;
  } else {
    // Then there is a wall
    WallExistsFWD = 1;
  }
  return WallExistsFWD;
}

// ===========================
// ===== MOTOR FUNCTIONS =====
// ===========================

void Step() {
  // Iterate one step
  digitalWrite(STEP_PINL, HIGH);
  digitalWrite(STEP_PINR, HIGH);
  delay(2);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(STEP_PINR, LOW);
  delay(2);
}

void SetDirFWD() {
  // Set direction pins for forward movement
  digitalWrite(DIR_PINL, HIGH);
  digitalWrite(DIR_PINR, HIGH);
}

void SetDirCW() {
  // Set direction pins for clockwise rotation
  digitalWrite(DIR_PINL, HIGH);
  digitalWrite(DIR_PINR, LOW);
}

void SetDirCCW() {
  // Set direction pins for counter clockwise rotation
  digitalWrite(DIR_PINL, LOW);
  digitalWrite(DIR_PINR, HIGH);
}

void Fwd_2Walls(int Steps) {
  SetDirFWD();
  int LMicros = 0;
  int RMicros = 0;
  for (int i = 1; i <= Steps; i++){
    float diff;
    // Move forward a number of steps with 2 wall centering
    // 2 wall centering requires distance readings from each sensor
    GetVoltageLT();
    GetVoltageRT();
    // Convert voltages to distances
    VoltageToDistLT();
    VoltageToDistRT();
    // With 2 walls we can use the difference in distance as a centering method
    // A positive difference indicates being closer to the right
    // A negative difference indicates being closer to the left
    diff = distL - distR;
    // Issue corrections by changing microstepping
    if (diff > FWD_DRIFT_TOLERANCE) {
      // Microstep the left wheel for a little while
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Ensure right wheel continues half stepping
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, LOW);
      // Track number of microsteps
      LMicros++;
    } else if (diff < -FWD_DRIFT_TOLERANCE) {
      // Microstep the right wheel for a little while
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      // Ensure left wheel continues half stepping
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, LOW);
      // Track number of microsteps
      RMicros++;
    } else {
      // Continue straight
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2L, LOW);
      digitalWrite(MS2L, LOW);
    }
    // If a wall is encountered in the forward direction, stop moving
    GetVoltageFWD();
    VoltageToDistFWD();
    if (distC <= FWD_PADDING) {
      Serial.println("Stopped due to forward wall");
      break;
    }
    Step();
  }
  // Debug
  delay(50);
  //Serial.print("Total microsteps taken: ");
  //Serial.println(RMicros + LMicros);
  // Try to correct direction if not parallel to cell
  if (RMicros > LMicros) {
    // Microstepped the right wheel more, so we may be turned right
    for (int j = 0; j < (RMicros-LMicros)/2; j++) {
      // Want to perform a CCW turn
      SetDirCCW();
      // Microstep the left wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  } else if (RMicros < LMicros) {
    // Microstepped the left wheel more, so we may be turned left
    for (int j = 0; j < (LMicros-RMicros)/2; j++) {
      // Want to perform a CW turn
      SetDirCW();
      // Microstep the right wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  }
  // Distance loss correction
  // Use recursive calling to drive the error to zero while checking angular heading and linear position
  if  (LMicros + RMicros > 100) {
    Steps = (LMicros + RMicros)*0.5;
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_2Walls(Steps);
  } else if (LMicros + RMicros > 50) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.8;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_2Walls(Steps);
  } else if (LMicros + RMicros > 30) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.9;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_2Walls(Steps);
  } else if (LMicros + RMicros > 10) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*1;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_2Walls(Steps);
  }
  // For very large angle differences the recursion can cause excessive total forward distance travel
  // Need to fix this
}

void Fwd_LWall(int Steps) {
  // Move forward a number of steps with left wall centering
  SetDirFWD();
  int LMicros = 0;
  int RMicros = 0;
  for (int i = 1; i < Steps; i++) {
    // Move forward a number of steps with left wall centering
    // left wall centering requires distance readings from the left sensor
    GetVoltageLT();
    // Convert voltages to distances
    VoltageToDistLT();
    // With only a left wall we rely on the distance from the wall for centering
    // A positive distance indicates being too far from the left wall
    // A negative distance indicates being too close to the left wall
    diff = distL - FWD_CENTER_DIST;
    if (diff > FWD_DRIFT_TOLERANCE) {
      // Microstep the left wheel for a little while
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Ensure right wheel continues half stepping
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, LOW);
      // Track number of microsteps
      LMicros++;
    } else if (diff < -FWD_DRIFT_TOLERANCE) {
      // Microstep the right wheel for a little while
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      // Ensure left wheel continues half stepping
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, LOW);
      // Track number of microsteps
      RMicros++;
    } else {
      // Continue straight
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2L, LOW);
      digitalWrite(MS2L, LOW);
    }
  // If a wall is encountered in the forward direction, stop moving
    GetVoltageFWD();
    VoltageToDistFWD();
    if (distC <= FWD_PADDING) {
      Serial.println("Stopped due to forward wall");
      break;
    }
    Step();
  }
  // Debug
  delay(50);
  //Serial.print("Microsteps taken: ");
  //Serial.println(LMicros + RMicros);
  // Try to correct direction if not parallel to cell
  if (RMicros > LMicros) {
    // Microstepped the right wheel more, so we may be turned right
    for (int j = 0; j < (RMicros-LMicros)/2; j++) {
      // Want to perform a CCW turn
      SetDirCCW();
      // Microstep the left wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  } else if (RMicros < LMicros) {
    // Microstepped the left wheel more, so we may be turned left
    for (int j = 0; j < (LMicros-RMicros)/2; j++) {
      // Want to perform a CW turn
      SetDirCW();
      // Microstep the left wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  }
  // Distance loss correction
  // Use recursive calling to drive the error to zero while checking angular heading and linear position
  if (LMicros + RMicros > 100) {
    Steps = (LMicros + RMicros)*0.5;
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_LWall(Steps);
  } else if (LMicros + RMicros > 50) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.8;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_LWall(Steps);
  } else if (LMicros + RMicros > 30) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.9;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_LWall(Steps);
  } else if (LMicros + RMicros > 10) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*1;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_LWall(Steps);
  }
}

void Fwd_RWall(int Steps) {
  // Move forward a number of steps with right wall centering
  SetDirFWD();
  int LMicros = 0;
  int RMicros = 0;
  for (int i = 1; i < Steps; i++) {
    // Move forward a number of steps with left wall centering
    // left wall centering requires distance readings from the left sensor
    GetVoltageRT();
    // Convert voltages to distances
    VoltageToDistRT();
    // With only a left wall we rely on the distance from the wall for centering
    // A positive distance indicates being too far from the right wall
    // A negative distance indicates being too close to the right wall
    diff = FWD_CENTER_DIST - distR;
    if (diff > FWD_DRIFT_TOLERANCE) {
      // Microstep the left wheel for a little while
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Ensure right wheel continues half stepping
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, LOW);
      // Track number of microsteps
      LMicros++;
    } else if (diff < -FWD_DRIFT_TOLERANCE) {
      // Microstep the right wheel for a little while
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      // Ensure left wheel continues half stepping
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, LOW);
      // Track number of microsteps
      RMicros++;
    } else {
      // Continue straight
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2L, LOW);
      digitalWrite(MS2L, LOW);
    }
    // If a wall is encountered in the forward direction, stop moving
    GetVoltageFWD();
    VoltageToDistFWD();
    if (distC <= FWD_PADDING) {
      Serial.println("Stopped due to forward wall");
      break;
    }
    Step();
  }
  // Debug
  delay(50);
  //Serial.print("Microsteps taken: ");
  //Serial.println(LMicros + RMicros);
  // Try to correct direction if not parallel to cell
  if (RMicros > LMicros) {
    // Microstepped the right wheel more, so we may be turned right
    for (int j = 0; j < (RMicros-LMicros)/2; j++) {
      // Want to perform a CCW turn
      SetDirCCW();
      // Microstep the left wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  } else if (RMicros < LMicros) {
    // Microstepped the left wheel more, so we may be turned left
    for (int j = 0; j < (LMicros-RMicros)/2; j++) {
      // Want to perform a CW turn
      SetDirCW();
      // Microstep the left wheel
      digitalWrite(MS1L, HIGH);
      digitalWrite(MS2L, HIGH);
      // Microstep the right wheel
      digitalWrite(MS1R, HIGH);
      digitalWrite(MS2R, HIGH);
      Step();
    }
  }
  // Distance loss correction
  // Use recursive calling to drive the error to zero while checking angular heading and linear position
  if  (LMicros + RMicros > 100) {
    Steps = (LMicros + RMicros)*0.5;
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_RWall(Steps);
  } else if (LMicros + RMicros > 50) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.8;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_RWall(Steps);
  } else if (LMicros + RMicros > 30) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*0.9;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_RWall(Steps);
  } else if (LMicros + RMicros > 10) {
    // An empirically determined fraction of the microsteps taken
    Steps = (LMicros + RMicros)*1;
    // Debug
    delay(50);
    //Serial.print("Linear correction steps: ");
    //Serial.println(Steps);
    Fwd_RWall(Steps);
  }
}

void Fwd_NoWall(int Steps) {
  // Move forward a number of steps with no wall centering and pray for no collision
  SetDirFWD();
  for (int i = 1; i < Steps; i++) {
    // Continue straight
    digitalWrite(MS1L, HIGH);
    digitalWrite(MS1R, HIGH);
    digitalWrite(MS2L, LOW);
    digitalWrite(MS2L, LOW);
    // If a wall is encountered in the forward direction, stop moving
    GetVoltageFWD();
    VoltageToDistFWD();
    if (distC <= FWD_PADDING) {
      Serial.println("Stopped due to forward wall");
      break;
    }
    Step();
  }
  
}

void Fwd_Unknown() {
  int StepsTaken;
  int StepsRemaining;
  int WallArray[3];
  // The condition where the mouse does not know what walls it can use to guide itself as it moves forward
  // Start by moving forward and checking for the wall condition ahead
  StepsTaken = Scout(WallArray);
  StepsRemaining = STEPS_STRT - StepsTaken;
  // Once the walls are identified, return to guided movement functions
  if (WallArray[0] == 1 && WallArray[2] == 1) {
    // Walls on both sides
    // Move forward with 2 points of guidance
    Serial.println("2 Walls available");
    Fwd_2Walls(StepsRemaining);
  } else if (WallArray[0] == 1 && WallArray[2] == 0) {
    // Wall on left side only
    // Move forward with left point of guidance
    Serial.println("Left wall only");
    Fwd_LWall(StepsRemaining);
  } else if (WallArray[0] == 0 && WallArray[2] == 1) {
    // Wall on right side only
    // Move forward with right point of guidance
    Serial.println("Right wall only");
    Fwd_RWall(StepsRemaining);
  } else {
    // No walls available
    // Move forward with no guidance and pray
    Serial.println("NO WALL -- Pray for rescue");
    Fwd_NoWall(StepsRemaining);
  }
}

int Scout(int (& WallArray)[3]) {
  int StepsTaken;
  // Move Forward some steps, and check for wall presence
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS1R, HIGH);
  digitalWrite(MS2L, LOW);
  digitalWrite(MS2R, LOW);
  SetDirFWD();
  for (int i = 0; i < SCOUT_STEPS; i++) {
    GetVoltageFWD();
    VoltageToDistFWD();
    if (distC <= FWD_PADDING) {
      Serial.println("Stopped due to forward wall");
      break;
    }
    Step();
  }
  // Check for walls
  // Left wall
  WallArray[0] = DetectWallToLT();
  // Right wall
  WallArray[2] = DetectWallToRT();
  StepsTaken = SCOUT_STEPS;
  return StepsTaken;
}

void Rot(bool Dir, int N) {
  // Rotate the robot N*90 degrees
  // Reset microsteps
  digitalWrite(MS1L, HIGH);
  digitalWrite(MS1R, HIGH);
  digitalWrite(MS2L, LOW);
  digitalWrite(MS2R, LOW);
  // Dir = true means CW rotation
  // Dir = false means CCW rotation
  if (Dir == 1) {
    SetDirCW();
  } else {
    SetDirCCW();
  }
  for (int i = 0; i < N*STEPS_SPIN; i++){
    Step();
  }
}

// Scouting
// Forward
//    2 walls
//    L wall
//    R wall
//    0 wall
//    Unknown wall
// Rotation
