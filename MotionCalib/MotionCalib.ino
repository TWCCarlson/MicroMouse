//LEFT STEPPER
#define STEP_PINL 32
#define DIR_PINL 14

//RIGHT STEPPER
#define STEP_PINR 33
#define DIR_PINR 15

#define MS1 13
#define MS2 A1

//IR SENSORS
#define SenL A4 // Profile iii
#define SenR A2 // Profile ii
#define SenC A3 // Profile i

bool dir;
float Tstart = 0;
float Tend = 0;
float Tspan = 0;
int stepsStrt = 2*212;
int stepsSpin = 342;
//360deg = 2*2*344 for R
//360deg = 2*2*340 for L
//90deg = 171 full steps for one wheel
//90deg ~= 86 full steps for two wheel
//18cm ~= 212 full steps
//Probably want to call a distance
//float dist =

void setup() {
  // Set up the motors
  // HIGH, HIGH = Forward
  digitalWrite(DIR_PINL, 1);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(DIR_PINR, 1);
  digitalWrite(STEP_PINR, LOW);
  pinMode(MS1, OUTPUT);
  pinMode(MS2, OUTPUT);
  digitalWrite(MS1, HIGH);
  digitalWrite(MS2, HIGH);
  pinMode(DIR_PINL, OUTPUT);
  pinMode(STEP_PINL, OUTPUT);
  pinMode(DIR_PINR, OUTPUT);
  pinMode(STEP_PINR, OUTPUT);
  Serial.begin(9600);
  delay(5000);
  Tstart = millis();
  Serial.println("Test start: ");
  Serial.println(Tstart);
  // Moving straight
  /*for (int i = 1; i<=stepsStrt; i++) {
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
    }*/
  digitalWrite(DIR_PINL, LOW);
  digitalWrite(DIR_PINR, HIGH);
  delay(1000);
  // Spin in place
  /*for (int i = 1; i <= stepsSpin; i++) {
    // Wheels spin opposite, motors spin with equal direction variable
    // L HIGH, R LOW = CW
    // L LOW, R HIGH = CCW
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
  }*/
  Tend = millis();
  Tspan = Tend - Tstart;
  Serial.println("Test end: ");
  Serial.println(Tend);
  Serial.println("Test duration: ");
  Serial.println(Tspan);
}

void loop() {
  // put your main code here, to run repeatedly:
  // Go forward one square
  /*digitalWrite(DIR_PINL, 1);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(DIR_PINR, 1);
  digitalWrite(STEP_PINR, LOW);
  for (int i = 1; i<=stepsStrt; i++) {
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
    }
  delay(1000);
  // Spin 180 deg
  digitalWrite(DIR_PINL, 0);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(DIR_PINR, 1);
  digitalWrite(STEP_PINR, LOW);
  for (int i = 1; i <= stepsSpin; i++) {
    // Wheels spin opposite, motors spin with equal direction variable
    // L HIGH, R LOW = CW
    // L LOW, R HIGH = CCW
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
  }
  // Forward again
  digitalWrite(DIR_PINL, 1);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(DIR_PINR, 1);
  digitalWrite(STEP_PINR, LOW);
  delay(1000);
  for (int i = 1; i<=stepsStrt; i++) {
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
    }
  delay(1000);
  // Spin 180 deg, opposite direction
  digitalWrite(DIR_PINL, 1);
  digitalWrite(STEP_PINL, LOW);
  digitalWrite(DIR_PINR, 0);
  digitalWrite(STEP_PINR, LOW);
  for (int i = 1; i <= stepsSpin; i++) {
    // Wheels spin opposite, motors spin with equal direction variable
    // L HIGH, R LOW = CW
    // L LOW, R HIGH = CCW
    digitalWrite(STEP_PINL, HIGH);
    digitalWrite(STEP_PINR, HIGH);
    delay(2);
    digitalWrite(STEP_PINL, LOW);
    digitalWrite(STEP_PINR, LOW);
    delay(2);
  }
  delay(1000);*/
}
