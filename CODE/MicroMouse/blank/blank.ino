#define STEP_PIN2 33
#define DIR_PIN2 15

#define STEP_PIN 13
#define DIR_PIN 14
 
bool dirHigh;
 
void setup()
{
  dirHigh = true;
  //digitalWrite(DIR_PIN, LOW);
  //digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN2, LOW);
  digitalWrite(STEP_PIN2, LOW);
  //pinMode(DIR_PIN, OUTPUT);
  //pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
}
 
void loop()
{
    // Trigger the motor to take one step.
    //digitalWrite(STEP_PIN, HIGH);
    digitalWrite(STEP_PIN2, HIGH);
    delay(1);
    //digitalWrite(STEP_PIN, LOW);
    digitalWrite(STEP_PIN2, LOW);
    delay(1);
}
