#define STEP_PIN 13
#define DIR_PIN 12
 
bool dirHigh;
 
void setup()
{
  dirHigh = true;
  digitalWrite(DIR_PIN, LOW);
  digitalWrite(STEP_PIN, LOW);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
}
 
void loop()
{
    // Trigger the motor to take one step.
    digitalWrite(STEP_PIN, HIGH);
    delay(1);
    digitalWrite(STEP_PIN, LOW);
    delay(1);
}
