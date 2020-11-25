//LEFT STEPPER
#define STEP_PIN2 33
#define DIR_PIN2 15

//RIGHT STEPPER
#define STEP_PIN 32
#define DIR_PIN 14

#define SenL A2
#define SenC A3
#define SenR A4
 
bool dirHigh;
 
void setup()
{
  dirHigh = true;
  digitalWrite(DIR_PIN, LOW); //HIGH: RIGHT WHEEL FWD
  digitalWrite(STEP_PIN, LOW);
  digitalWrite(DIR_PIN2, HIGH); //LOW: LEFT WHEEL FWD
  digitalWrite(STEP_PIN2, LOW);
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  pinMode(DIR_PIN2, OUTPUT);
  pinMode(STEP_PIN2, OUTPUT);
  Serial.begin(115200);
  
}
 
void loop()
{
    float volts1 = analogRead(SenC);
    //float volts11 = analogRead(SenL);
    //float volts21 = analogRead(SenR);
    delay(1);
    float volts2 = analogRead(SenC);
    //float volts12 = analogRead(SenL);
    //float volts22 = analogRead(SenR);
    delay(1);
    float volts3 = analogRead(SenC);
    //float volts13 = analogRead(SenL);
    //float volts23 = analogRead(SenR);
    delay(1);
    float volts4 = analogRead(SenC);
    //float volts14 = analogRead(SenL);
    //float volts24 = analogRead(SenR);
    delay(1);
    float volts5 = analogRead(SenC);
    //float volts15 = analogRead(SenL);
    //float volts25 = analogRead(SenR);
    delay(1);
    float Voltavg = ((volts1))*3.3/4095;
    Serial.println(Voltavg);
    //delay(100);

    if (Voltavg < 1.2) {
      // FWD
      digitalWrite(DIR_PIN, LOW); //HIGH: RIGHT WHEEL FWD
      digitalWrite(DIR_PIN2, HIGH); //LOW: LEFT WHEEL FWD
      
      digitalWrite(STEP_PIN, HIGH);
      digitalWrite(STEP_PIN2, HIGH);
      delay(2);
      digitalWrite(STEP_PIN, LOW);
      digitalWrite(STEP_PIN2, LOW);
      delay(2);
    }
    if (Voltavg > 1.2) {
      // REVERSE
      digitalWrite(DIR_PIN, HIGH); //HIGH: RIGHT WHEEL FWD
      digitalWrite(DIR_PIN2, LOW); //LOW: LEFT WHEEL FWD
      
      digitalWrite(STEP_PIN, HIGH);
      digitalWrite(STEP_PIN2, HIGH);
      delay(2);
      digitalWrite(STEP_PIN, LOW);
      digitalWrite(STEP_PIN2, LOW);
      delay(2);
    }
    // Trigger the motor to take one step.
    
}
