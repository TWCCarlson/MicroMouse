//Mode testing
#include <ArduinoSort.h>
float array[100];
//Setup for button
//Pushing the button should trigger a sampling and averaging period, then return a result
const int buttonPin = 13;
//const int ledPin = 13;
int buttonState = 0;
//Setup for IR sensor
#define Sen A1
float volts;
float voltavg;

void setup() {
  // put your setup code here, to run once:
  //Initialize the button
  //pinMode(ledPin, OUTPUT);
  pinMode(buttonPin, INPUT);
  //Open the serial monitor
  Serial.begin(9600);
}

void loop() {
  // put your main code here, to run repeatedly:
  //Read the button state
  volts = analogRead(Sen);
  volts = volts*3.3/1024;
  Serial.println(volts);
  voltavg = 0;
  buttonState = digitalRead(buttonPin);
  /*Print the button state
  Serial.println(buttonState);*/
  /*Control the onboard LED
  //Lights up when button pressed
  if (buttonState == HIGH){
    digitalWrite(ledPin, HIGH);
  } else {
    digitalWrite(ledPin, LOW);
  }*/
  //If the button is pressed, sample 30 times
  /*if (buttonState == HIGH){
    Serial.println("Acquiring...");
    for (int i = 1; i <= 30; i++) {
      volts = analogRead(Sen);
      //Serial.println("Read: ");
      //Serial.println(volts);
      //convert to an actual voltage reading
      //3v3, 1023 values for UNO
      //3v3, 4095 values for ESP32
      volts = volts*3.3/4095;
      //Serial.println("Voltage avg read: ");
      //Serial.println(volts);
      voltavg = voltavg + volts;
      delay(10);
    }
    voltavg = voltavg / 30;
    Serial.println("Sample average voltage: ");
    Serial.println(voltavg);
  }*/

  //Using mode
  /*float number = array[0];
  float Mode = number;
  int count = 1;
  int countMode = 1;
  if (buttonState == HIGH){
    Serial.println("Acquiring...");
    //Take down values and store in an array
    for (int i = 0; i<100; i++) {
      volts = analogRead(Sen);
      array[i] = volts*3.3/4095;
      //Serial.println(array[i]);
      delay(10);
    }
    //Sort the array to find the mode
    sortArray(array, 100);
    //Determine the mode
    for (int i = 1; i<100; i++) {
      if (array[i] == number){
        count++;
      }
      else {
        if (count > countMode){
          countMode = count;
          Mode = number;
        }
        count = 1;
        number = array[i];
      }
    }
    Serial.println("Sample mode voltage: ");
    Serial.println(Mode);
  }
  
  /*float volts1 = analogRead(Sen);
  delay(5);
  float voltavg = ((volts1))*5/4095;
  Serial.println(voltavg);*/
}
