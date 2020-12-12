#include <WiFi.h>
#include <WebServer.h>

const char ssid[] = "laurinet";
const char pass[] = "1onegem99";
int status = WL_IDLE_STATUS;
// Tell router to reserve the IP!
// 192.168.0.185
// MAC AC:67:B2:09:19:54

// Establish the ESP-32 webserver
WebServer server(80);
String header;

//Trying to toggle the red usb LED
#define LEDpin 13

void setup() {
  Serial.begin(9600);
  connectToNetwork();

  Serial.println(WiFi.localIP());
  Serial.println(WiFi.macAddress());

  //WiFi.disconnect(true);
  //Serial.println(WiFi.localIP());

  pinMode(LEDpin, OUTPUT);
  digitalWrite(LEDpin, LOW);

  server.begin();
  Serial.println("HTTP Server started...");

  // Each webpage can be thought of as a "function" to be called by a remote client
  // Every handle_ function is the function called upon client requesting a webapge

  // Handle connection to the root directory
  server.on("/", handle_onConnect);
  // Handle connection to the LED-ON directory
  server.on("/ledon", handle_ledon);
  // Handle connection to the LED-OFF directory
  server.on("/ledoff", handle_ledoff);
  // Other urls are not valid, return an error
  server.onNotFound(handle_NotFound);
}

void loop() {
  server.handleClient();
}

// ======================
// ===== NETWORKING =====
// ======================

void connectToNetwork() {
  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Establishing connection to WiFi..");
  }

  Serial.println("Connected to network");
}

void handle_onConnect() {
  digitalWrite(LEDpin, LOW);
  Serial.println("Welcome. Reinitialized LED to OFF.");
  server.send(200, "text/html", "INITIALIZED");
}

void handle_ledon() {
  int countdown = 10;
  digitalWrite(LEDpin, HIGH);
  Serial.println("LED ON");
  server.send(200, "text/plain", "LED ON");
}

void handle_ledoff() {
  digitalWrite(LEDpin, LOW);
  Serial.println("LED OFF");
  server.send(200, "text/plain", "LED OFF");
}

void handle_NotFound() {
  server.send(404, "text/plain", "NOT FOUND");
}
