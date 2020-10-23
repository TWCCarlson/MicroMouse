#include <WiFi.h>

const char ssid[] = "laurinet";
const char pass[] = "1onegem99";

const uint ServerPort = 23; // Telnet communications port
WiFiServer server(ServerPort);

void setup() {
  // put your setup code here, to run once:
  Server.begin(115200);

  Serial.print("Connecting to network SSID: ");
  Serial.println(ssid);

  WiFi.begin(ssid, pass);

  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print("Establishing connection to WiFi...");
  }

  Serial.println("Connected with IP: ");
  Serial.println(WiFi.localIP());

  server.beging();
}

void loop() {
  // put your main code here, to run repeatedly:

}

void CheckForConnections() {
  if (Server.hasClient()) {
    if (RemoteClient.connected()) {
      Serial.println("Connection rejected");
      Server.available().stop();
    }
    else {
      Serial.println("Connection accepted");
      RemoteClient = Server.available();
    }
  }
}
