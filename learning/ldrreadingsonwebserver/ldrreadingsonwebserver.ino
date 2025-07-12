#include <WiFi.h>
#include <WebServer.h>
#include <ESPmDNS.h>

const char* ssid = "Prajeeta";
const char* password = "ra958k2ubh";

WebServer server(80);

const int ledPin = 2;   // Your LED pin
const int ldrPin = 8;   // Analog pin for LDR sensor
const int buzzer=6;

void handleRoot() {
  digitalWrite(ledPin, HIGH);
  int ldrValue = analogRead(ldrPin);
  String message = "Light Level: " + String(ldrValue);
  server.send(200, "text/html", "<html><body><h3>LDR Readings</h1>" + message + "</body></html>");
  if(ldrValue>100)
  {
     digitalWrite(buzzer, HIGH);
     String message = "TOO HIGH!!!";
  }
  else
  {
    digitalWrite(buzzer, LOW);
     String message = "TOO LOW!!!";
  }
  digitalWrite(ledPin, LOW);
}

void handleNotFound() {
  digitalWrite(ledPin, HIGH);
  String message = "File Not Found\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  for (uint8_t i = 0; i < server.args(); i++) {
    message += " " + server.argName(i) + ": " + server.arg(i) + "\n";
  }
  server.send(404, "text/plain", message);
  digitalWrite(ledPin, LOW);
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  Serial.println();

  // Wait for connection
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.print("Connected to ");
  Serial.println(ssid);
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  if (MDNS.begin("esp32")) {
    Serial.println("MDNS responder started");
  }

  server.on("/", handleRoot);

  server.onNotFound(handleNotFound);

  server.begin();
  Serial.println("HTTP server started");
}

void loop() {
  server.handleClient();
  // Add other tasks or functionality here
}
