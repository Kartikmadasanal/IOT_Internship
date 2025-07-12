#include <ESP8266WiFi.h>
#include <WiFiClientSecure.h>
#include <UniversalTelegramBot.h>
#include <ArduinoJson.h>

// Replace with your network credentials
const char* ssid = "1111111111";
const char* password = "12345678";

// Initialize Telegram BOT
#define BOTtoken "6692921732:AAGFPVFp-iZpIM2G0x855vS1UsscveDIrVg"  // your Bot Token (Get from Botfather)

// Use @myidbot to find out the chat ID of an individual or a group
// Also note that you need to click "start" on a bot before it can
// message you
#define CHAT_ID "1220633691"

// Define the pin for the button
const int buttonPin = D2; // Change this to the pin where your button is connected on ESP8266

WiFiClientSecure client; 
UniversalTelegramBot bot(BOTtoken, client);

void setup() {
  Serial.begin(115200);
  delay(1000); // give me time to bring up serial monitor
  Serial.println("ESP8266 Touch Test");

  // Attempt to connect to Wifi network:
  Serial.print("Connecting Wifi: ");
  Serial.println(ssid);

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  
  // Setup button pin
  pinMode(buttonPin, INPUT);

  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(500);
  }

  Serial.println("");
  Serial.println("WiFi connected");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  bot.sendMessage(CHAT_ID, "Bot started up", "");
}

void loop() {
  int buttonState = digitalRead(buttonPin);

  // Serial.println(touchRead(A0));  /// get value of Touch 0 pin = A0 on ESP8266
  delay(1000);
  
  // if (touchRead(A0) <= 10) {
  //   bot.sendMessage(CHAT_ID, "Touch Detected", "");
  //   Serial.println("Humidity Updated In percentage");
  //   delay(1000);
  // }

  // Check if the button is pressed
  if (buttonState == HIGH) {
    bot.sendMessage(CHAT_ID, "Button Pressed", "");
    delay(1000); // Add a delay to avoid multiple messages with a single press
  }
}
