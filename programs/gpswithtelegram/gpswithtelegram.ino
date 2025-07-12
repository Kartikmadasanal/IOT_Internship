#include <SoftwareSerial.h>

static const int RXPin = D0, TXPin = D2;

SoftwareSerial ss(RXPin, TXPin);

// Flag to track whether HTTP POST has been sent


void setup() {
  pinMode(D1, INPUT);

  Serial.begin(9600);
  ss.begin(9600);

  Serial.println("Starting...");

  ss.println("\r");
  ss.println("AT\r");
  delay(10000);
  ss.println("\r");
  ss.println("AT+CGREG=1\r");
  delay(3000);
  ss.println("AT+CGATT=1\r");
  delay(3000);
  ss.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r");
  delay(3000);
  ss.println("AT+CGACT=1,1\r");
  delay(6000);
  ss.println("\r");
  Serial.println("Setup Executed");
}

void loop() {
  int buttonState = digitalRead(D1);

  if (buttonState == HIGH ) {
    // AT+HTTPPOST command corrected
ss.println("AT+HTTPPOST=\"https://api.telegram.org/bot6803373268:AAEPjaoFZTW8rZdhzlCZh2yNrc6v3uwlOHY/sendMessage\",\"application/x-www-form-urlencoded\",\"chat_id=%40testingbotkartik&text=hi\"\r");
    delay(5000);
    Serial.println("SEMDIG");
     // Set the flag to indicate that HTTP POST has been sent

    delay(10000);  // Add a delay to avoid multiple messages with a single press
  }
}
