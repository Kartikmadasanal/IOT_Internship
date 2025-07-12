
#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

static const int RXPin = D1, TXPin = D2; // Use appropriate GPIO pins on NodeMCU for SIM800L
static const int buzzerPin = D7; // Use appropriate GPIO pin for the buzzer
unsigned long interval = 10000;
unsigned long previousMillis = 0;
TinyGPSPlus gps; // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin); // The serial connection to the GPS device

float initialLatitude = 15.882826;
 
float initialLongitude = 74.515321;
const float maxDistance = 30;

void setup()
{
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
  ss.begin(9600); // Baud rate for the SIM800L module

  Serial.println("Starting...");

  // Connect to WiFi
  // WiFi.begin(ssid, password);
  // while (WiFi.status() != WL_CONNECTED)
  // {
  //   delay(1000);
  //   Serial.println("Connecting to WiFi...");
  // }

  // Serial.println("Connected to WiFi");

  ss.println("\r");
  ss.println("AT\r");
  delay(1000);

  ss.println("\r");
  ss.println("AT+GPS=1\r");

  delay(100);
  ss.println("AT+CREG=2\r");
  delay(6000);

  ss.println("AT+CGATT=1\r");
  delay(6000);

  ss.println("AT+CGDCONT=1,\"IP\",\"WWW\"\r");
  delay(6000);

  ss.println("AT+CGACT=1,1\r");
  delay(6000);

  ss.println("\r");
  ss.println("AT+GPS=1\r");
  delay(1000);

  ss.println("AT+GPSRD=1\r");
  delay(100);

  ss.println("AT+CMGF=1\r");
  delay(1000);

  Serial.println("Setup Executed");

}

void loop()
{
  smartDelay(2000);

  if (millis() > 5000 && gps.charsProcessed() < 10)

    Serial.println(F("No GPS data received: check wiring"));

  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - previousMillis) >= interval)
  {
    send_gps_data();
    previousMillis = currentMillis;
  }
}



// smart delay function
static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}




void send_gps_data()
{
  if (gps.location.lat() == 0 || gps.location.lng() == 0)
  {
    Serial.println("No GPS data available");
    return;
  }

  float currentLat = gps.location.lat();
  float currentLng = gps.location.lng();
  Serial.print("Latitude (deg): ");
  Serial.println(currentLat, 6);

  Serial.print("Longitude (deg): ");
  Serial.println(currentLng, 6);
  float distance = getDistance(currentLat, currentLng, initialLatitude, initialLongitude);


  if(distance < maxDistance) {
    // Turn on the buzzer
    digitalWrite(buzzerPin, HIGH);
  Serial.println("enterd the fence");

    // Add a delay if you want the buzzer to stay on for a certain duration
    delay(5000); // Adjust the delay as needed
      digitalWrite(buzzerPin, LOW);

    // Turn off the buzzer
    return;
  }
 

}

float getDistance(float flat1, float flon1, float flat2, float flon2) {
  float diflat = radians(flat2 - flat1);
  flat1 = radians(flat1);
  flat2 = radians(flat2);
  float diflon = radians((flon2) - (flon1));
  float dist_calc = (sin(diflat / 2.0) * sin(diflat / 2.0));
  float dist_calc2 = cos(flat1) * cos(flat2) * sin(diflon / 2.0) * sin(diflon / 2.0);
  dist_calc += dist_calc2;
  dist_calc = (2 * atan2(sqrt(dist_calc), sqrt(1.0 - dist_calc)));
  dist_calc *= 6371000.0;  // Converting to meters
  return dist_calc;

}


