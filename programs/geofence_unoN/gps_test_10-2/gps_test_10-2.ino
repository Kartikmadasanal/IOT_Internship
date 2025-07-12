#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 5;
static const int buzzerPin = 7;  // Define the buzzer pin
String s = "kartik_test";

unsigned long interval = 10000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;

TinyGPSPlus gps;  // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin);  // The serial connection to the GPS device

// Define geofence boundaries
float fenceMinLat = 37.7749; // Minimum latitude of the geofence
float fenceMaxLat = 37.8049; // Maximum latitude of the geofence
float fenceMinLng = -122.4194; // Minimum longitude of the geofence
float fenceMaxLng = -122.3894; // Maximum longitude of the geofence

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);

  // ... (the rest of your setup code)

  // Set the buzzer pin as an OUTPUT
  pinMode(buzzerPin, OUTPUT);

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
    checkGeoFence();
    previousMillis = currentMillis;
  }
}

static void smartDelay(unsigned long ms)
{
  unsigned long start = millis();
  do
  {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}

void checkGeoFence()
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

  if (currentLat >= fenceMinLat && currentLat <= fenceMaxLat &&
      currentLng >= fenceMinLng && currentLng <= fenceMaxLng)
  {
    Serial.println("Inside the Geofence");
    // Turn on the buzzer
    digitalWrite(buzzerPin, HIGH);

    // Add a delay if you want the buzzer to stay on for a certain duration
    delay(2000);  // Adjust the delay as needed

    // Turn off the buzzer
    digitalWrite(buzzerPin, LOW);
  }
  else
  {
    Serial.println("Outside the Geofence");
  }
}
