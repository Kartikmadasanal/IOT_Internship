



// test


#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>
#include <ESP8266WiFi.h>

#define MAX_GEOFENCES 3

struct Geofence {
  float latitude;
  float longitude;
  float maxDistance;
};

Geofence geofences[MAX_GEOFENCES];

static const int RXPin = D1, TXPin = D2;
static const int buzzerPin = D7;
unsigned long interval = 10000;
unsigned long previousMillis = 0;
int i;
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

// float initialLatitude = 15.867157;
// float initialLongitude = 74.511392;

void setup() {
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
  ss.begin(9600);

  Serial.println("Starting...");

  // Initialize geofences (add as many as needed)
  geofences[0] = { 15.881553, 74.513808, 20 };  // Geofence 1
  geofences[1] = { 15.881497, 74.515174, 20 };            // Geofence 2
  geofences[2] = { 15.882490, 74.516010, 25.0 };          // Geofence 3

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

void loop() {
  smartDelay(2000);

  if (millis() > 5000 && gps.charsProcessed() < 10)
    Serial.println(F("No GPS data received: check wiring"));

  unsigned long currentMillis = millis();

  if ((unsigned long)(currentMillis - previousMillis) >= interval) {
    send_gps_data();
    previousMillis = currentMillis;
  }
}

static void smartDelay(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (ss.available())
      gps.encode(ss.read());
  } while (millis() - start < ms);
}




void send_gps_data() {
  if (gps.location.lat() == 0 || gps.location.lng() == 0) {
    Serial.println("No GPS data available");
    return;
  }

  float currentLat = gps.location.lat();
  float currentLng = gps.location.lng();
  Serial.print("Latitude (deg): ");
  Serial.println(currentLat, 6);

  Serial.print("Longitude (deg): ");
  Serial.println(currentLng, 6);

  for (int i = 0; i < MAX_GEOFENCES; i++) {

    checkGeofence(geofences[i], currentLat, currentLng);
  }
}



void checkGeofence(Geofence fence, float currentLat, float currentLng) {
  float distance = getDistance(currentLat, currentLng, fence.latitude, fence.longitude);

  if (distance < fence.maxDistance) {
    digitalWrite(buzzerPin, HIGH);
    Serial.print("Entered geofence ");
    Serial.println(i + 1);  // Print the geofence number

    delay(5000);

    digitalWrite(buzzerPin, LOW);
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


