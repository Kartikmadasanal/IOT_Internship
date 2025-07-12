#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>


#define MAX_GEOFENCES 6

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
TinyGPSPlus gps;
SoftwareSerial ss(RXPin, TXPin);

void setup() {
  pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
  ss.begin(9600);

  Serial.println("Starting...");

  // Initialize geofences (add as many as needed)
  geofences[0] = {15.824935, 74.489391, 100};    // college
  geofences[1] = {15.859334, 74.507958, 100};    // bogarwas
  geofences[2] = {15.867530, 74.511593, 100};    // channamma
  geofences[3] = {15.866505, 74.518674, 100};    // rto
  geofences[4] = {15.862881, 74.523916, 100};    // cbt
  geofences[5] = {15.874742, 74.530102, 100};    // mahantesh nagar

  ss.println("\r");
  ss.println("AT\r");
  delay(5000);

  ss.println("\r");
  ss.println("AT+GPS=1\r");
  delay(3000);
  ss.println("AT+CREG=2\r");
  delay(3000);
  ss.println("AT+CGATT=1\r");
  delay(3000);
  ss.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r");
  delay(3000);
  ss.println("AT+CGACT=1,1\r");
  delay(6000);
  ss.println("\r");
  ss.println("AT+GPSRD=1\r");
  delay(100);

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
    checkGeofence(geofences[i], currentLat, currentLng, i + 1); // Pass the corrected geofence number
  }
}

void checkGeofence(Geofence fence, float currentLat, float currentLng, int fenceNumber) {
  float distance = getDistance(currentLat, currentLng, fence.latitude, fence.longitude);

  if (distance < fence.maxDistance) {
    digitalWrite(buzzerPin, HIGH);
    Serial.print("Entered geofence ");
    Serial.println(fenceNumber);
    // delay(2000);
  } else {
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
  dist_calc *= 6371000.0; // Converting to meters
  return dist_calc;
}
