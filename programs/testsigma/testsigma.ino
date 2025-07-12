#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 5;
unsigned long interval = 10000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;
 
SoftwareSerial ss(RXPin, TXPin);

void setup()
{
  Serial.begin(9600);
  ss.begin(GPSBaud);
  
  Serial.println("Starting...");
      

}


void loop()
{
  // unsigned long currentMillis = millis();

  // if ((unsigned long)(currentMillis - previousMillis) >= interval)
  // {
  //   previousMillis = currentMillis;
  // }
    retrieve_gps_data();
}

void retrieve_gps_data()
{
   ss.println("AT+GPS=1");

  delay(2000);
  ss.println("AT+GPSRD=10");

  delay(11000); 
  while (ss.available())
  {
    String gpsData = ss.readStringUntil('\n');

    if (gpsData.startsWith("$GNGGA") || gpsData.startsWith("$GNRMC"))
    {
      Serial.println("Raw GPS Data: " + gpsData);

    
     
      float latitude, longitude;
    if (parseGPS(gpsData, latitude, longitude)) {
      Serial.print("Latitude: ");
      Serial.println(latitude, 7); 
      Serial.print("Longitude: ");
      Serial.println(longitude, 7); 
    } else {
      Serial.println("Error parsing GPS data");
    }
    String googleMapsLink = "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6);
        Serial.println("Google Maps Link: " + googleMapsLink);
  }}


}

bool parseGPS(String data, float &latitude, float &longitude) {
  int index1 = data.indexOf(',') + 1;
  int index2 = data.indexOf(',', index1) + 1;
  int index3 = data.indexOf(',', index2) + 1;
  int index4 = data.indexOf(',', index3) + 1;
  int index5 = data.indexOf(',', index4) + 1;
  int index6 = data.indexOf(',', index5) + 1;

  if (index1 > 0 && index2 > 0 && index3 > 0 && index4 > 0 && index5 > 0 && index6 > 0) {
    String latStr = data.substring(index3, index4 - 1);
    String lonStr = data.substring(index5, index6 - 1);

    // Convert the format DDM  M.MMMM to decimal degrees
    latitude = latStr.substring(0, 2).toFloat() + latStr.substring(2).toFloat() / 60.0;
    longitude = lonStr.substring(0, 3).toFloat() + lonStr.substring(3).toFloat() / 60.0;

    return true;
  }

  return false;
  
}