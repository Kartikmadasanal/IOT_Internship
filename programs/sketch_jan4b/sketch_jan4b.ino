#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 5;
unsigned long interval = 10000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;
      float latitude, longitude;

String s = "https://www.google.com/maps?q=";

const size_t BUFSIZE = 300;
char f_buffer[BUFSIZE];
float *f_buf = (float*)f_buffer;

SoftwareSerial ss(RXPin, TXPin);

void setup()
{
 Serial.begin(9600);
  ss.begin(GPSBaud);
  
  Serial.println("Starting...");
 

  ss.println("AT+GPS=1\r");

  delay(100);
  ss.println("AT+CREG=2\r");
  delay(6000);

  //ss.print("AT+CREG?\r");
  ss.println("AT+CGATT=1\r");
  delay(6000);

  // ss.println("AT+CGDCONT=1,\"IP\",\"WWW\"\r");
  // delay(6000);

  // ss.println("AT+LOCATION=1\r");
  ss.println("AT+CGACT=1,1\r");
  delay(6000);

  //Initialize ends
  //Initialize GPS

  //ss.println("AT+GPSMD=1\r");   // Change to only GPS mode from GPS+BDS, set to 2 to revert to default.


  // set SMS mode to text mode
  ss.println("AT+CMGF=1\r");
  delay(1000);

  //ss.println("AT+LOCATION=2\r");

  Serial.println("Setup Executed");
}

void loop()
{
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

      if (parseGPS(gpsData, latitude, longitude))
      {
        Serial.print("Latitude: ");
        Serial.println(latitude, 7);
        Serial.print("Longitude: ");
        Serial.println(longitude, 7);

        String googleMapsLink = "https://www.google.com/maps?q=" + String(latitude, 6) + "," + String(longitude, 6);
        Serial.println("Google Maps Link: " + googleMapsLink);
      }
      else
      {
        Serial.println("Error parsing GPS data");
      }
    }
  }







  Serial.print("Latitude (deg): ");
  Serial.println(latitude,6);

  Serial.print("Longitude (deg): ");
  Serial.println(longitude,6);



  s += String(latitude, 6);
  s += ",";
  s += String(longitude, 6);
  s += "/";

  Serial.println(s);


    Serial.println("Sending Message");

    ss.println("AT+CMGF=1\r");
    delay(1000);

    ss.println("AT+CNMI=2,2,0,0,0\r");
    delay(1000);

    ss.print("AT+CMGS=\"+918867975992\"\r");//Replace this with your mobile number
    delay(1000);
    ss.print(s);
    ss.write(0x1A);
    delay(1000);
    s = "https://www.google.com/maps?q=";
  }


bool parseGPS(String data, float &latitude, float &longitude)
{
  int index1 = data.indexOf(',') + 1;
  int index2 = data.indexOf(',', index1) + 1;
  int index3 = data.indexOf(',', index2) + 1;
  int index4 = data.indexOf(',', index3) + 1;
  int index5 = data.indexOf(',', index4) + 1;
  int index6 = data.indexOf(',', index5) + 1;

  if (index1 > 0 && index2 > 0 && index3 > 0 && index4 > 0 && index5 > 0 && index6 > 0)
  {
    String latStr = data.substring(index3, index4 - 1);
    String lonStr = data.substring(index5, index6 - 1);

    // Convert the format DDM M.MMMM to decimal degrees
    latitude = latStr.substring(0, 2).toFloat() + latStr.substring(2).toFloat() / 60.0;
    longitude = lonStr.substring(0, 3).toFloat() + lonStr.substring(3).toFloat() / 60.0;

    return true;
  }

  return false;
}





