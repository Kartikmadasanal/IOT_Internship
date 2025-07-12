#include <TinyGPSPlus.h>
#include <SoftwareSerial.h>

static const int RXPin = 6, TXPin = 5;
String s = "kartik_test";
static const int buzzerPin = 7;
unsigned long interval = 10000;
static const uint32_t GPSBaud = 9600;
unsigned long previousMillis = 0;

TinyGPSPlus gps;  // The TinyGPSPlus object
SoftwareSerial ss(RXPin, TXPin);  // The serial connection to the GPS device

void setup()
{
    pinMode(buzzerPin, OUTPUT);
  Serial.begin(9600);
  ss.begin(GPSBaud);

  Serial.println("Starting...");
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




// sending gps 

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
    Serial.println(currentLat , 6);



  Serial.print("Longitude (deg): ");
    Serial.println(currentLng ,6);



if (currentLng >= 74.513179 && currentLng <= 74.513378 )

  {
    // Turn on the buzzer
    digitalWrite(buzzerPin, HIGH);

    // Add a delay if you want the buzzer to stay on for a certain duration
    delay(5000);  // Adjust the delay as needed

    // Turn off the buzzer
    digitalWrite(buzzerPin, LOW);
    return;
  }
}
