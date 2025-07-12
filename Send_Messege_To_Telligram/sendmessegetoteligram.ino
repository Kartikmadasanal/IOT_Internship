#include <SoftwareSerial.h>
#include <TinyGPS++.h>
// Define RX and TX pins
static const int RXPin = 2;  // RX pin for SoftwareSerial (D2)
static const int TXPin = 3;  // TX pin for SoftwareSerial (D3)

SoftwareSerial ss(RXPin, TXPin);  // Create SoftwareSerial object

TinyGPSPlus gps;

// Variables to store data dynamically
float Latitude = 0;
float Longitude = 0;

void setup() {
  pinMode(8, INPUT);  // Assuming the button is connected to pin 8

  Serial.begin(9600);  // Start the Serial Monitor for debugging
  ss.begin(9600);      // Start the SoftwareSerial for GSM communication

  Serial.println("Starting...");

  // GSM Initialization and connection setup
  ss.println("\r");
  ss.println("AT\r");  // Check communication with GSM module
  delay(5000);
  // Wait for 5 seconds to ensure GSM module is ready
  ss.println("AT+GPS=1\r");  // Check signal quality (CSQ)
  delay(2000);
  readGSMResponse();           // Read and print the GSM response for signal quality
  ss.println("AT+GPSRD=2\r");  // Check GPRS registration status
  delay(2000);
  readGSMResponse();  // Read and print the GSM response for signal quality

  ss.println("AT+CSQ\r");  // Check signal quality (CSQ)
  delay(2000);
  readGSMResponse();  // Read and print the GSM response for signal quality

  ss.println("AT+CGREG?\r");  // Check GPRS registration status
  delay(3000);
  readGSMResponse();  // Read and print the GSM response for registration status

  ss.println("AT+CGATT=1\r");  // Attach to GPRS service
  delay(5000);
  readGSMResponse();  // Check response for GPRS attach

  ss.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r");  // Set APN
  delay(3000);
  readGSMResponse();  // Read the GSM response

  ss.println("AT+CGACT=1,1\r");  // Activate GPRS
  delay(5000);
  readGSMResponse();  // Read the response after activating GPRS
  delay(8000);

  Serial.println("GSM Setup Executed");
}

void loop() {



  boolean newData = true;
  for (unsigned long start = millis(); millis() - start < 2000;) {
    while (ss.available()) {
      if (gps.encode(ss.read())) {
        newData = true;
      }
    }
  }
  if (newData) {

    Serial.print("Latitude= ");
    Serial.print(gps.location.lat(), 6);
    Latitude = gps.location.lat();
    Serial.print(" Longitude= ");
    Serial.println(gps.location.lng(), 6);
    Longitude = gps.location.lng();
    newData = true;
    delay(300);
  }



  int buttonState = digitalRead(8);  // Read the button state

  if (buttonState == HIGH) {



    // Create the URL with dynamic variables
    String url = "https://api.telegram.org/bot7783621581:AAGc-KnO6_W9RZrZhEGjk8BCAZPQrXRYBNc/sendMessage?chat_id=-4558240424&text=Latitude:";
    url += Latitude;  // Add Latitude value
    url += "longitude:";
    url += Longitude;

    // Print the generated URL for debugging
    Serial.println("Sending request: " + url);

    // Send the HTTP POST request to Telegram bot API with dynamic URL
    ss.println("AT+HTTPPOST=\"" + url + "\",\"application/x-www-form-urlencoded\",\"\"\r");

    delay(5000);        // Wait for the GSM module to process the request
    readGSMResponse();  // Print response from GSM module after sending HTTP POST

    Serial.println("Message Sent");  // Print to Serial Monitor for debugging





    delay(10000);  // Wait to prevent multiple messages on a single button press
  }
}








// Function to read and print the response from the GSM module
void readGSMResponse() {
  while (ss.available()) {
    char c = ss.read();  // Read the response character by character
    Serial.write(c);     // Print the response to the Serial Monitor
  }
}























































#include <SoftwareSerial.h>
#include <TinyGPS++.h>


TinyGPSPlus gps;
// Define RX and TX pins
#define RXPin 2   // RX pin for SoftwareSerial (D2)
#define TXPin 3   // TX pin for SoftwareSerial (D3)
#define GPS_RX 4  // Connect Neo-6M TX to Arduino Uno pin 4
#define GPS_TX 5
SoftwareSerial ss(RXPin, TXPin); 
SoftwareSerial neo6m(5, 6);
// Variables to store data dynamically
float lastLatitude = 15.895155;
float lastLongitude = 74.509292;
String Device_Id;                // This will hold the SIM ICCID
const float minMovement = 50.0;  // Minimum distance to trigger an update (50 meters)

void setup() {

  Serial.begin(9600);   // Start the Serial Monitor for debugging
  ss.begin(9600);     // Start the SoftwareSerial for GSM communication
 
  Serial.println("GPS and GSM tracking initialized.");
  initializeGSM();
  // GSM Initialization and connection setup
}

void loop() {
  smartdelay_gps(1000);

    
  if (gps.location.isValid())  {
    float currentLatitude = gps.location.lat();
    float currentLongitude = gps.location.lng();
    Serial.print(currentLatitude);

      sendLocationToServer(currentLatitude, currentLongitude);


    // Calculate distance from the last sent location
    // float distanceMoved = TinyGPSPlus::distanceBetween(lastLatitude, lastLongitude, currentLatitude, currentLongitude);

    // Serial.print("Current Distance Moved: ");
    // Serial.println(distanceMoved);
    // if (distanceMoved >= minMovement) {
    //   sendLocationToServer(currentLatitude, currentLongitude);

    //   // Update last sent location
    //   lastLatitude = currentLatitude;
    //   lastLongitude = currentLongitude;
    // }
    // Send data if moved more than 50 meters
  }
  sendLocationToServer(lastLatitude, lastLongitude);
  delay(10000);
}


// Function to read and print the response from the GSM module







void sendLocationToServer(float latitude, float longitude) {

  String url = "http://steadfast-band-kettle.glitch.me/sendCoordinets?Latitude=";
  url += latitude;  // Add Latitude value
  url += "&longitude=";
  url += longitude;  // Add Longitude value
  url += "&Device_Id=";
  url += Device_Id;  // Add Device_Id value (ICCID)

  // Print the generated URL for debugging
  Serial.println("Sending request: " + url);

  // Send the HTTP POST request to Telegram bot API with dynamic URL
  ss.println("AT+HTTPPOST=\"" + url + "\",\"application/x-www-form-urlencoded\",\"\"\r");

  delay(5000);  // Wait for the GSM module to process the request
  while (ss.available()) {
    char c = ss.read();  // Read the response character by character
    Serial.write(c);     // Print the response to the Serial Monitor
  }
  Serial.println("Message Sent");  // Print to Serial Monitor for debugging
}









static void smartdelay_gps(unsigned long ms) {
  unsigned long start = millis();
  do {
    while (neo6m.available())
      gps.encode(neo6m.read());
  } while (millis() - start < ms);
}





void initializeGSM() {
  ss.println("\r");
  ss.println("AT\r");  // Check communication with GSM module
  delay(5000);         // Wait for 5 seconds to ensure GSM module is ready

  ss.println("AT+CSQ\r");  // Check signal quality (CSQ)
  delay(2000);
  readGSMResponse();  // Read and print the GSM response for signal quality

  ss.println("AT+CGREG?\r");  // Check GPRS registration status
  delay(3000);
  readGSMResponse();  // Read and print the GSM response for registration status

  ss.println("AT+CGATT=1\r");  // Attach to GPRS service
  delay(5000);
  readGSMResponse();  // Check response for GPRS attach

  ss.println("AT+CGDCONT=1,\"IP\",\"airtelgprs.com\"\r");  // Set APN
  delay(3000);
  readGSMResponse();  // Read the GSM response

  ss.println("AT+CGACT=1,1\r");  // Activate GPRS
  delay(6000);
  readGSMResponse();  // Read the response after activating GPRS

  // Get SIM ICCID
  getICCID();

  Serial.println("GSM Setup Executed");
}

void readGSMResponse() {
  while (ss.available()) {
    char c = ss.read();  // Read the response character by character
    Serial.write(c);     // Print the response to the Serial Monitor
  }
}
// Function to get ICCID (SIM card ID)
void getICCID() {
  ss.println("AT+CCID\r");  // Command to get SIM ICCID
  delay(2000);
  String iccid = "";  // Temporary string to hold ICCID value

  while (ss.available()) {
    char c = ss.read();          // Read the ICCID response
    if (c >= '0' && c <= '9') {  // Only capture numeric characters
      iccid += c;                // Store only numeric characters
    }
  }

  Device_Id = iccid;                                  // Assign the cleaned ICCID to Device_Id
  Serial.println("Device ID (ICCID): " + Device_Id);  // Print ICCID for debugging
}
