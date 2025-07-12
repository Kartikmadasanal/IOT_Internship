#include <SoftwareSerial.h>

// Create a software serial port for the GSM module (D5 -> RX, D6 -> TX)
SoftwareSerial gsmSerial(D5, D6);

String response = "";

void setup() {
  Serial.begin(115200);        // Start serial communication with the serial monitor
  gsmSerial.begin(9600);       // Start communication with GSM module
  delay(1000);
  Serial.println("Initializing GSM module...");
  delay(1000);
  gsmSerial.println("AT+GPS=1"); // Enable GPS
  delay(1000);
  Serial.println("GSM module initialized");
}

void loop() {
  fetchLocation();
  delay(5000); // Wait for 5 seconds before fetching the location again
}

void fetchLocation() {
  Serial.println("Requesting GPS location...");
  gsmSerial.println("AT+GPSRD=10"); // Request GPS information
  delay(2000);

  response = "";  // Clear the previous response

  while (gsmSerial.available()) {
    char c = gsmSerial.read();
    response += c;
  }

  if (response.indexOf("$GNRMC") != -1) {
    float latitude, longitude;
    if (parseGPS(response, latitude, longitude)) {
      Serial.print("Latitude: ");
      Serial.println(latitude, 6); // Print latitude with 6 decimal places
      Serial.print("Longitude: ");
      Serial.println(longitude, 6); // Print longitude with 6 decimal places
    } else {
      Serial.println("Error parsing GPS data");
    }
  } else {
    Serial.println("No GPS data available");
  }
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

    latitude = convertToDecimalDegrees(latStr, data.charAt(index4));
    longitude = convertToDecimalDegrees(lonStr, data.charAt(index6));

    return true;
  }

  return false;
}

float convertToDecimalDegrees(String coordinate, char direction) {
  float decimalDegrees = coordinate.substring(0, 2).toFloat(); // Degrees
  float minutes = coordinate.substring(2).toFloat() / 60.0;     // Minutes to decimal

  decimalDegrees += minutes;

  if (direction == 'S' || direction == 'W') {
    decimalDegrees = -decimalDegrees; // Negative for South and West coordinates
  }

  return decimalDegrees;
}
