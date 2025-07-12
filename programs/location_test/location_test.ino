 void setup() {
  Serial.begin(9600);
  delay(1000);
  Serial.println("Initializing GSM module...");
  delay(1000);
  Serial.println("AT+GPS=1"); // Enable GPS
  delay(1000);
  // Serial.println("AT+CGNSSEQ=\"RMC\""); // Set NMEA sequence to RMC
  // delay(1000);
  Serial.println("GSM module initialized");
}

void loop() {
  // Fetch and print GPS location
   ();
  delay(5000); // Wait for 5 seconds before fetching the location again
}

void fetchLocation() {
  Serial.println("Requesting GPS location...");
  Serial.println("AT+GPSRD=10"); // Request GPS information
  delay(2000);

  // Read and extract GPS coordinates from the response
  // String response = "$GNRMC,000439.263,V,1549.3608,N,07429.3399,E,0.000,0.00,060180,,,N*52";
  while (Serial.available()) {
    char c = Serial.read();
    response += c;
  }

  // Check if the response contains valid GPS data
  if (response.indexOf("$GNRMC") != -1) {
    // Parse and print GPS coordinates
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
  // Example: $GNRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A
  int index1 = data.indexOf(',') + 1;
  int index2 = data.indexOf(',', index1) + 1;
  int index3 = data.indexOf(',', index2) + 1;
  int index4 = data.indexOf(',', index3) + 1;
  int index5 = data.indexOf(',', index4) + 1;
  int index6 = data.indexOf(',', index5) + 1;

  if (index1 > 0 && index2 > 0 && index3 > 0 && index4 > 0 && index5 > 0 && index6 > 0) {
    String latStr = data.substring(index3, index4 - 1);
    String lonStr = data.substring(index5, index6 - 1);

    latitude = latStr.toFloat();
    longitude = lonStr.toFloat();

    return true;
  }

  return false;
}
