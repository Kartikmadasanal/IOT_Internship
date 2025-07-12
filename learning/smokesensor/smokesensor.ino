const int mq135Pin = 9;  // Analog pin to which the MQ-135 sensor is connected

void setup() {
  Serial.begin(9600);  // Initialize serial communication
}

void loop() {
  int sensorValue = analogRead(mq135Pin);  // Read the analog value from the sensor
  float voltage = (sensorValue / 1024.0) * 5.0;  // Convert the sensor value to voltage
  float ppm = (voltage - 0.25) / 0.05;  // Convert the voltage to parts per million (PPM)

  Serial.print("CO2 Concentration: ");
  Serial.print(ppm);
  Serial.println(" ppm");

  delay(1000);  // Delay for a second (adjust as needed)
}
