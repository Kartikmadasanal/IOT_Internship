#define trigPin 7  // Trigger pin of the ultrasonic sensor connected to GPIO 4
#define echoPin 5  // Echo pin of the ultrasonic sensor connected to GPIO 5
int buzzer=6;
void setup() {
  Serial.begin(115200);
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  pinMode(buzzer, OUTPUT);
}

void loop() {
  // Send a 10us pulse to trigger the ultrasonic sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  // Read the echo pulse duration in microseconds
  long duration = pulseIn(echoPin, HIGH);

  // Calculate the distance in centimeters
  // (speed of sound is approximately 343 meters per second)
  float distance_cm = (duration / 2) * 0.0343;
  if(distance_cm<=10)
  {
    digitalWrite(buzzer, HIGH);
  }
  else
  {
    digitalWrite(buzzer, LOW);
  }
  Serial.print("Distance: ");
  Serial.print(distance_cm);
  Serial.println(" cm");

  delay(1000);  // Wait for a moment before the next reading
}