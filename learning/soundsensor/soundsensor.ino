int sound=8;
int buzzer=6;
void setup() {
  // put your setup code here, to run once:
pinMode(sound, INPUT);
pinMode(buzzer, OUTPUT);
}

void loop() {
  // put your main code here, to run repeatedly:
int data=digitalRead(sound);
if (data==1)
{
  digitalWrite(buzzer, HIGH);
  Serial.println("Obstacle");
}
else
{
  digitalWrite(buzzer, LOW);
  Serial.println("No Obstacle");
}
}
