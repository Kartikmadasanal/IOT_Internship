int brightness = LED_BUILTIN;

void setup()
{
 
}
void loop()
{
  for (brightness = 0; brightness <= 100; brightness++5) {
    analogWrite(2, brightness);
    delay(300); // Wait for 30 millisecond(s)
  }
  for (brightness = 100; brightness >= 0; brightness--5) {
    analogWrite(2, brightness);
    delay(30); // Wait for 30 millisecond(s)
  }
}