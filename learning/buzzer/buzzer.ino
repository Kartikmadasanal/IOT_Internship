//BLINKIN

int buzzer=1;


void setup() {
pinMode(buzzer,OUTPUT);

}

void loop() {
digitalWrite(buzzer,HIGH);
delay(100);
digitalWrite(buzzer,LOW);
delay(100);
}

