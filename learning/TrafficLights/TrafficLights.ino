//Project Traffic lights

int led1=1;
int led2=2;
int led3=3;

void setup() {
pinMode(1,OUTPUT);
pinMode(2,OUTPUT);
pinMode(3,OUTPUT);
}

void loop() {
digitalWrite(led1,HIGH);
delay(1000);
digitalWrite(led1,LOW);
delay(1000);
digitalWrite(led2,HIGH);
delay(1000);
digitalWrite(led2,LOW);
delay(1000);
digitalWrite(led3,HIGH);
delay(1000);
digitalWrite(led3,LOW);
delay(1000);
}

