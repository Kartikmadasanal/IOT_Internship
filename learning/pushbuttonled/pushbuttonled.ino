//Project Push Button Led

int led=1;
int button=2;

void setup() {
pinMode(led,OUTPUT);
pinMode(button,INPUT);

}

void loop() {
  if(digitalRead(button)==HIGH)
  {
    digitalWrite(led,HIGH);    
  }
  else
  {
    digitalWrite(led,LOW);
  }
  delay(3000);
}

