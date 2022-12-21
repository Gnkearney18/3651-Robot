#include <Servo.h>
Servo myservo; 
void setup() {
  // put your setup code here, to run once:
  pinMode(40, INPUT);
  Serial.begin(115200);
  myservo.attach(3); 
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(digitalRead(40));
  if (digitalRead(40)){
    myservo.write(0);
  } else {
    myservo.write(180);
  }
  
}
