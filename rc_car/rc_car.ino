#include <AFMotor.h> 
AF_DCMotor m1(1,MOTOR12_64KHZ);
AF_DCMotor m2(2,MOTOR12_64KHZ);
AF_DCMotor m3(3,MOTOR34_64KHZ);
AF_DCMotor m4(4,MOTOR34_64KHZ);

void setup()
{
Serial.begin(9600);
pinMode(0, INPUT);
pinMode(13, INPUT);
pinMode(2, INPUT);

int Delay = 500;
int fast = 255;
int slow = 100;

m1.setSpeed(fast);
m2.setSpeed(fast);
m3.setSpeed(fast);
m4.setSpeed(fast);
/*
m1.run(FORWARD);
m2.run(FORWARD);
m3.run(FORWARD);
m4.run(FORWARD);
Serial.println("Forward!!");
delay(Delay);
m1.run(BACKWARD);
m2.run(BACKWARD);
m3.run(BACKWARD);
m4.run(BACKWARD);
Serial.println("Backward!!");
delay(Delay);
m1.run(RELEASE);
m2.run(RELEASE);
m3.run(RELEASE);
m4.run(RELEASE);
Serial.println("Release");
delay(Delay);
*/
}

void loop()
{
  Serial.print(digitalRead(0));
  Serial.print(digitalRead(13));
  Serial.print(digitalRead(2));

  if(digitalRead(0) == LOW && digitalRead(13) == LOW && digitalRead(2) == HIGH)
  {
    Serial.println("FORWARD");
    m1.run(FORWARD);
    m2.run(FORWARD);
    m3.run(FORWARD);
    m4.run(FORWARD);
  }
  else if(digitalRead(0) == LOW && digitalRead(13) == LOW && digitalRead(2) == LOW)
  {
    Serial.println("BACKWARD");
    m1.run(BACKWARD);
    m2.run(BACKWARD);
    m3.run(BACKWARD);
    m4.run(BACKWARD);
  }
  else if(digitalRead(0) == HIGH && digitalRead(13) == LOW && digitalRead(2) == HIGH)
  {
    Serial.println("RIGHT");
    m1.run(FORWARD);
    m2.run(FORWARD);
    m3.run(BACKWARD);
    m4.run(BACKWARD);
  }
  else if(digitalRead(0) == HIGH && digitalRead(13) == LOW && digitalRead(2) == LOW)
  {
    Serial.println("LEFT");
    m1.run(BACKWARD);
    m2.run(BACKWARD);
    m3.run(FORWARD);
    m4.run(FORWARD);
  }
  else if(digitalRead(0) == HIGH && digitalRead(13) == HIGH && digitalRead(2) == HIGH)
  {
    Serial.println("Release");
    m1.run(RELEASE);
    m2.run(RELEASE);
    m3.run(RELEASE);
    m4.run(RELEASE);
  }
}