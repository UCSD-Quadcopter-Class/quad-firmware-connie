#include <serLCD.h>
#include <radio.h>

serLCD * lcd;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  rfBegin(12);
  lcd = new serLCD();

}

void loop() {
  // put your main code here, to run repeatedly:
  //Serial.println(analogRead(0));
  //Serial.println(analogRead(1));
  //Serial.println(analogRead(2));
  //Serial.println(analogRead(3));

  lcd->clear();
  unsigned int pitch = analogRead(3)*2-260;
  unsigned int yaw = analogRead(0)*2 - 220;
  unsigned int roll = analogRead(2)*2-240;
  //lcd->print(analogRead(0)*2 - 220); //yaw
  unsigned int throttle = analogRead(1)*2 - 214; //throttle
  //lcd->print(analogRead(2)*2-240); //roll
  //lcd->print(pitch); //pitch
  rfWrite(char(throttle/12));
  /*lcd->print("|");
  lcd->print(analogRead(2));
  lcd->print("|");
  lcd->print(analogRead(3));
  lcd->print("|");*/


  

}
