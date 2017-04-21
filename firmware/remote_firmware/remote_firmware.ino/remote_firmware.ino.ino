#include <serLCD.h>

serLCD * lcd;


void setup() {
  // put your setup code here, to run once:
  Serial.begin(9600);
  lcd = new serLCD();

}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(analogRead(0));
  Serial.println(analogRead(1));
  Serial.println(analogRead(2));
  Serial.println(analogRead(3));

  lcd->clear();
  //lcd->print(analogRead(0)*2 - 220); //yaw
  lcd->print(analogRead(1)*2 - 214); //throttle
  /*lcd->print("|");
  lcd->print(analogRead(2));
  lcd->print("|");
  lcd->print(analogRead(3));
  lcd->print("|");*/
  delay(1000);

  

}
