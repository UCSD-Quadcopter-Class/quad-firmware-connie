#include <radio.h>


int motor1Pin = 3;

void setup() {
  Serial.begin(9600);
  rfBegin(12);
  // put your setup code here, to run once:
  pinMode(motor1Pin, OUTPUT);
  
}

void loop() {
  // put your main code here, to run repeatedly:
  

  if (rfAvailable())
  {
    unsigned int throttle = (unsigned int)(rfRead())*2;
    //Serial.println();
    //int temp = rfRead();
   // temp /= 6;
    //Serial.println(temp);
    analogWrite(motor1Pin, throttle);
  }
  

}
