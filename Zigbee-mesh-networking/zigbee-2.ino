#include "SoftwareSerial.h"

int i;
char received[10];

SoftwareSerial XBee(9,8);

void setup()

{

  Serial.begin(9600);

  XBee.begin(9600);
  Serial.println("started!");
}

void loop(){

  Serial.println("sending...1");
  XBee.write('1');
  delay(1000);

  Serial.println("sending...2");
  XBee.write('0');
  delay(1000);

  if(XBee.available() > 0){
    Serial.println("trying to recieve data!");
    char received = XBee.read();
    Serial.println(received);
  }
  else{
  Serial.println("not working");
  }
}


