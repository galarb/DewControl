#include "HardwareSerial.h"
#include "ButtonIRQ.h"
#include <Arduino.h>

volatile bool flag = false;
     
void buttonPressed(){
  flag = !flag;
  delay(100);   
}

ButtonIRQ::ButtonIRQ(int Button) {
  _button = Button;
  pinMode(_button, INPUT_PULLUP); 
   //Using Interupt to check when there is a chage from obstruction to hole in the encoder
  attachInterrupt(digitalPinToInterrupt(_button), buttonPressed, FALLING); //on falling edge

}

void ButtonIRQ::begin() {
  delay(30);
  Serial.println("Button IRQ systemm Started");
  Serial.print("Button pin = ");  
  Serial.println(_button);
  Serial.println("Button IRQ system Setup finished");
}

bool ButtonIRQ::isTrue() {
   if (flag) {
    flag = !flag; 
    return true;
  }
  else{return false;}
}
