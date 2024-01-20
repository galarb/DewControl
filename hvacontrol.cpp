#include "pins_arduino.h"
#include "hvacontrol.h"
#include "WString.h"
#include "Stream.h"
#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_NeoPixel.h>
#include <LiquidCrystal_I2C.h>
#include <Adafruit_NeoPixel.h>
#include <SoftwareSerial.h>

int aState, aLastState;
bool devMode = false;
int deg = 0, max = 100;
int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3;
 
LiquidCrystal_I2C lcd(0x27,16,2);

hvacontrol::hvacontrol(int encoderPinA, int encoderPinB, int buttonClick) {
  _encoderPinA = encoderPinA;
  _encoderPinB = encoderPinB;
  _buttonClick = buttonClick;
  _onofsw  = false;

  pinMode(_encoderPinA, INPUT); 
  pinMode(_encoderPinB, INPUT); 
  pinMode (_buttonClick, INPUT); //pinout encoder button


}

void hvacontrol::begin(double bdrate) {
  Serial.begin(bdrate);      
  delay(30);
  Serial.println("Started");
  Serial.print("encoderPinA=");
  Serial.println(_encoderPinA);
  Serial.print("encoderPinB = ");  
  Serial.println(_encoderPinB);
  
  /*lcd.init();  
  lcd.backlight();
  lcd.setCursor(1,0);
  lcd.print("HVAC CONTROL");
  lcd.setCursor(1, 1);
  lcd.print("Setup finished");
  lcd.setBacklight(0);*/
  aLastState = digitalRead(_encoderPinA); //setup the last var of encoder 

  Serial.println("Setup finished");
}

double hvacontrol::PIDcalc(double inp, int sp){
  //  Serial.println(steps);

   currentTime = millis();                //get current time
   elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
   //Serial.print(currentTime); //for serial plotter
   //Serial.println("\t"); //for serial plotter
   error = sp - inp;                                  // determine error
   cumError += error * elapsedTime;                   // compute integral
   rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
   if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator when Proportional is doing the work

   double out = kp*error + ki*cumError + kd*rateError; //PID output               

   lastError = error;                                 //remember current error
   previousTime = currentTime;                        //remember current time
   if(out > 254){out = 254;}    //limit the function for smoother operation
   if(out < -254){out = -254;}
   if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
   return out;                                        //the function returns the PID output value 
  
}


void hvacontrol::lcdswitch(bool status){
  if(status){lcd.backlight();}
  else{lcd.setBacklight(0);}
}

void hvacontrol::ShowInfoLcd(int speed, int direction, int BTstatus){ 
  //lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("speed| dir | bat");
  lcd.setCursor(1,1);
  lcd.print(speed);  
  lcd.setCursor(4,1);
  lcd.print(" |  "); 
  lcd.setCursor(7,1);
  lcd.print(direction); 
  lcd.setCursor(10,1);
  lcd.print(" | ");  
  lcd.setCursor(13,1);

}

void hvacontrol::lcdenshow(int clicks, int output, int tempsteps){ 
 // lcd.setBacklight(1);
  lcd.clear();
  lcd.setCursor(0, 0);
   //        0123456789012345 
  lcd.print(" SP | PID |steps");
  lcd.setCursor(0,1);
  lcd.print(clicks);  
  lcd.setCursor(4,1);
  lcd.print("|"); 
  lcd.setCursor(6,1);
  lcd.print(output); 
  lcd.setCursor(10,1);
  lcd.print("|");  
  lcd.setCursor(12,1);
}

void hvacontrol::run(int kpp, int kii, int kdd){
  checkencoder();
  checkButton();
  Serial.println(getdew_point());
}
void hvacontrol::checkencoder(){
  aState = digitalRead(_encoderPinA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(_encoderPinB) != aState) { 
      if (deg < max){
       deg ++;
      }
     } else {
      if (deg > 0){
       deg --;
      }
     }
    
     Serial.print("Position: ");
     Serial.println(deg);
   } 
   
   aLastState = aState; // Updates the previous state of the outputA with the current state
  }
void hvacontrol::checkButton(){
  if (digitalRead(_buttonClick) == 0){
    devMode = !devMode;
    Serial.println("Button Clicked");

   }
}

float hvacontrol::getdew_point(){
  float airtemp = analogRead(AirTempPin);
  float rh = analogRead(RHPin);

  float dewpoint = airtemp - ((100 - rh) / 5);
  return dewpoint;
}
float hvacontrol::getdew_valvestat(){
  float valvestatus = analogRead(ValveStatusPin);
  return valvestatus;
}

