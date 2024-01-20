#include <avr/interrupt.h>
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
int setdelta = 5, maxdelta = 50;
int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3;

LiquidCrystal_I2C lcd(0x27,16,2);

hvacontrol::hvacontrol(int encoderPinA, int encoderPinB, int buttonClick, int valvecontrolPin) {
  _encoderPinA = encoderPinA;
  _encoderPinB = encoderPinB;
  _buttonClick = buttonClick;
  _valvecontrolPin = valvecontrolPin;
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


void hvacontrol::run(int kpp, int kii, int kdd){
  float pipetempPV = getwatertemp();//a number between 0-50
  float pipetempSP = setpipetemp();//default dp+5, otherwise between dp and 50
  setValve(PIDcalc(pipetempPV, pipetempSP));//expexts values between 0..100
  checkButton();
}

float hvacontrol::setpipetemp(){ // returns the setpoint pipe temp
  aState = digitalRead(_encoderPinA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(_encoderPinB) != aState) { 
      if (setdelta < maxdelta){
       setdelta = setdelta + 0.1;
      }
     } 
     else if (setdelta > 0){
       setdelta = setdelta - 0.1;
     }
   } 
  aLastState = aState; // Updates the previous state of the outputA with the current state
  float tempdpreading = getdew_point();
  float dpdelta = tempdpreading + setdelta;
  ShowInfoLcd(tempdpreading, setdelta);
  return dpdelta;
}

void hvacontrol::checkButton(){
  if (digitalRead(_buttonClick) == 0){
    devMode = !devMode;
    //Serial.println("Button Clicked");

   }
}

float hvacontrol::getdew_point(){
  float airtemp = analogRead(AirTempPin);
  airtemp = map(airtemp, 0 ,5, 0, 50); //verify correct settings of S1 jumper in 22UTH-13
  float rh = analogRead(RHPin);
  rh = map(rh, 0, 5, 0, 50);//verify correct settings of S1 jumper in 22UTH-13
  float dewpoint = airtemp - ((100 - rh) / 5);
  return dewpoint;
}
float hvacontrol::getvalvestat(){
  float valvestatus = analogRead(ValveStatusPin);
  return valvestatus;
}

float hvacontrol::getwatertemp(){
  float watertemp = analogRead(WaterTempPin);
  watertemp = map(watertemp, 0 , 5, 0, 50);//jumper S2 should be selected to 0-50 in 22UT-14
  return watertemp;
}

void hvacontrol::lcdswitch(bool status){
  if(status){lcd.backlight();}
  else{lcd.setBacklight(0);}
}

void hvacontrol::ShowInfoLcd(int dp, int setdp){ 
  //lcd.clear();
  lcd.setCursor(0, 0);
  //         0123456789012345
  lcd.print("DewPoint");
  lcd.setCursor(11,0);
  lcd.print(dp);  
  lcd.setCursor(1,1);
  lcd.print("Set for +"); 
  lcd.setCursor(12,1);
  lcd.print(setdp); 
  lcd.setCursor(10,1);
  
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

bool hvacontrol::setValve(int valve){//0..100
  int valveposition = map(valve, 0, 100, 0, 254);
  analogWrite(_valvecontrolPin, valve);
  int valvestatus = getvalvestat();
  valvestatus = map(valvestatus, 0 , 1024, 0, 100);
  if (valvestatus < valve){//check if valve got to the new position
    return 0;
  }
  return 1;
}