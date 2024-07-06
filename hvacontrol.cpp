#include <math.h>
#include "Adafruit_ST77xx.h"
#include "hvacontrol.h"
#include "HardwareSerial.h"
#include <Arduino.h>
#include <Wire.h>
#include <SoftwareSerial.h> 
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <SD.h>
#include "ButtonIRQ.h"
#include <TouchScreen.h> 

#define WHITE 0x0000
#define BLACK 0xFFFF
#define CYAN 0xF800
#define MAGENTA 0x07E0
#define YELLOW 0x001F
#define RED 0x07FF
#define GREEN 0xF81F
#define BLUE 0xFFE0
//#define X 0xFC00

bool aState, aLastState; //encoder state variables
//bool devMode = false; //a flag to control graphics
float setdelta = 5, maxdelta = 50; //default 5 above the calculated dp
static int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3, PotenPin=A4;//sensors pins
bool togsw = false;//button flag
bool stat; //button variable
bool laststat;//for button memory
bool mode; //heating 1, cooling 0
bool alarmAck = false; //true overides the default tft 
float sp = 25; //setpoint for heating
bool direction = 1;
float Vmin = 200; //part of 1024 of analog read.
float Last_sp, Last_dp, Last_valve, Last_airtemp, Last_RH, Last_pipetemp;
Adafruit_ST7789 tft = Adafruit_ST7789(10, 9, 8);//CS, dc(MISO), MOSI, SCK
//9(CS), 11(COPI), 12(CIPO), 13(SCK)
//(int8_t cs, int8_t dc, int8_t rst);
/*reset 7 - blue(purple)
DC 8 - yellow(black)
CS 9 - green(black)
clk 13 - yellow
Mosi 11 - green
miso 12 - blue(gray)*/
ButtonIRQ devmodebutton(2); //initiate IRQ Button



hvacontrol::hvacontrol(int encoderPinA, int encoderPinB, int valvecontrolPin, int alarmAckPin, int buttonPin) {
  _encoderPinA = encoderPinA;
  _encoderPinB = encoderPinB;
  _valvecontrolPin = valvecontrolPin;
  _alarmAckPin = alarmAckPin;
  _onofsw  = false;
  _ButtonPin = buttonPin;

  
}

void hvacontrol::begin(double bdrate) {
  Serial.begin(bdrate);      
  delay(30);
  Serial.println("###   HVAC CONTROL Started   ###");
  Serial.print("encoderPinA=");
  Serial.println(_encoderPinA);
  Serial.print("encoderPinB = ");  
  Serial.println(_encoderPinB);
  Serial.print("valvecontrolPin=");
  Serial.println(_valvecontrolPin);
  Serial.print("alarmAckPin = ");  
  Serial.println(_alarmAckPin);
  
  pinMode(_encoderPinA, INPUT); 
  pinMode(_encoderPinB, INPUT); 
  pinMode(_alarmAckPin, INPUT_PULLUP);
  pinMode(_valvecontrolPin, OUTPUT);
  pinMode(_ButtonPin, INPUT_PULLUP);
  aLastState = digitalRead(_encoderPinA); //setup the last var of encoder

  tft.init(240, 320); 
  tft.setRotation(1);     //to 90 deg
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  //sdbegin();  
  tftwelcome(); //welcome image for 1 min
  laststat = devmodebutton.isTrue();

  checkmode();
  Serial.println("Setup finished");
}

bool hvacontrol::encoderchange(){
  aState = digitalRead(_encoderPinA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){
    aLastState = aState;
    return true;} 
  else{  
    return false;} 
}

bool hvacontrol::getdir(){
  aState = digitalRead(_encoderPinA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(_encoderPinB) != aState) { //CW
       direction = 1;
     } 
     else { //CCW
       direction = 0;
     }
   } 
  aLastState = aState; // Updates the previous state of the outputA with the current state
  return direction;
}

bool hvacontrol::getevaldir(){//quitens down a spiky encoder
  bool tempdir = getdir();
  bool tempdir2 = getdir();
  bool tempdir3 = getdir();
  bool tempdir4 = getdir();
  while (tempdir == 1 && tempdir == tempdir2 && tempdir2 == tempdir3 and tempdir3 == tempdir4){
    return true;
  }
  while (tempdir == 0 && tempdir == tempdir2 && tempdir2 == tempdir3 and tempdir3 == tempdir4){
    return false;
  }
}

bool hvacontrol::checkmode(){
  double t = 0; //timer variable
  for (int c = 10; c > 0; c--){
    Serial.print(c);
    tft.setTextColor(GREEN);
    tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
    tft.setCursor(50, 5);
    tft.write("CHOOSE MODE");
    tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
    tft.setCursor(10, 120);
    tft.println(c);   
    tft.setCursor(80, 120);
    tft.write("Seconds left");
    
    Serial.println(" sec left!");
    Serial.println(mode);
    if (mode == 1){ //hot mode
      Serial.println("Hot mode");
      tft.setTextColor(RED);//...red 
      tft.setCursor(40, 180);
      tft.write("Hot mode");
    }
    else{ //cold mode
      Serial.println("Cold mode");
      tft.setTextColor(BLUE);//...blue
      tft.setCursor(40, 180);
      tft.write("Cold mode");
    }
    while (t < 1000){
      t++;
      pinMode(_ButtonPin, INPUT_PULLUP);
      delay(1);
      if (digitalRead(_ButtonPin) == 0){
        mode = !mode;
      }
      if(encoderchange()){

        t = 1000;}//skip the timer
    }
    t = 0;
    tft.fillScreen(BLACK);//..black
  }
  //Serial.print("mode ="); Serial.println(mode);

  return mode;
}
double hvacontrol::PIDcalc(double inp, int sp){
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
  // Serial.print("current time = ");Serial.println(currentTime); //for serial plotter
  //Serial.println("\t"); //for serial plotter
  error = inp - sp;              // determine error
  cumError += (error * elapsedTime);            // compute integral
  rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
  //Serial.print("rateError = "); Serial.println(rateError);
  //Serial.print("elapsed time = "); Serial.println(elapsedTime);
  //delay(1000);

  if(rateError < 0.3 || rateError > -0.3){cumError = 0;}// reset the Integral commulator
  //Serial.print("I = "); Serial.println(cumError);

  double out = kp*error + ki*cumError + kd*rateError; //PID output               

  lastError = error;                                 //remember current error
  previousTime = currentTime;                        //remember current time
  if(out > 254){out = 254;}    //limit the function for smoother operation
  if(out < -254){out = -254;}
  Serial.print("out value = "); Serial.println(out);
  return out;    //the function returns the PID output value 
}

void hvacontrol::run(float kpp, float kii, float kdd){
  kp = kpp;
  ki = kii;
  kd = kdd;
  if(mode == 1){//heating
    tft.fillScreen(GREEN);
    Serial.println("heating mode");
    delay(5000);
  }
  else { //cooling
    float pipetempPV = getwatertemp();//a number between 0-50
    float pipetempSP = setpipetempcool();//dp+potentiometer. up to +15
    int ValveValue = map(PIDcalc(pipetempPV, pipetempSP), 0, 50, 0, 100);
    //Serial.print("pipetempPV = "); Serial.println(pipetempPV); 
    //Serial.print("pipetempSP = "); Serial.println(pipetempSP); //delay(2000);
    setValve(ValveValue);//expexts values between 0..100
    
    if(checkButton()){
      //Serial.println("data show mode");
      //if (!selftest()){
        tftdatashow(getvalvestat(), getairtemp(), getRH(), getwatertemp());
        //Serial.println("Self test passed");
        //}
      }
    else{
      tftopershow(getdew_point(), setpipetempcool());
      //Serial.println("operator show mode");
    }
  }
  Last_dp = 0;
  Last_sp = 0;
  Last_valve = 1;
  Last_airtemp = 1;
  Last_RH = 1;
  Last_pipetemp = 1;
}

float hvacontrol::setpipetempcool(){ // returns the setpoint pipe temp
  /*aState = digitalRead(_encoderPinA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
  if (aState != aLastState){     
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(_encoderPinB) != aState) { 
      if (setdelta < maxdelta){
       setdelta = setdelta + 1;
      }
     } else {
        //if (setdelta > 0){
          setdelta = setdelta - 1;
       // }
     }
   } 
  aLastState = aState; // Updates the previous state of the outputA with the current state*/
  float tempdpreading = getdew_point();
  float poten = map(analogRead(PotenPin), 0, 1023, 0, 15);
  float dpdelta = tempdpreading + poten;
  //Serial.print("setdelta   = "); Serial.println(setdelta);//XX
  return dpdelta;
}

float hvacontrol::setpipetempheat(){ // returns the setpoint pipe temp
  if(getevaldir()){
    sp = sp + 0.5;
  }
  else{
    sp = sp - 0.5;
  }
  return sp;
}

bool hvacontrol::checkButton(){
  stat = devmodebutton.isTrue();
  //Serial.print("switch status = "); Serial.println(stat);
  //stat = digitalRead(_ButtonPin);
    //Serial.print("togsw = "); Serial.println(togsw);
    //Serial.print("laststat = "); Serial.println(laststat);

  if(laststat != stat){
    togsw = !togsw;
    tft.fillScreen(BLACK);//to clear between the screens

  }
  laststat = stat;
  return togsw;
}

float hvacontrol::getdew_point(){
  float dewpoint = getairtemp() - ((100 - getRH()) / 5);
  //Serial.print("dewpoint = ");Serial.println(dewpoint);//XX

  return dewpoint;
}

float hvacontrol::getvalvestat(){
  float valvestatus = analogRead(ValveStatusPin);
  valvestatus = map(valvestatus, 0, 1024, 0, 100);
  return valvestatus;
}

float hvacontrol::getwatertemp(){
  float watertemp = analogRead(WaterTempPin);
  watertemp = map(watertemp, 0 , 1024, 0, 50);//jumper S2 should be selected to 0-50 in 22UT-14
  return watertemp;
}

bool hvacontrol::setValve(int valve){//0..100
  Serial.println("*********new iteration********");
  Serial.print("valve argument = ");Serial.println(valve);
  int valvecommand = map(valve, 0, 100, 0, 254);
  if(valvecommand > 254){valvecommand = 254;}
  if(valvecommand < 0){valvecommand = 0;}
  analogWrite(_valvecontrolPin, valvecommand);
  Serial.print("valve command = ");Serial.println(valvecommand); delay(1000);
  int valvestatus = getvalvestat();
  if (valvestatus < valve){//check if valve got to the new position
    return 0;
  }
  else return 1;
}

void hvacontrol::tftwelcome(){ 
  //a nice greeting screen
  //File entry = SD.open(BMP_IMAGE_PATH);  // open SD card main root
    //bmpDraw(BMP_IMAGE_PATH, 0, 0);   // draw it    
  ///entry.close();  // close the file
  //delay(2500);
  tft.fillScreen(YELLOW);

  tft.setTextColor(BLACK);
  tft.println("Welcome to HVAC control!");
  tft.println("powered by NTG Solutions");
  delay(1000);
  tft.fillScreen(BLACK);//...black
}
void hvacontrol::tftdatashow(float valve, float airtemp, float RH, float pipetemp){
  tft.setCursor(50, 10);
  //tft.setTextColor(ST77XX_CYAN);//...red 
  //tft.setTextColor(ST77XX_MAGENTA);//...green
  tft.setTextColor(ST77XX_ORANGE);//... 
  if (mode){
    tft.setTextColor(ST77XX_CYAN); 
  }
  //...blue
  tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  tft.write("HVAC CONTROL");
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  tft.setTextColor(ST77XX_BLACK);//...white
  tft.setCursor(5, 38);
  tft.write("Valve Status");
  
  if (valve != Last_valve){
    tft.setCursor(250, 38);
    tft.fillRoundRect(250, 38, 90, 18, 1, ST77XX_WHITE);
    tft.print(valve);
    Last_valve = valve;
  }
  if (pipetemp != Last_pipetemp){
    tft.setCursor(250, 175);
    tft.fillRoundRect(250, 175, 100, 18, 1, ST77XX_WHITE);
    tft.print(pipetemp);
    Last_pipetemp = pipetemp;
  }
  if (airtemp != Last_airtemp){
    tft.setCursor(250, 83);
    tft.fillRoundRect(250, 84, 100, 18, 1, ST77XX_WHITE);
    tft.print(airtemp);
    Last_airtemp = airtemp;
  }
  if (RH != Last_RH){
    tft.setCursor(250, 129);
    tft.fillRoundRect(250, 129, 90, 18, 1, ST77XX_WHITE);
    tft.print(RH);
    Last_RH = RH;
  }
  tft.setCursor(5, 83);
  tft.write("Air Temp");
  tft.setCursor(5, 129);
  tft.write("RH");
  tft.setCursor(5, 175);
  tft.write("Pipe Temp");

  tft.drawFastHLine(0, 56, 310, ST77XX_BLACK);
  tft.drawFastHLine(0, 102, 310, ST77XX_BLACK);
  tft.drawFastHLine(0, 148, 310, ST77XX_BLACK);
  tft.drawFastHLine(0, 194, 310, ST77XX_BLACK);

  tft.drawFastVLine(180, 35, 200, ST77XX_BLACK);
}
void hvacontrol::tftopershow(float dp, float sp){
  tft.setCursor(50, 10);
  //tft.setTextColor(ST77XX_CYAN);//...red 
  //tft.setTextColor(ST77XX_MAGENTA);//...green 
  tft.setTextColor(ST77XX_ORANGE);//... 
  if (mode){
    tft.setTextColor(ST77XX_CYAN);
    
  }
 // tft.setTextColor(ST77XX_YELLOW);//...blue
  tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  tft.write("HVAC CONTROL");
  tft.setCursor(5, 70);
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  tft.setTextColor(ST77XX_BLACK);//...white
  tft.write("Set Point");
  tft.setCursor(5, 140);
  tft.write("Dew Point");
  if (sp != Last_sp){
    tft.setCursor(220, 70);
    tft.fillRoundRect(220, 70, 90, 20, 1, ST77XX_WHITE); 
    tft.print(sp);
    Last_sp = sp;
  }
  if (dp != Last_dp){
    tft.setCursor(220, 140);
    tft.fillRoundRect(220, 140, 100, 20, 1, ST77XX_WHITE);
    tft.print(dp);
    Last_dp = dp;
  }
  tft.drawFastHLine(0, 110, 310, ST77XX_BLACK);
  tft.drawFastVLine(180, 60, 100, ST77XX_BLACK);
  delay(200);
}

float hvacontrol::getairtemp(){
  float tempair = analogRead(AirTempPin);//verify correct settings of jumpers in 22UTH-13 (S4 - closed, S5 - open)
  return map(tempair, 0, 1024, 0, 50);
}

float hvacontrol::getRH(){
  float tempRH = analogRead(RHPin);
  return map(tempRH, 0, 1024, 0, 100);
}

void hvacontrol::sdbegin(){
  Serial.print("\nInitializing SD card...");

  // we'll use the initialization code from the utility libraries
  // since we're just testing if the card is working!
  if (!SD.begin(10)) {//CS pin
    Serial.println("initialization failed. Things to check:");
    Serial.println("* is a card inserted?");
    Serial.println("* is your wiring correct?");
    Serial.println("* did you change the chipSelect pin to match your module?");
    while (1);
  } else {
      Serial.println("Wiring is correct and a card is present.");
      delay(50); 
      }    
}
bool hvacontrol::selftest(){
  if (digitalRead(_alarmAckPin) == 0){
    return false;
  }
  //check all inputs for failure. 
  //calls fault(x)//1 - valve, 2 - temp, 3 - RH, 4 - airTemp
  float valvestat = analogRead(ValveStatusPin);
  float valvestat2 = analogRead(ValveStatusPin);
  if(valvestat  < Vmin &&  valvestat2 < Vmin){
    fault(1);
    return true;
  }
  float PipeTemp = analogRead(WaterTempPin);
  float PipeTemp2 = analogRead(WaterTempPin);
  if(PipeTemp  < Vmin &&  PipeTemp2 < Vmin){
    fault(2);
    return true;

  }
  float RH = analogRead(RHPin);
  float RH2 = analogRead(RHPin);
  if(RH  < Vmin &&  RH2 < Vmin){
    fault(3);
    return true;
  }
  float AirTemp = analogRead(AirTempPin);
  float AirTemp2 = analogRead(AirTempPin);
  if(AirTemp  < Vmin &&  AirTemp2 < Vmin){
    fault(4);
    return true;
  }  
  return false;
}
void hvacontrol::fault(int x){


  //Serial.print("x = "); Serial.println(x);

  switch (x) {
    case 1: //error message 1 , valve sensor disconnected
      Serial.println("valve status sensor disconnected!");
      tftfault(1);
    break;
    case 2: //error message 2 , sensor temp
      Serial.println("pipe temp sensor disconnected!");
      tftfault(2);
    break;
    case 3: //error message 3 - RH
      Serial.println("RH sensor disconnected!");
      tftfault(3);
    break;
    case 4: //error message 4 - airTemp
      Serial.println("air temp sensor disconnected!");
      tftfault(4);
    break;
  }
}

void hvacontrol::tftfault(int x){
  if(digitalRead(_alarmAckPin)){
    tft.fillRoundRect(0, 35, 320, 165, 1, ST77XX_WHITE);
    tft.setTextColor(ST77XX_CYAN);
    tft.setCursor(5, 160);
    tft.setTextSize(3);
    tft.print("FAULT ");
    tft.print(x);
    tft.print(" detected!");
    tft.setTextSize(2);
    tft.setCursor(15, 190);
    tft.print("Check Sensors Connection");
    tft.setCursor(90, 210);
    tft.print("and power");
    delay(1000);
    tft.fillRoundRect(70, 200, 200, 18, 1, ST77XX_WHITE);
    tft.fillRoundRect(20, 220, 300, 18, 1, ST77XX_WHITE);
  }
}