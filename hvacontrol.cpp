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

bool aState, aLastState; //encoder state variables
//bool devMode = false; //a flag to control graphics
float setdelta = 5, maxdelta = 50; //default 5 above the calculated dp
static int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3, PotenPin=A4;//sensors pins
bool togsw = false;//button flag
bool result; //button variable
bool mode; //heating 1, cooling 0
bool alarmAck = false; //true overides the default tft 
float sp = 25; //setpoint for heating
bool direction = 1;
float Vmin = 200; //part of 1024 of analog read.
Adafruit_ST7789 tft = Adafruit_ST7789(9, 8, 7);//CS, dc(MISO), MOSI, SCK
//9(CS), 11(COPI), 12(CIPO), 13(SCK)
//(int8_t cs, int8_t dc, int8_t rst);
/*reset 7 - blue(purple)
DC 8 - yellow(black)
CS 9 - green(black)
clk 13 - yellow
Mosi 11 - green
miso 12 - blue(gray)*/
ButtonIRQ devmodebutton(2); //initiate IRQ Button


void printDirectory(File dir, int numTabs) {
  while (true) {
    File entry =  dir.openNextFile();
    if (! entry) {
      // no more files
      break;
    }
    for (uint8_t i = 0; i < numTabs; i++) {
      Serial.print('\t');
    }
    Serial.print(entry.name());
    if (entry.isDirectory()) {
      Serial.println("/");
      printDirectory(entry, numTabs + 1);
    } else {
      // files have sizes, directories do not
      Serial.print("\t\t");
      Serial.println(entry.size(), DEC);
    }
    entry.close();
  }
}

uint16_t read16(File f) {
  uint16_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read(); // MSB
  return result;
}
 
uint32_t read32(File f) {
  uint32_t result;
  ((uint8_t *)&result)[0] = f.read(); // LSB
  ((uint8_t *)&result)[1] = f.read();
  ((uint8_t *)&result)[2] = f.read();
  ((uint8_t *)&result)[3] = f.read(); // MSB
  return result;
}

#define BUFFPIXEL 20
void bmpDraw(char *filename, uint8_t x, uint16_t y) {
  File     bmpFile;
  int      bmpWidth, bmpHeight;   // W+H in pixels
  uint8_t  bmpDepth;              // Bit depth (currently must be 24)
  uint32_t bmpImageoffset;        // Start of image data in file
  uint32_t rowSize;               // Not always = bmpWidth; may have padding
  uint8_t  sdbuffer[3*BUFFPIXEL]; // pixel buffer (R+G+B per pixel)
  uint8_t  buffidx = sizeof(sdbuffer); // Current position in sdbuffer
  boolean  goodBmp = false;       // Set to true on valid header parse
  boolean  flip    = true;        // BMP is stored bottom-to-top
  int      w, h, row, col;
  uint8_t  r, g, b;
  uint32_t pos = 0, startTime = millis();
 
  if((x >= tft.width()) || (y >= tft.height())) return;
 
  Serial.println();
  Serial.print(F("Loading image '"));
  Serial.print(filename);
  Serial.println('\'');
 
  // Open requested file on SD card
  if ((bmpFile = SD.open(filename)) == NULL) {
    Serial.print(F("File not found"));
    return;
  }
 
  // Parse BMP header
  if(read16(bmpFile) == 0x4D42) { // BMP signature
    Serial.print(F("File size: ")); Serial.println(read32(bmpFile));
    (void)read32(bmpFile); // Read & ignore creator bytes
    bmpImageoffset = read32(bmpFile); // Start of image data
    Serial.print(F("Image Offset: ")); Serial.println(bmpImageoffset, DEC);
    // Read DIB header
    Serial.print(F("Header size: ")); Serial.println(read32(bmpFile));
    bmpWidth  = read32(bmpFile);
    bmpHeight = read32(bmpFile);
    if(read16(bmpFile) == 1) { // # planes -- must be '1'
      bmpDepth = read16(bmpFile); // bits per pixel
      Serial.print(F("Bit Depth: ")); Serial.println(bmpDepth);
      if((bmpDepth == 24) && (read32(bmpFile) == 0)) { // 0 = uncompressed
 
        goodBmp = true; //Supported BMP format -- proceed!
        Serial.print(F("Image size: "));
        Serial.print(bmpWidth);
        Serial.print('x');
        Serial.println(bmpHeight);
 
        // BMP rows are padded (if needed) to 4-byte boundary
        rowSize = (bmpWidth * 3 + 3) & ~3;
 
        // If bmpHeight is negative, image is in top-down order.
        // This is not canon but has been observed in the wild.
        if(bmpHeight < 0) {
          bmpHeight = -bmpHeight;
          flip      = false;
 }
 
        // Crop area to be loaded
        w = bmpWidth;
        h = bmpHeight;
        if((x+w-1) >= tft.width())  w = tft.width()  - x;
        if((y+h-1) >= tft.height()) h = tft.height() - y;
 
        // Set TFT address window to clipped image bounds
        tft.startWrite();
        tft.setAddrWindow(x, y, w, h);
 
        for (row=0; row<h; row++) { // For each scanline...
 
          // Seek to start of scan line.  It might seem labor-
          // intensive to be doing this on every line, but this
          // method covers a lot of gritty details like cropping
          // and scanline padding.  Also, the seek only takes
          // place if the file position actually needs to change
          // (avoids a lot of cluster math in SD library).
          if(flip) // Bitmap is stored bottom-to-top order (normal BMP)
            pos = bmpImageoffset + (bmpHeight - 1 - row) * rowSize;
          else     // Bitmap is stored top-to-bottom
            pos = bmpImageoffset + row * rowSize;
          if(bmpFile.position() != pos) { // Need seek?
            tft.endWrite();
            bmpFile.seek(pos);
            buffidx = sizeof(sdbuffer); // Force buffer reload
          }
 
          for (col=0; col<w; col++) { // For each pixel...
            // Time to read more pixel data?
            if (buffidx >= sizeof(sdbuffer)) { // Indeed
              bmpFile.read(sdbuffer, sizeof(sdbuffer));
              buffidx = 0; // Set index to beginning
              tft.startWrite();
            }
 
            // Convert pixel from BMP to TFT format, push to display
            b = sdbuffer[buffidx++];
            g = sdbuffer[buffidx++];
            r = sdbuffer[buffidx++];
            tft.pushColor(tft.color565(r,g,b));
          } // end pixel
        } // end scanline
        tft.endWrite();
        Serial.print(F("Loaded in "));
        Serial.print(millis() - startTime);
        Serial.println(" ms");
      } // end goodBmp
    }
  }
 bmpFile.close();
  if(!goodBmp) Serial.println(F("BMP format not recognized."));
}
// These read 16- and 32-bit types from the SD card file.
// BMP data is stored little-endian, Arduino is little-endian too.
// May need to reverse subscript order if porting elsewhere.
 

hvacontrol::hvacontrol(int encoderPinA, int encoderPinB, int valvecontrolPin, int alarmAckPin) {
  _encoderPinA = encoderPinA;
  _encoderPinB = encoderPinB;
  _valvecontrolPin = valvecontrolPin;
  _alarmAckPin = alarmAckPin;
  _onofsw  = false;

  pinMode(_encoderPinA, INPUT); 
  pinMode(_encoderPinB, INPUT); 
  pinMode(_alarmAckPin, INPUT_PULLUP);

}

void hvacontrol::begin(double bdrate) {
  Serial.begin(bdrate);      
  delay(30);
  Serial.println("###   HVAC CONTROL Started   ###");
  Serial.print("encoderPinA=");
  Serial.println(_encoderPinA);
  Serial.print("encoderPinB = ");  
  Serial.println(_encoderPinB);
  
  aLastState = digitalRead(_encoderPinA); //setup the last var of encoder

  tft.init(240, 320); 
  tft.setRotation(1);     //to 90 deg
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  //sdbegin();  
  tftwelcome(); //welcome image for 1 min
  checkmode();
  devmodebutton.begin();//required for button IRQ
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
  double t = 0;
  pinMode(2, INPUT_PULLUP);
  for (int c = 10; c > 0; c--){
    Serial.print(c);
    tft.setTextColor(ST77XX_MAGENTA);//...green 
    tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
    tft.setCursor(50, 5);
    tft.write("CHOOSE MODE");
    tft.setTextSize(3); //1 is default 6x8, 2 is 12x16, 3 is 18x24
    tft.setCursor(10, 120);
    tft.print(c);   
    tft.setCursor(80, 120);
    tft.write("Seconds left");
    
    Serial.println(" sec left!");
    if (mode == 1){ //hot mode
      Serial.println("Hot mode");
      tft.setTextColor(ST77XX_CYAN);//...red 
      tft.setCursor(40, 180);
      tft.write("Hot mode");
    }
    else{ //cold mode
      Serial.println("Cold mode");
      tft.setTextColor(ST77XX_YELLOW);//...blue
      tft.setCursor(40, 180);
      tft.write("Cold mode");
    }
    while (t < 1000){
      t++;
      delay(1);
      pinMode(2, INPUT_PULLUP);
      if (digitalRead(2) == 0){
        mode = !mode;
      }
      if(encoderchange()){t = 1000;}//skip the timer
    }
    t = 0;
    tft.fillScreen(ST77XX_WHITE);//..black
  }
  return mode;
}
double hvacontrol::PIDcalc(double inp, int sp){
  currentTime = millis();                //get current time
  elapsedTime = (double)(currentTime - previousTime)/1000; //compute time elapsed from previous computation (60ms approx). divide in 1000 to get in Sec
  //Serial.print(currentTime); //for serial plotter
  //Serial.println("\t"); //for serial plotter
  error = sp - inp;              // determine error
  cumError += error * elapsedTime;                   // compute integral
  rateError = (error - lastError)/elapsedTime;       // compute derivative deltaError/deltaTime
  if(rateError > 0.3 || rateError < -0.3){cumError = 0;}             // reset the Integral commulator when Proportional is doing the work

  double out = kp*error + ki*cumError + kd*rateError; //PID output               

  lastError = error;                                 //remember current error
  previousTime = currentTime;                        //remember current time
  if(out > 254){out = 254;}    //limit the function for smoother operation
  if(out < -254){out = -254;}
  if(cumError > 255 || cumError < -255){cumError = 0; out = 0;} // reset the Integral commulator
  //Serial.println(out);
  return out;    //the function returns the PID output value 
}

void hvacontrol::run(float kpp, float kii, float kdd){
  kp = kpp;
  ki = kii;
  kd = kdd;
  if(mode == 1){//heating
    
  }
  else { //cooling
    //if(encoderchange()){
      float pipetempPV = getwatertemp();//a number between 0-50
      float pipetempSP = setpipetempcool();//default dp+5, otherwise between dp and 50
      int ValveValue = map(PIDcalc(pipetempPV, pipetempSP), 0, 50, 0, 100);
      //Serial.print("pipetempPV = "); Serial.println(pipetempPV); 
      //Serial.print("pipetempSP = "); Serial.println(pipetempSP); //delay(2000);
      setValve(ValveValue);//expexts values between 0..100
      
   // } else {
     //       int ValveValue = map(PIDcalc(getwatertemp(), 15), 0, 50, 0, 100);
       //     setValve(ValveValue);
     //}
    if(checkButton()){
      selftest();
      tftdatashow(getvalvestat(), getairtemp(), getRH(), getwatertemp());
      }
      else{
        tftopershow(getdew_point(), setpipetempcool());
      }
  }
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
  result = devmodebutton.isTrue();
  if(result){
    togsw = !togsw;
    tft.fillScreen(ST77XX_WHITE);//...black

  }
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
  int valvecommand = map(valve, 0, 100, 0, 254);
  if(valvecommand > 254){valvecommand = 254;}
  analogWrite(_valvecontrolPin, valvecommand);
  //Serial.print("valve command = ");Serial.println(valvecommand); delay(2000);
  int valvestatus = getvalvestat();
  if (valvestatus < valve){//check if valve got to the new position
    return 0;
  }
  else return 1;
}
#define BMP_IMAGE_PATH "/225termi.bmp"

void hvacontrol::tftwelcome(){ 
  //a nice greeting screen
  //File entry = SD.open(BMP_IMAGE_PATH);  // open SD card main root
    //bmpDraw(BMP_IMAGE_PATH, 0, 0);   // draw it    
  ///entry.close();  // close the file
  //delay(2500);
  tft.fillScreen(ST77XX_WHITE);//...black

  tft.setTextColor(ST77XX_BLACK);
  tft.println("Welcome to HVAC control!");
  tft.println("powered by NTG Solutions");
  delay(1000);
  tft.fillScreen(ST77XX_WHITE);//...black
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
  tft.fillRoundRect(250, 38, 90, 18, 1, ST77XX_WHITE);
  tft.fillRoundRect(250, 84, 100, 18, 1, ST77XX_WHITE);
  tft.fillRoundRect(250, 129, 90, 18, 1, ST77XX_WHITE);
  tft.fillRoundRect(250, 175, 100, 18, 1, ST77XX_WHITE);
  
  tft.setCursor(250, 38);
  tft.print(valve);
  tft.setCursor(5, 83);
  tft.write("Air Temp");
  tft.setCursor(250, 83);
  tft.print(airtemp);
  tft.setCursor(5, 129);
  tft.write("RH");
  tft.setCursor(250, 129);
  tft.print(RH);
  tft.setCursor(5, 175);
  tft.write("Pipe Temp");
  tft.setCursor(250, 175);
  tft.print(pipetemp);

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
  tft.setCursor(220, 70);
  tft.fillRoundRect(220, 70, 90, 20, 1, ST77XX_WHITE);
  tft.fillRoundRect(220, 140, 100, 20, 1, ST77XX_WHITE);
  tft.print(sp);
  tft.setCursor(220, 140);
  tft.print(dp);
  
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
  if(!digitalRead(_alarmAckPin)){
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