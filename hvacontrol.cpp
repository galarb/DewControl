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
int setdelta = 5, maxdelta = 50; //default 5 above the calculated dp
static int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3;//sensors pins
bool togsw = false;//button flag
bool result; //button variable
bool mode; //heating 1, cooling 0
float sp = 25; //setpoint for heating
bool direction = 1;

Adafruit_ST7789 tft = Adafruit_ST7789(9, 8, 7);//CS, dc(MISO), MOSI, SCK
//9(CS), 11(COPI), 12(CIPO), 13(SCK)
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
 

hvacontrol::hvacontrol(int encoderPinA, int encoderPinB, int valvecontrolPin) {
  _encoderPinA = encoderPinA;
  _encoderPinB = encoderPinB;
  _valvecontrolPin = valvecontrolPin;
  _onofsw  = false;

  pinMode(_encoderPinA, INPUT); 
  pinMode(_encoderPinB, INPUT); 
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
  tft.fillScreen(ST77XX_RED);
  tft.setRotation(1);     //to 90 deg
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  sdbegin();  
  tftwelcome(); //welcome image for 1 min
  devmodebutton.begin();//required for button IRQ
  Serial.println("Setup finished");
  checkmode();
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
  if(getevaldir()){//heating mode
    mode = 1;//heating
  }
  else {
    mode = 0;// cooling
  }
 // if(digitalRead(2)){while(1);}
  Serial.print("mode  = "); Serial.println(mode);
  return mode;
}
double hvacontrol::PIDcalc(double inp, int sp){
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
  if(mode == 1){//heating
    float pipetempPV = getwatertemp();//a number between 0-50
    float pipetempSP = setpipetempheat();//default dp+5, otherwise between dp and 50
    setValve(PIDcalc(pipetempPV, pipetempSP));//expexts values between 0..100
    //tftrun();
    if(checkButton()){
      //tftdatashow(getvalvestat(), getairtemp(), getRH(), getwatertemp());
     // Serial.println("Datashow");
     }
    else{
      //tftopershow(getdew_point(), setpipetemp());
      //Serial.println("opershow");
    }
    selftest();
  }
  //File entry = SD.open("225termi.bmp");  // open SD card main root
    //bmpDraw("225termi.bmp", 0, 0);   // draw it 
    //delay(1500);

  File entry = SD.open("225termi.bmp");   
    bmpDraw("225termi.bmp", 0, 0);
  entry.close();  // close the file
  delay(1500);
}

float hvacontrol::setpipetempcool(){ // returns the setpoint pipe temp
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
  //ShowInfoLcd(tempdpreading, setdelta);
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
  }
  return togsw;
}

float hvacontrol::getdew_point(){
  float dewpoint = getairtemp() - ((100 - getRH()) / 5);
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
  int valveposition = map(valve, 0, 100, 0, 254);
  analogWrite(_valvecontrolPin, valve);
  int valvestatus = getvalvestat();
  valvestatus = map(valvestatus, 0 , 1024, 0, 100);
  if (valvestatus < valve){//check if valve got to the new position
    return 0;
  }
  else return 1;
}
void hvacontrol::tftwelcome(){ 
  //a nice greeting screen
  File entry = SD.open("225termi.bmp");  // open SD card main root
   // bmpDraw("225termi.bmp", 0, 0);   // draw it    
  entry.close();  // close the file
  delay(1500);
}

void hvacontrol::tftopershow(float dp, float sp){ 
  //dp and sp values 0-50
  // Serial.println("Operation Mode");
  File entry = SD.open("oper.bmp");  // open SD card main root
  bmpDraw(entry.name(), 0, 0);   // draw it
  entry.close();  // close the file
}
void hvacontrol::tftdatashow(float valve, float airtemp, float RH, float pipetemp){
  //valve and RH in %, pipetemp and airtemp 0-50
   //Serial.println("Maintenance Mode");
  File entry = SD.open("data.bmp");  // open SD card main root
  bmpDraw(entry.name(), 0, 0);   // draw it
  entry.close();  // close the file
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
void hvacontrol::selftest(){
  //check all inputs for failure. 
  //calls fault(x)//1 - valve, 2 - temp, 3 - RH, 4 - airTemp
}
void hvacontrol::fault(int x){
  switch (x) {
    case '1': //Set port to HIGH
    //error message 1 , valve sensor disconnected
    break;
    case '2': //Set port to HIGH
    //error message 2 , sensor temp
    break;
      //and so on
  }    
}