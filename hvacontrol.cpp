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
#include <Adafruit_GFX.h>    // Core graphics library
#include <Adafruit_ST7789.h> // Hardware-specific library for ST7789
#include <SPI.h>
#include <SD.h>
#include "ButtonIRQ.h"

int aState, aLastState; //encoder state variables
bool devMode = false; //a flag to control graphics
int setdelta = 5, maxdelta = 50; //default 5 above the calculated dp
int ValveStatusPin=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3;//sensors pins
bool togsw = false;//button flag
bool result; //button variable
const int chipSelect = 10;//for SD Card

LiquidCrystal_I2C lcd(0x27,16,2);
Adafruit_ST7789 tft = Adafruit_ST7789(9, 12, 11, 13);//CS, dc(MISO), MOSI, SCK
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

  tft.init(240, 320, SPI_MODE2); 
  tft.fillScreen(ST77XX_RED);
  tft.setRotation(2);     //to 90 deg
  tft.setTextSize(2); //1 is default 6x8, 2 is 12x16, 3 is 18x24
  sdbegin();  
  tftwelcome(); //welcome image
  devmodebutton.begin();//required for button IRQ
  Serial.println("Setup finished");
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
  float pipetempPV = getwatertemp();//a number between 0-50
  float pipetempSP = setpipetemp();//default dp+5, otherwise between dp and 50
  setValve(PIDcalc(pipetempPV, pipetempSP));//expexts values between 0..100
  //tftrun();
  if(checkButton()){
    //tftdatashow(getvalvestat(), getairtemp(), getRH(), getwatertemp());
  }
  else{
    //tftopershow(getdew_point(), setpipetemp());
    }
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
  //ShowInfoLcd(tempdpreading, setdelta);
  return dpdelta;
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
  //a nice greeting screen}
 /* File entry = SD.open("/WLCM.BMP");  // open SD card main root
  printDirectory(entry, 0);
  uint8_t nameSize = String(entry.name()).length();  // get file name size
  String str1 = String(entry.name()).substring( nameSize - 4 );  // save the last 4 characters (file extension)
  if ( str1.equalsIgnoreCase(".bmp")){  // if the file has '.bmp' extension
      Serial.println("Drawing!");
      //bmpDraw(entry.name(), 0, 0);   // draw it
      } 
  if(entry.name()=="WLCM.BMP"){
    Serial.println("found welcome screen");
  }    
  entry.close();  // close the file
*/
  File root = SD.open("/");  // open SD card main root
  printDirectory(root, 0);   // print all files names and sizes
  root.close();              // close the opened root

  delay(500);

 // tft.drawRGBBitmap(0, 0, image_array1, 120, 120);
  
}
void hvacontrol::tftopershow(float dp, float sp){ 
//dp and sp values 0-50
// Serial.println("Operation Mode");
}
void hvacontrol::tftdatashow(float valve, float airtemp, float RH, float pipetemp){
  //valve and RH in %, pipetemp and airtemp 0-50
   //Serial.println("Maintenance Mode");

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
    Serial.println("* did you change the chipSelect pin to match your shield or module?");
    while (1);
  } else {
      Serial.println("Wiring is correct and a card is present.");
      delay(50); 
      if(SD.exists("wlcm.bmp")){
        Serial.println("wlcm.bmp EXISTS on SD card!!!");
        }

      File entry = SD.open("wlcm.bmp");  // open SD card 

      uint8_t nameSize = String(entry.name()).length();  // get file name size
      String str1 = String(entry.name()).substring( nameSize - 4 );  // save the last 4 characters (file extension)
      if (str1.equalsIgnoreCase(".bmp")){  // if the file has '.bmp' extension
          Serial.println("Drawing!");
         // bmpDraw(entry.name(), 0, 0);   // draw it
          } 
      entry.close();  // close the file
     }    
}
