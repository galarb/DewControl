#include "LiquidCrystal_I2C.h"
#include "Wire.h"

LiquidCrystal_I2C lcd(0x27,  16, 2);
//pinout
 #define outputA 5
 #define outputB 6
 #define buttonClick 7
//var selection
 int deg = 0; 
 int max = 100;
 int dew_point = 0;
 int aState;
 int aLastState;
 bool devMode = false;

 void setup() { 
   pinMode (outputA,INPUT); //pinout encoder
   pinMode (outputB,INPUT); //pinout encoder
   pinMode (buttonClick, INPUT); //pinout encoder button
   lcd.init(); //setup lcd
   lcd.backlight(); //turn on the light of the lcd
   lcd.setCursor(0, 0); //points lcd to start
   Serial.begin (115200); //starts the terminal
   aLastState = digitalRead(outputA); //setup the last var of encoder 
 } 

// int deg_check(int deg, int dew_point, int max) {
//   if (deg <= dew_point){
//     return dew_point;
//   }
//   else if (deg >= max){
//     return max;
//   }
//   else {
//     return deg;
//   }
// }

 void loop() { 
   aState = digitalRead(outputA); // Reads the "current" state of the outputA
   // If the previous and the current state of the outputA are different, that means a Pulse has occured
   if (aState != aLastState){     
     lcd.clear();
     // If the outputB state is different to the outputA state, that means the encoder is rotating clockwise
     if (digitalRead(outputB) != aState) { 
      if (deg < max){
       deg ++;
      }
     } else {
      if (deg > dew_point){
       deg --;
      }
     }
     lcd.print("Position: ");
     lcd.println(deg);
     Serial.print("Position: ");
     Serial.println(deg);
     Serial.println(devMode);
   } 
   if (digitalRead(buttonClick) == 0){
    devMode = !devMode;
   }
   aLastState = aState; // Updates the previous state of the outputA with the current state
 }
