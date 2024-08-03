/*   Notes:
  Button on pin 2
  ValveStatus=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3, PotenPin=A4
  SPI Connection:
  ** MOSI - pin 11
  ** MISO - pin 12
  ** CLK - pin 13 
  tft CS - pin 10
  SD Card CS - pin Currently#10
*/

#include "clicli.h"
#include "hvacontrol.h"

hvacontrol hvac(
  3, //encoder A
  4, //encoder B
  5, //valve control
  6, //alarm Acknowlege
  2); //button pin

clicli mycli(hvac); //setup clicli tool for hvac control

void setup() { 
  hvac.begin(115200);

}

void loop() { 
  mycli.run();
  hvac.run(10, 0, 0); 
  /* a Kp of 1 will yield the following :
  for example a pipe temp of 30 and SP of 20, DP = 18.
   PID out will be 10, valve will be sent a 20 value
   the setvalve function will write a mapped value to 254, about 44
   so the max gain possible is about 20% off the valve capability (254)
   therefore a suggested value for KP is: 5 and up.
   */
   
}
