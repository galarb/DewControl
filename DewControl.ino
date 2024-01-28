/*   Notes:
  Button on pin 2
  ValveStatus=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3
  SPI Connection:
  ** MOSI - pin 11
  ** MISO - pin 12
  ** CLK - pin 13 
  tft CS - pin 9
  SD Card CS - pin 10
*/
#include "clicli.h"
#include "hvacontrol.h"

hvacontrol hvac(
5, //encoder A
6, //encoder B
3); //valve control

clicli mycli(hvac); //setup clicli tool for hvac control

void setup() { 
  hvac.begin(115200);
}

void loop() { 
  mycli.run();
  hvac.run(1, 0, 0);
}
