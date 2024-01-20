#include "clicli.h"
#include "hvacontrol.h"

hvacontrol hvac(5, 6, 2); //encoder A and B, button
//ValveStatus=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3

clicli mycli(hvac); //setup clicli tool for hvac control

void setup() { 
  hvac.begin(115200);
}

void loop() { 
  mycli.run();
  hvac.run(1, 0, 0);
 }
