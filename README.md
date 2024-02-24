HVAC Dewpoint Control
	PID control of Water Valve 
    featuring:
		   - Dew point calculation
       - PID generic function
       - Selftest procedure
       - Encoder input
       - button interupt support
       - tft screen and SD card on SPI
       - Clicli Embedded (CLI Interface)
 Note the 22UTH-13 and 22UTH-14 should be carefully checked to 0-50 mmeasurig range by their jumpers.
 Static pins configuration:
      Button - pin 2
      ValveStatus=A0, WaterTempPin=A1, RHPin=A2, AirTempPin=A3
      SPI Connection:
      MOSI - pin 11
      MISO - pin 12
      CLK - pin 13 
      tft CS - pin 9
      SD Card CS - pin 10

By:
Tomer Ozer
Noam Ron
Gal Arbel
2024