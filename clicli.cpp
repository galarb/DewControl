/*

                    CLI Software for Arduino

               A Simple Command Line Interface 
  Example functions:              
              Feature                |  CLI Usage
___________________________________________________
 Digial Write HIGH to a specific pin |  h (pin)
 Digial Write LOW to a specific pin  |  l (pin)
 Shoot command                       |  s (angle)
 Digital Read                        |  r (pin)
 Analog Read                         |  e (pin) 

               
      by Gal Arbel
      Oct 2022
*/


#include "clicli.h"
#include "Arduino.h"
#include "HardwareSerial.h"

const unsigned int MAX_MESSAGE_LENGTH = 64;

clicli::clicli(hvacontrol &hvacontrol):myhvacontrol(hvacontrol), number(7) {

}

void clicli::begin() {
  //
}
void clicli::run() {

// CLI - Messages from Terminal
  while (Serial.available() > 0) { 
   char message[MAX_MESSAGE_LENGTH];
   static unsigned int message_pos = 0;
   char inByte = Serial.read();   //Read the next available byte in the serial receive buffer
    if ( inByte != '\n' && (message_pos < MAX_MESSAGE_LENGTH - 1) )
     {
     message[message_pos] = inByte;  //Add the incoming byte to our message
     message_pos++;
     }
     //Full message received...
     else
     {
      message[message_pos] = '\0';     //Add null character to string
      Serial.println(message);     //echo the message to terminal
        
      int command[4];
      int argindex = 0;
      char cmd;
      char delim[] = " ";
	     char tmpmsg[MAX_MESSAGE_LENGTH];
       strcpy(tmpmsg,message);
       message_pos = 0;
       message[message_pos] = '\0';     //Add null character to string

        char *ptr = strtok(tmpmsg, delim);
	      while(ptr != NULL)
	       {
		      //Serial.printf("'%s'\n", ptr);
          if (argindex == 0) {
            cmd = ptr[0];
          }
          command[argindex] = atoi(ptr);   
          //Serial.println(command[argindex]);
          argindex++;  
		      ptr = strtok(NULL, delim);
	       } 

      switch (cmd) {
       case 'h': //Set port to HIGH
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],HIGH);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is SET");   
        delay(1000);
        break;
        
       case 'l': // Set port to LOW
        pinMode(command[1],OUTPUT);
        digitalWrite(command[1],LOW);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.println(" is RESET");   
        delay(1000);
        break;
       
       case 'r': // digital read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(digitalRead(command[1]));   
        delay(1000);
        break;
       
       case 'a':
        
        break;
       

      case 'e': // analog read
        pinMode(command[1],INPUT);
        Serial.print("Pin "); 
        Serial.print(command[1]);   
        Serial.print(" Value = "); 
        Serial.println(analogRead(command[1]));   
        delay(1000);
        break;

      case 'd'://print debug info
        Serial.print("valvestat = ");
        Serial.print(myhvacontrol.getvalvestat());
        Serial.println("%");
        Serial.print("watertemp = ");
        Serial.print(myhvacontrol.getwatertemp());
        Serial.println(" Degrees");
        Serial.print("Airtemp = ");
        Serial.print(myhvacontrol.getairtemp());
        Serial.println(" Degrees");
        Serial.print("RH = ");
        Serial.print(myhvacontrol.getRH());
        Serial.println("%");
        Serial.print("dew point = ");
        Serial.print(myhvacontrol.getdew_point());
        Serial.println(" Degrees");

        break;
      case 'v':
        myhvacontrol.setValve(command[1]);
        break;
      case 'p':
        myhvacontrol.PIDcalc(command[1], command[2]);
        break;
       message_pos = 0;     //Reset for the next message
      }
   }
   delay (60); 
 } 
}
