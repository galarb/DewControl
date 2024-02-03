/*                 ******************
              Class for HVAC Control
              Featuring: Dew Point calculation
                         Valve Control 
                         PID function
                         OLED HMI

                By: Noam Ron, Tomer Ozer, Gal Arbel, 2024
                **************************************              */
#ifndef HVACONTROL_H
#define HVACONTROL_H
#include <Arduino.h>
#include "WString.h"

class hvacontrol { 

  public:
    hvacontrol(int ecoderPinA, int ecoderPinB, int valvecontrolPin); 
    void begin(double bdrate);
    bool checkmode();
    void run(int kpp, int kii, int kdd);
    float getdew_point();
    float getvalvestat();
    float getwatertemp();
    float getairtemp();
    float getRH(); 
    bool setValve(int valve);
    void tftwelcome();
    void tftopershow(float dp, float sp);
    void tftdatashow(float valve, float airtemp, float RH, float pipetemp);
    void fault(int x);
    void sdbegin();
    void tftrun();
    void selftest();
    bool getdir();
    bool getevaldir();

  private:
    float setpipetempcool();
    float setpipetempheat();
    bool checkButton();
    int _encoderPinA;
    int _encoderPinB;
    int _valvecontrolPin;
    double PIDcalc(double inp, int sp);
    unsigned long currentTime;
    unsigned long previousTime;
    double elapsedTime;
    double error;
    double lastError;
    double input, output;
    double cumError, rateError;
    double kp = 0;
    double ki = 0; 
    double kd = 0;
    bool  _onofsw;
};
#endif 