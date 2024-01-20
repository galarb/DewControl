/*                 ******************
              Class for HVAC Control
              Featuring: Dew Point calculation
                          
                         PID function
                By: Noam Ron, Tomer Ozer, Gal Arbel, 2024
                **********************                                   */
#ifndef HVACONTROL_H
#define HVACONTROL_H

class hvacontrol { 

  public:
    hvacontrol(int ecoderPinA, int ecoderPinB, int buttonClick); 
    void begin(double bdrate);    
    int getrefligh();
    void ShowInfoLcd(int speed, int direction, int BTstatus);
    void lcdenshow(int clicks, int output, int tempsteps);
    void lcdswitch(bool status);
    void run(int kpp, int kii, int kdd);
    float getdew_point();
    float getdew_valvestat();


  private:
    void checkencoder();
    void checkButton();

    int _encoderPinA;
    int _encoderPinB;
    int _buttonClick;



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