/*                 ******************
              Class for managing Button Interface
              Featuring Interrupt requests
                         
                By: Gal Arbel 2023
                **********************                                   */
#ifndef BUTTONIRQ_H
#define BUTTONIRQ_H
#include "ButtonIRQ.h"

class ButtonIRQ { 

  public:
    ButtonIRQ(int Button); 
    void begin();
    bool isTrue(); 

  private:
    int _button;
};


#endif 