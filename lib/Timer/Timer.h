#ifndef Timer_h
#define Timer_h

#include <Arduino.h>

class Timer
{
public:
    // functions
    Timer(unsigned long updatePeriod);
    boolean doRun();

private:
    //variable and objects
    unsigned long lastActivation; // ms
    unsigned long updatePeriod; // ms

};
#endif