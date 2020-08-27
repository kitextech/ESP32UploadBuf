#include "Timer.h"

Timer::Timer(unsigned long updateperiod) :
updatePeriod(updateperiod) {}

boolean Timer::doRun() {
    if (millis() - lastActivation >= updatePeriod) {
        lastActivation = millis();
        return true;
    }
    return false;
}