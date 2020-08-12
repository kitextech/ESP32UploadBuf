#include "Timer.h"

Timer::Timer(int _frequency) {
    updatePeriod = (unsigned long)( 1000 / _frequency);
}

boolean Timer::doRun() {
    if (millis() - lastActivation >= updatePeriod) {
        lastActivation = millis();
        return true;
    }
    return false;
}