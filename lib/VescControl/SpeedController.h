#ifndef SpeedController_h
#define SpeedController_h

#include <VescUart.h>

class SpeedController
{
private:
    enum ControlMode
    {
        OPTIMAL_POWER,
        RPM_LIMIT
    } mode = OPTIMAL_POWER;

    /* data */
    float shortTermPower;
    float longTermPower;
    float rpmsetpoint;
    float lastCurrent;
    int transitionIndex; // transition smoothly from RPM to optimal control


public:
    SpeedController(/* args */);
    void doControlV1(VescUart &vesc, float &currentSetupoint, float torqueCoefficient);
    void doControlV2(VescUart &vesc);

    void controlOptimalPower(VescUart &vesc, float &currentSetupoint, float torqueCoefficient);
    void controlRPM(VescUart &vesc);
};

#endif