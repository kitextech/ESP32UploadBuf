#include "SpeedController.h"

#define RPM_TO_ERPM 163.8f
#define RPM_TO_OMEGA 2.0f * PI / 60.0f
#define ERPM_TO_RPM 1.0f/163.8f
#define ERPM_TO_OMEGA 2.0f * PI / (163.8f * 60.0f);

SpeedController::SpeedController(/* args */)
{
    
}
    
void SpeedController::controlOptimalPower(VescUart &vesc, float &currentSetpoint, float torqueCoefficient) {
    float rpm = vesc.data.rpm*ERPM_TO_RPM;

    // as long as we are under rated power are in current control: https://www.intechopen.com/books/fundamental-and-advanced-topics-in-wind-power/a-complete-control-scheme-for-variable-speed-stall-regulated-wind-turbines

    // K = 1/(2 * lambda_optiomal^3 ) * rho * pi * R^5 * Cp_max = 1/(2*6^3)*1.2*pi*2.1^5*0.33 = 0.1176
    // example: 100 RPM => (100/60*2*pi)^2 * 0.1176 = 12.89 Nm

    // free spin the turbine if we are using power and below curent se RPM.
        
    float omega = vesc.data.rpm*ERPM_TO_OMEGA;
    float torque = omega*omega*torqueCoefficient; // 0.1176;
    float current = torque/-1.6; // T / I = -1.6 => I = T/ -1.6 // from influx db. 

    if (current > 0 ) { // no positive current! 
        current = 0;
    }

    if (vesc.data.rpm < 0) { // if we for some reason are turning backwards: ground handling, test ect.
        current = -current;
    }

    if (transitionIndex < 10) { // phase new current in slowly
        float factor = (transitionIndex+1)/10.f;
        current = factor * current + (1-factor) * lastCurrent;
        transitionIndex +=1;
    } 

    vesc.setCurrent(current); 
    currentSetpoint = current;

    if (rpm > 140) { // check if we need to change control mode
        mode = RPM_LIMIT;
        rpmsetpoint = (float) vesc.data.rpm; // start at current rpm
    } 
}
    
void SpeedController::controlRPM(VescUart &vesc) {

    // change the rpm by 200 eRPM every time the function is run.. // set the RPM at maximum 100 eRPM from the current eRPM
    float rpmTarget = 135*RPM_TO_ERPM;

    float delta = rpmTarget - rpmsetpoint;
    delta = min(delta, 200.0f);
    delta = max(-200.0f, delta);
    rpmsetpoint = rpmsetpoint + delta;
    
    vesc.setRPM( rpmsetpoint );

    if (longTermPower < 400) { // power lower than 400W. 
        mode = OPTIMAL_POWER;
        lastCurrent = vesc.data.avgMotorCurrent;
        transitionIndex = 0;
    } 

}

void SpeedController::doControlV1(VescUart& vesc, float &currentSetpoint, float torqueCoefficient) {

    // keep track of shortterm power average and longterm power average.
    float power = vesc.data.inpVoltage * vesc.data.avgInputCurrent;
    shortTermPower = power * 0.1 + shortTermPower * 0.9;
    longTermPower = power * 0.01 + longTermPower * 0.99;


    switch (mode)
    {
    case OPTIMAL_POWER:
        controlOptimalPower(vesc, currentSetpoint, torqueCoefficient);
        break;
    case RPM_LIMIT:
        controlRPM(vesc);
        break;
    }
}

void SpeedController::doControlV2(VescUart& vesc) {
    // not implemented

}