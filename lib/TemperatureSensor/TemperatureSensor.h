#ifndef TemperatureSensor_h
#define TemperatureSensor_h

#include <Arduino.h>
#include <schema.pb.h>

#define TEMP_SENS_NUM_SAMPLES 5


class TemperatureSensor
{
  public:
    TemperatureSensor(uint16_t uploadFreq, uint8_t pinTherm, uint16_t nomTherm, int8_t nomTemp, \
      uint16_t B, uint16_t rSeries);
    // void setup();
    Temperature prepareData(int64_t time);
    
    int t0;
    uint16_t uploadFrequency;

  private:
    uint8_t pinTherm;        // which analog pin to connect
    uint16_t nomTherm;   // resistance at 25 degrees C
    int8_t nomTemp;           // temp. for nominal resistance (almost always 25 C)
    uint8_t numSamples();           // how many samples to take and average, more takes longer but is more 'smooth'
    uint16_t beta;              // The beta coefficient of the thermistor (usually 3000-4000)
    uint16_t rSeries;      // the value of the 'other' resistor
    
    int adc_samples[TEMP_SENS_NUM_SAMPLES];

    // class adc_samples
    // {
    //   int* array;
    //   adc_samples(int x) : array(new int[x]) {};
    // };
    
    // adc_samples adc_arr;

    // TemperatureSensor(int x) : adc_samples(new int[x]) {};
};
#endif