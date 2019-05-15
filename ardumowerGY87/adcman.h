/*
Note: requires Arduino Due
Problem: you have multiple analog inputs, some need only to be sampled once, other need
a fixed sample rate. 

Solution:  
Arduino ADC manager (ADC0-ADC9)
- can capture multiple pins one after the other (example ADC0: 1000 samples, ADC1: 100 samples, ADC2: 1 sample etc.)
- can capture more than one sample into buffers (fixed sample rate)
- runs in background: interrupt-based (free-running) 
- two types of ADC capture:
  1) free-running ADC capturing (for certain sample count) (8 bit signed - zero = VCC/2)
  2) ordinary ADC sampling (one-time sampling) (10 bit unsigned)
- WARNING: never use any 'analogRead()' in your code when using this class!

How to use it: 
------example for one sampling-------
1. Initialize ADC:  ADCMan.begin();
2. Set ADC pin:     ADCMan.setupChannel(pinMotorMowSense, 1, true);
3. Program loop:    while (true){
                      ADCMan.run();
                      if (ADCMan.isConvComplete(pinMotorMowSense)){
                        int value = ADCMan.getValue(pinMotorMowSense);
                      }
                    }                    
------example for multiple samplings-------
1. Initialize ADC:  ADCMan.begin();
2. Set ADC pin:     ADCMan.setupChannel(pinPerimeterLeft, 255, true);
3. Program loop:    while (true){
                      ADCMan.run();
                      if (ADCMan.isConvComplete(pinPerimeterLeft)){                        
                          int16_t sampleCount = ADCMan.getSampleCount(pinPerimeterLeft);
                          int8_t *samples = ADCMan.getSamples(pinPerimeterLeft);    
                      }
                    }     
                    
*/


#ifndef ADCMAN_H
#define ADCMAN_H


#include <Arduino.h>

#define ADC_BITS  12    // 12 bit ADC  
#define ADC_REF  3.3   // 3.3 Volt reference

// sample rates
enum {
  SRATE_9615,
  SRATE_19231,
  SRATE_38462  
};

#define ADC_CHANNEL_COUNT_MAX 12

// one channel data
typedef struct ADCStruct {  
  int sampleCount;
  byte pin;  
  int8_t *samples;
  uint16_t value;
  uint16_t minValue;
  uint16_t maxValue;  
  uint16_t zeroOfs;
  bool convComplete;
  bool autoCalibrate;    
};


class ADCManager
{
  public:
    ADCManager();    
    int sampleRate;
    virtual void begin();
    virtual void run();
    virtual void setupChannel(byte pin, int samplecount, bool autocalibrate);    
    virtual int8_t* getSamples(byte pin);
    virtual int getSampleCount(byte pin);
    virtual uint16_t getValue(byte pin);    
    virtual float getVoltage(byte pin);
    virtual bool isConvComplete(byte pin);
    virtual void restartConv(byte pin);
    virtual void printInfo();
    virtual int getConvCounter();
  private:
    int convCounter;
    byte chCurr;
    byte chNext;
    virtual void setSampleCount(byte ch, int samplecount);    
    virtual void init(byte ch);
    virtual void postProcess(byte ch);    
    ADCStruct channels[ADC_CHANNEL_COUNT_MAX];
    boolean loadCalib();
    void loadSaveCalib(boolean readflag);
    void saveCalib();    
    void printCalib();
};

extern ADCManager ADCMan;


#endif
