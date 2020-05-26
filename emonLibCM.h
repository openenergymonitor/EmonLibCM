/*
  emonLibCM.h - Library for openenergymonitor
  GNU GPL
*/

// This library provides continuous single-phase monitoring of real power on up to five CT channels.
// All of the time-critical code is now contained within the ISR, only the slower activities
// are done within the main code. These slower activities include RF transmissions,
// and all Serial statements (not part of the library).  
//
// This library is suitable for either 50 or 60 Hz operation.
//
// Original Author: Robin Emley (calypso_rae on Open Energy Monitor Forum)
// Addition of Wh totals by: Trystan Lea
// Heavily modified to improve performance and calibration; temperature measurement 
//  and pulse counting incorporated into the library,  by Robert Wall 
//  Release for testing 4/1/2017
//
// Version 2.0  21/11/2018
// Version 2.01  3/12/2018  No change.
// Version 2.02 13/07/2019  getLogicalChannel( ), ReCalibrate_VChannel( ), ReCalibrate_IChannel( ) added, setPulsePin( ) interrupt no. was obligatory,
// Version 2.03 25/10/2019  getLineFrequency( ), setADC_VRef( ) added.

                              



#ifndef EmonLibCM_h
#define EmonLibCM_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

#define MICROSPERSEC 1.0e6

// Dallas DS18B20 commands
#define SKIP_ROM 0xCC 
#define MATCH_ROM 0x55
#define CONVERT_TEMPERATURE 0x44
#define READ_SCRATCHPAD 0xBE
#define WRITE_SCRATCHPAD 0x4E
#define COPY_SCRATCHPAD 0x48
#define UNUSED_TEMPERATURE 30000     // this value (300C) is sent if no sensor has ever been detected
#define OUTOFRANGE_TEMPERATURE 30200 // this value (302C) is sent if the sensor reports < -55C or > +125C
#define BAD_TEMPERATURE 30400        // this value (304C) is sent if no sensor is present or the checksum is bad (corrupted data)
                                     // NOTE: The sensor might report 85C if the temperature is retrieved but the sensor has not been commanded
                                     //  to measure the temperature.

#define TEMPRES_9 0x1F
#define TEMPRES_10 0x3F
#define TEMPRES_11 0x5F
#define TEMPRES_12 0x7F
#define CONVERSION_LEAD_TIME 752     // this is the conversion time of the DS18B20 in ms at 12-bits (rounded up to multiple of 8).

#define VREF_EXTERNAL 0              // ADC Reference is Externally supplied voltage
#define VREF_NORMAL 1                // ADC Reference is Processor Supply (AVcc)
#define VREF_INTERNAL 3              // ADC Reference is Internal 1.1V reference


typedef uint8_t DeviceAddress[8];

void EmonLibCM_cycles_per_second(int _cycles_per_second);
void EmonLibCM_min_startup_cycles(int _min_startup_cycles);
void EmonLibCM_datalog_period(float _datalog_period_in_seconds);
void EmonLibCM_setADC(int _ADCBits,  int ADCDuration);
void EmonLibCM_ADCCal(double _RefVoltage);
void EmonLibCM_SetADC_VChannel(byte ADC_Input, double _amplitudeCal);
void EmonLibCM_SetADC_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal);
void EmonLibCM_setADC_VRef(byte _ADCRef);
void EmonLibCM_ReCalibrate_VChannel(double _amplitudeCal);
void EmonLibCM_ReCalibrate_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal);

void EmonLibCM_setPulseEnable(bool _enable);
void EmonLibCM_setPulsePin(int _pin);
void EmonLibCM_setPulsePin(int _pin, int _interrupt);
void EmonLibCM_setPulseMinPeriod(int _periodwidth);

bool EmonLibCM_acPresent(void);

int EmonLibCM_getLogicalChannel(byte ADC_Input);
int EmonLibCM_getRealPower(int channel);
int EmonLibCM_getApparentPower(int channel);
double EmonLibCM_getPF(int channel);
double EmonLibCM_getIrms(int channel);
double EmonLibCM_getVrms(void);
double EmonLibCM_getLineFrequency(void);
long EmonLibCM_getWattHour(int channel);
unsigned long EmonLibCM_getPulseCount(void);


void EmonLibCM_setTemperatureDataPin(byte _dataPin);
void EmonLibCM_setTemperaturePowerPin(char _powerPin);
void EmonLibCM_setTemperatureResolution(byte _resolution);
void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray);
void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray, bool keep);
void EmonLibCM_setTemperatureArray(int *temperatureArray);
void EmonLibCM_setTemperatureMaxCount(int _maxCount);
void EmonLibCM_TemperatureEnable(bool _enable);
bool EmonLibCM_getTemperatureEnabled(void);
void printTemperatureSensorAddresses(void);
void convertTemperatures(void);
void retrieveTemperatures(void);
int EmonLibCM_getTemperatureSensorCount(void);
float EmonLibCM_getTemperature(char sensorNumber);

#ifdef INTEGRITY
int EmonLibCM_minSampleSetsDuringThisMainsCycle(void);
#endif


void EmonLibCM_Init();

void EmonLibCM_Start();

bool EmonLibCM_Ready();

// for general interaction between the main code and the ISR
extern volatile boolean datalogEventPending;


#endif
