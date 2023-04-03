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
// Version 2.04  1/08/2020  getDatalog_period( ) added.
// Version 2.1.0 9/7/2021   2nd pulse input added. Array of structs was individual variables. Set watthour and pulse count added.
// Version 2.1.1 26/7/2021  Version 2.1.0 was dated 9/7/20, "Shield" define was EmonLibCM2P_h
// Version 2.1.2 7/8/2021   'assumedACVoltage' was 'assumedVrms' (name conflict in some sketches).
// Version 2.2.0 14/9/2021  Optional parameter 'edge' added to setPulseMinPeriod().
// Version 2.2.1 5/12/2021  Repackaged - no change.
// Version 2.2.2 15/9/2022  No change.



#ifndef EmonLibCM_h
#define EmonLibCM_h

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif

#define MICROSPERSEC 1.0e6

// Dallas DS18B20 libraries & commands

#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>

#define DS18B20SIG 0x28
#define SKIP_ROM 0xCC 
#define MATCH_ROM 0x55
#define CONVERT_TEMPERATURE 0x44
#define READ_SCRATCHPAD 0xBE
#define WRITE_SCRATCHPAD 0x4E
#define COPY_SCRATCHPAD 0x48
#define UNUSED_TEMPERATURE 30000     // this value (300°C) is sent if no sensor has ever been detected
#define OUTOFRANGE_TEMPERATURE 30200 // this value (302°C) is sent if the sensor reports < -55°C or > +125°C
#define BAD_TEMPERATURE 30400        // this value (304°C) is sent if no sensor is present or the checksum is bad (corrupted data)
                                     // NOTE: The sensor might report 85°C if the temperature is retrieved but the sensor has not been commanded
                                     //  to measure the temperature.

#define TEMPRES_9 0x1F
#define TEMPRES_10 0x3F
#define TEMPRES_11 0x5F
#define TEMPRES_12 0x7F
#define CONVERSION_LEAD_TIME 752     // this is the conversion time of the DS18B20 in ms at 12-bits (rounded up to multiple of 8).
#define SENSOR_READ_TIME 16          // this is the time to read one sensor in ms (plus the interval between readings)

#define VREF_EXTERNAL 0              // ADC Reference is Externally supplied voltage
#define VREF_NORMAL 1                // ADC Reference is Processor Supply (AVcc)
#define VREF_INTERNAL 3              // ADC Reference is Internal 1.1V reference


typedef uint8_t DeviceAddress[8];

void EmonLibCM_cycles_per_second(unsigned int _cycles_per_second);
void EmonLibCM_min_startup_cycles(unsigned int _min_startup_cycles);
void EmonLibCM_datalog_period(float _datalog_period_in_seconds);
void EmonLibCM_setADC(int _ADCBits,  int ADCDuration);
void EmonLibCM_ADCCal(double _RefVoltage);
void EmonLibCM_SetADC_VChannel(byte ADC_Input, double _amplitudeCal);
void EmonLibCM_SetADC_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal);
void EmonLibCM_setADC_VRef(byte _ADCRef);
void EmonLibCM_ReCalibrate_VChannel(double _amplitudeCal);
void EmonLibCM_ReCalibrate_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal);

void EmonLibCM_setAssumedVrms(double _assumedVrms);
void EmonLibCM_setWattHour(byte channel, long _wh);
void EmonLibCM_setPulseCount(long _pulseCount);
void EmonLibCM_setPulseCount(byte channel, long _pulseCount);
void EmonLibCM_setPulseEnable(byte channel, bool _enable);
void EmonLibCM_setPulseEnable(bool _enable);
void EmonLibCM_setPulsePin(int _pin);
void EmonLibCM_setPulsePin(byte channel, int _pin, int _interrupt);
void EmonLibCM_setPulsePin(byte channel, int _pin);
void EmonLibCM_setPulseMinPeriod(int _period, byte _edge=FALLING);
void EmonLibCM_setPulseMinPeriod(byte channel, int _period, byte _edge=FALLING);

bool EmonLibCM_acPresent(void);

int EmonLibCM_getLogicalChannel(byte ADC_Input);
int EmonLibCM_getRealPower(int channel);
int EmonLibCM_getApparentPower(int channel);
double EmonLibCM_getPF(int channel);
double EmonLibCM_getIrms(int channel);
double EmonLibCM_getVrms(void);
double EmonLibCM_getAssumedVrms(void);
double EmonLibCM_getDatalog_period(void);
double EmonLibCM_getLineFrequency(void);
long EmonLibCM_getWattHour(int channel);
unsigned long EmonLibCM_getPulseCount(void);
unsigned long EmonLibCM_getPulseCount(byte channel);


void EmonLibCM_setTemperatureDataPin(byte _dataPin);
void EmonLibCM_setTemperaturePowerPin(char _powerPin);
void EmonLibCM_setTemperatureResolution(byte _resolution);
void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray);
void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray, bool keep);
void EmonLibCM_setTemperatureArray(int *temperatureArray);
void EmonLibCM_setTemperatureMaxCount(int _maxCount);
void EmonLibCM_TemperatureEnable(bool _enable);
bool EmonLibCM_getTemperatureEnabled(void);
void printTemperatureSensorAddresses(bool emonPi=false);
void printTemperatureSensorsEnabled(void);
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

// General calculations

void calcPhaseShift(byte lChannel);
void calcTemperatureLead(void);

// Pulse debounce

void countPulses(void);

#endif
