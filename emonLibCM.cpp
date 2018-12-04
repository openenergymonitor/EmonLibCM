// emonLibCM.cpp - Library for openenergymonitor
// GNU GPL

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
// Version 2.01  3/12/2018  Calculation error in phase error correction - const.'360' missing, 'x' & 'y' coefficients swapped.


// #include "WProgram.h" un-comment for use on older versions of Arduino IDE

// #define SAMPPIN 5           // Preferred pin for testing. This MUST be commented out if a temperature sensor is connected. Only include for testing.
// #define SAMPPIN 19          // Alternative pin for testing. This MUST be commented out if the temperature sensor power is connected here. Only include for testing.

#include "emonLibCM.h"

#if defined(ARDUINO) && ARDUINO >= 100

#include "Arduino.h"

#else

#include "WProgram.h"

#endif


// Dallas DS18B20 commands

#include <Wire.h>
#include <SPI.h>
#include <util/crc16.h>
#include <OneWire.h>
#include <DallasTemperature.h>                                         //http://download.milesburton.com/Arduino/MaximTemperature/DallasTemperature_LATEST.zip



int cycles_per_second = 50;                                            // mains frequency in Hz (i.e. 50 or 60)
float datalog_period_in_seconds = 10.0;
int min_startup_cycles = 10;

// Maximum number of Current (I) channels used to create arrays
static const int max_no_of_channels = 5;

// User set number of Current (I) channels used by 'for' loops
int no_of_channels = 4;
// number of Current (I) channels that have been set
byte no_of_Iinputs = 0;

// for general interaction between the main code and the ISR
volatile boolean datalogEventPending;
unsigned long missing_Voltage = 0;                                     // provides a timebase mechanism for current-only use
                                                                       //  - uses the ADC free-running rate as a clock.
// Arrays for current channels (zero-based)                                                                       
int realPower_CT[max_no_of_channels];
int apparentPower_CT[max_no_of_channels];
double Irms_CT[max_no_of_channels];
long wh_CT[max_no_of_channels];
double pf[max_no_of_channels];
double Vrms;
volatile boolean ChannelInUse[max_no_of_channels];

    
// analogue ports
static byte ADC_Sequence[max_no_of_channels+1] = {0,1,2,3,4,5};        // <-- Sequence in which the analogue ports are scanned, first is Voltage, remainder are currents
// ADC data
int ADCBits = 10;                                                      // 10 for the emonTx and most of the Arduino range, 12 for the Arduino Due.
double Vref = 3.3;                                                     // ADC Reference Voltage = 3.3 for emonTX, 5.0 for most of the Arduino range.
int ADCDuration = 104;                                                 // Time in microseconds for one ADC conversion = 104 for 16 MHz clock 

// Pulse Counting;
bool PulseEnabled = false;
byte PulsePin = 3;                                                     // default to DI3 for the emonTx V3
byte PulseInterrupt = 1;                                               // default to int1 for the emonTx V3
unsigned long PulseMinPeriod = 110;                                    // default to 110 ms
unsigned long pulseCount = 0;                                          // Total number of pulses from switch-on 
volatile unsigned long pulses= 0;                                      // Incremental number of pulses between readings
unsigned long pulseTime;                                               // Instant of the last pulse - used for debounce logic
void onPulse();                                                        // pulse time of arrival


// Set-up values
//--------------
// These set up the library for different hardware configurations
//
// setADC                    Sets the ADC resolution and the conversion time
// cycles_per_second         Defines the mains frequency
// _min_startup_cycles       The period of inactivity whilst the system settles at start-up
// _datalog_period           The rate at which data is reported for logging
// SetADC_Channel            Defines the channels input pin and calibration constants
//
//
// Calibration values
//-------------------
// Many calibration values are used in this sketch: 
//
// ADCCal                    This sets up the ADC reference voltage
// voltageCal                This is the principal calibration for the ac adapter.
// currentCal                A per-channel amplitude calibration for each current transformer.
// phaseCal                  A per-channel calibration for the phase error correction


// With most hardware, the default values are likely to work fine without 
// need for change.  A compact explanation of each of these values now follows:

// Voltage calibration constant. This is the mains voltage that would give 1 V 
//  at the ADC input:

// AC-AC Voltage adapter is designed to step down the voltage from 240V to 9V
// but the AC Voltage adapter is running open circuit and so output voltage is
// likely to be about 20% higher than 9V, actually 11.6 V for the UK Ideal adapter
//  (from the data sheet). 
// Open circuit step down = 240 / 11.6 = 20.69

// The output voltage is then stepped down further with the voltage divider which has 
// values Rb = 10k, Rt = 120k which will reduce the voltage by 13 times.

// The combined step down is therefore 20.69 x 13 = 268.97 which is the 
// theoretical calibration constant. The actual constant for a given
//   unit and ac adapter is likely to be different by a few percent.
//   Other adapters are different may be more different.

// Current calibration constant. This is the mains current that would give 1 V
// at the ADC input:
// Current calibration constant channels 1 - 3 = 2000 / 22 Ohms burden resistor = 90.9
// (The CT sensor is 100 A : 50 mA, therefore a ratio of 2000:1)
//  for channel 4 is 2000 / 120R burden resistor = 16.67
//  The actual constant for a given unit and CT is likely to be different by a few percent.

// phaseCal is used to alter the phase of the voltage waveform relative to the
// current waveform.  The algorithm interpolates between the most recent pair
// of voltage samples according to the value of phaseCal. 
//
// The value of phaseCal entered is difference between the phase lead of the voltage transformer and
// the phase lead of the current transformer, in degrees (changes of less than 0.1 deg are
// unlikely to make a detectable difference).



/**************************************************************************************************
*
*   General variables 
*
*
***************************************************************************************************/


// --------------  general global variables -----------------
// Some of these variables are used in multiple blocks so cannot be static.
// For integer maths, many variables need to be 'long' or in extreme cases 'int64_t'

double currentCal[max_no_of_channels] = {90.91, 90.91, 90.91, 16.67, 90.91};
double  phaseCal_CT[max_no_of_channels] ={4.2, 4.2, 4.2, 1.0, 4.2}; 

double voltageCal = 268.97;

unsigned int ADC_Counts = 1 << ADCBits;

static const byte startUpPeriod = 3;                // in seconds, to allow inputs to settle

bool stop = false;
bool firstcycle = true;
    
unsigned int samplesDuringThisCycle;
bool acPresent = false;                             // TRUE when ac voltage input is detected.
unsigned int acDetectedThreshold = ADC_Counts >> 5; // ac voltage detection threshold, ~10% of nominal voltage (given large amount of ripple)

int datalogPeriodInMainsCycles;
unsigned long ADCsamples_per_datalog_period;

// accumulators & counters for use by the ISR
long     cumV_deltas;                               // <--- for offset removal (V)
int64_t  sumPA_CT[max_no_of_channels];              // 'partial' power for real power calculation
int64_t  sumPB_CT[max_no_of_channels];              // 'partial' power for real power calculation
uint64_t sumIsquared_CT[max_no_of_channels];
long     cumI_deltas_CT[max_no_of_channels];        // <--- for offset removal (I)
uint64_t sum_Vsquared;                              // for Vrms datalogging   
long     samplesDuringThisDatalogPeriod;   

// Copies of ISR data for use by the main code
// These are filled by the ADC helper routine at the end of the datalogging period

volatile int64_t  copyOf_sumPA_CT[max_no_of_channels]; 
volatile int64_t  copyOf_sumPB_CT[max_no_of_channels]; 
volatile uint64_t copyOf_sumIsquared_CT[max_no_of_channels]; 
volatile uint64_t copyOf_sum_Vsquared;
volatile long     copyOf_samplesDuringThisDatalogPeriod;
volatile int64_t  copyOf_cumI_deltas[max_no_of_channels];
volatile int64_t  copyOf_cumV_deltas;

// For mechanisms to check the integrity of this code structure
#ifdef INTEGRITY
int sampleSetsDuringThisMainsCycle;    
int lowestNoOfSampleSetsPerMainsCycle;
volatile int copyOf_lowestNoOfSampleSetsPerMainsCycle;
#endif

enum polarities {NEGATIVE, POSITIVE};
// For an enhanced polarity detection mechanism, which includes a persistence check
#define POLARITY_CHECK_MAXCOUNT 3 // 1
enum polarities polarityUnconfirmed;   
enum polarities polarityConfirmed;                  // for improved zero-crossing detection
enum polarities polarityConfirmedOfLastSampleV;     // for zero-crossing detection

float residualEnergy_CT[max_no_of_channels];
double x[max_no_of_channels], y[max_no_of_channels]; // coefficients for real power interpolation


// Temperature measurement
//
// Hardware Configuration
byte W1Pin = 5;                                     // 1-Wire pin for temperature = 5 for emonTx V3, 4 for emonTx V2 & emonTx Shield
char DS18B20_PWR = -1;                              // Power pin for DS18B20 temperature sensors. Default -1 - power off

// Global variables used only inside the library
OneWire oneWire(W1Pin);
DallasTemperature sensors(&oneWire);
bool temperatureEnabled = false;
byte numSensors = 0;
byte temperatureResolution = TEMPRES_11;
byte temperatureMaxCount = 1;
DeviceAddress *temperatureSensors = NULL;
int *temperatures = NULL;
unsigned int temperatureConversionDelayTime; 
unsigned long temperatureConversionDelaySamples;

bool startConvertTemperatures = false;
bool convertingTemperaturesNoAC = false;            // Only used when not using mains for timing.

/**************************************************************************************************
*
*   APPLICATION INTERFACE - Getters & Setters
*
*
***************************************************************************************************/

void EmonLibCM_SetADC_VChannel(byte ADC_Input, double _amplitudeCal)
{
    ADC_Sequence[0] = ADC_Input; 
    voltageCal = _amplitudeCal;
}

void EmonLibCM_SetADC_IChannel(byte ADC_Input, double _amplitudeCal, double _phaseCal)
{
    currentCal[no_of_Iinputs] = _amplitudeCal;
    phaseCal_CT[no_of_Iinputs] = _phaseCal;
    ChannelInUse[no_of_Iinputs] = true;            
    ADC_Sequence[++no_of_Iinputs] = ADC_Input; 
}

void EmonLibCM_cycles_per_second(int _cycles_per_second)
{
    cycles_per_second = _cycles_per_second;
    datalogPeriodInMainsCycles = datalog_period_in_seconds * cycles_per_second;  
}

void EmonLibCM_min_startup_cycles(int _min_startup_cycles)
{
    min_startup_cycles = _min_startup_cycles;
}

void EmonLibCM_datalog_period(float _datalog_period_in_seconds)
{
    datalog_period_in_seconds = _datalog_period_in_seconds;
    datalogPeriodInMainsCycles = datalog_period_in_seconds * cycles_per_second;  
}

void EmonLibCM_setADC(int _ADCBits,  int _ADCDuration)
{
    ADCBits = _ADCBits;
    ADCDuration = _ADCDuration;
}

void EmonLibCM_setPulseEnable(bool _enable)
{
    PulseEnabled = _enable;
}

void EmonLibCM_setPulsePin(int _pin, int _interrupt)
{
    PulsePin = _pin;
    PulseInterrupt = _interrupt;
}

void EmonLibCM_setPulseMinPeriod(int _period)
{
    PulseMinPeriod = _period;
}


void EmonLibCM_ADCCal(double _Vref)
{
    Vref = _Vref;
}   


bool EmonLibCM_acPresent(void)
{
    return(acPresent);
}

int EmonLibCM_getRealPower(int channel)
{
    return realPower_CT[channel];
}

int EmonLibCM_getApparentPower(int channel)
{
    return apparentPower_CT[channel];
}

double EmonLibCM_getPF(int channel)
{
    return pf[channel];
}

double EmonLibCM_getIrms(int channel)
{
    return Irms_CT[channel];
}

double EmonLibCM_getVrms(void)
{
    return Vrms;
}

long EmonLibCM_getWattHour(int channel)
{
    return wh_CT[channel];
}

unsigned long EmonLibCM_getPulseCount(void)
{
    return pulseCount;
}


#ifdef INTEGRITY
int EmonLibCM_minSampleSetsDuringThisMainsCycle(void)    
{
    return copyOf_lowestNoOfSampleSetsPerMainsCycle;
    // The answer should be 192 (50 Hz) or 160 (60 Hz) divided by
    // 2 for 1 CT in use, 3 for 2 CTs in use, etc.
    // Returns 999 if no mains is detected.
}    
#endif



void EmonLibCM_Init(void)
{   
    // Set number of channels to the number defined, else use the defaults
    if (no_of_Iinputs)
    { 
        no_of_channels = no_of_Iinputs;
        for (byte i = no_of_Iinputs+1; i < max_no_of_channels; i++)
            ChannelInUse[i] = false;
    }
 
    const double two_pi = 6.2831853;
    double sampleRate = ADCDuration * (no_of_channels + 1) * two_pi * cycles_per_second / MICROSPERSEC; // in radians

    // Set up voltage calibration to take account of ADC width etc
    voltageCal = voltageCal * Vref / ADC_Counts;
    
    // Likewise each current channel
    for (int i=0; i<no_of_channels; i++)
    {
        currentCal[i] = currentCal[i] * Vref / ADC_Counts;  
 
        // phaseCal value supplied is the difference between VT lead and CT lead in degrees
        //  Add the delay due to the time taken by the ADC to convert one sample (ADCDuration),
        //  knowing the position of the current sample with respect to 
        //  the voltage, then convert to radians.
        // x & y are the constants used in the power interpolation. (Sanity check: x + y ≈ 1)
        
        double phase_shift = (phaseCal_CT[i] / 360.0 + ADC_Sequence[i+1] * 
                               (double)ADCDuration * cycles_per_second/MICROSPERSEC) * two_pi;                // Total phase shift in radians
        y[i] = sin(phase_shift) / sin(sampleRate);        
        x[i] = cos(phase_shift) - y[i] * cos(sampleRate);
    
        residualEnergy_CT[i] = 0;
        wh_CT[i] = 0;   
    }

    EmonLibCM_Start();
    
    datalogPeriodInMainsCycles = datalog_period_in_seconds * cycles_per_second;  
    datalogEventPending = false;
    ADCsamples_per_datalog_period = datalog_period_in_seconds * MICROSPERSEC / ADCDuration;
        // nominally a 0.16% at 1s, or 0.004% at 10 s truncation error by having this as an integer - insignificant 
#ifdef  SAMPPIN
    pinMode(SAMPPIN, OUTPUT);
    digitalWrite(SAMPPIN, LOW);
#endif

    if (PulseEnabled)
    {
        pinMode(PulsePin, INPUT_PULLUP);                              // Set interrupt pulse counting pin as input
        attachInterrupt(PulseInterrupt, onPulse, RISING);             // Attach pulse counting interrupt pulse counting,  
    }
}

/**************************************************************************************************
*
*   START
*
*
***************************************************************************************************/


void EmonLibCM_Start(void)
{
    firstcycle = true;
    missing_Voltage = 0;
    
    // Set up the ADC to be free-running 
    // 
    // BIT:    7,    6,    5,    4,    3,    2,     1,     0
    // ADCSRA: ADEN, ADSC, ADFR, ADIF, ADIE, ADPS2, ADPS1, ADPS0
    //
    // ADEN: ADC Enable
    // ADSC: ADC Start Conversion
    // ADFR: ADC Free Running Select, or ADATE (ADC Auto Trigger Enable)
    // ADIF: ADC Interrupt Flag
    // ADIE: ADC Interrupt Enable
    // ADPS2, ADPS1, ADPS0: ADC Prescaler Select Bits (CLOCK FREQUENCY)
    //
    // The default value of ADCSRA before we change it with the following is 135
    // ADCSRA: ADEN, ADSC, ADFR, ADIF, ADIE, ADPS2, ADPS1, ADPS0
    //         128   64    32    16    8     4      2      1
    //         1     0     0     0     0     1      1      1
    // The ADC is enabled and the ADC clock is set to system clock / 128
    //
    // The following sets ADCSRA to a value of 239
    //         1     1     1     0     1     1      1      1
     
    ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
    ADCSRA |= (1 << ADEN);                       // Enable the ADC 

    ADCSRA |= (1<<ADATE);  // set the Auto Trigger Enable bit in the ADCSRA register.  Because 
                           // bits ADTS0-2 have not been set (i.e. they are all zero), the 
                           // ADC's trigger source is set to "free running mode".
                         
    ADCSRA |=(1<<ADIE);    // set the ADC interrupt enable bit. When this bit is written 
                           // to one and the I-bit in SREG is set, the 
                           // ADC Conversion Complete Interrupt is activated. 

    ADCSRA |= (1<<ADSC);   // start ADC manually first time 
    sei();                 // Enable Global Interrupts
    
    
}


void EmonLibCM_StopADC(void)
{
    // This stop function returns the ADC to default state
    ADCSRA  = (1<<ADPS0)+(1<<ADPS1)+(1<<ADPS2);  // Set the ADC's clock to system clock / 128
    ADCSRA |= (1<<ADEN);                         // Enable the ADC
    ADCSRA |= (0<<ADATE);
    ADCSRA |= (0<<ADIE);
    ADCSRA |= (0<<ADSC);
    
    stop = false;
}


/**************************************************************************************************
*
*   Retrieve and apply final processing of data ready for reporting
*
*
***************************************************************************************************/


void EmonLibCM_get_readings()
{
    // Copy the variables passed through from the ISR.
    // It is theoretically possible for the values being copied from to be rewritten
    // by the ISR whilst this function is calculating the final values. This second
    // layer of buffering removes that possibility.
    volatile int64_t  protected_sumPA[max_no_of_channels];
    volatile int64_t  protected_sumPB[max_no_of_channels];
    volatile uint64_t protected_sumIsquared[max_no_of_channels];
    volatile int64_t  protected_cumI_deltas[max_no_of_channels];
    cli();
 
    volatile long protected_samplesDuringThisDatalogPeriod = copyOf_samplesDuringThisDatalogPeriod;
    volatile uint64_t protected_sum_Vsquared = copyOf_sum_Vsquared;
    volatile int64_t  protected_cumV_deltas = copyOf_cumV_deltas;  

    for (int i=0; i<no_of_channels; i++) 
    {
        if (ChannelInUse[i])
        {
            protected_sumPA[i] = copyOf_sumPA_CT[i];
            protected_sumPB[i] = copyOf_sumPB_CT[i];
            protected_sumIsquared[i] = copyOf_sumIsquared_CT[i];
            protected_cumI_deltas[i] = copyOf_cumI_deltas[i];
        }
        else
        {
            protected_sumPA[i] = 0;
            protected_sumPB[i] = 0;
            protected_sumIsquared[i] = 0;
            protected_cumI_deltas[i] = 0;
        }
    }           

    if (pulses)                         // if the ISR has counted some pulses, update the total count
    {
        pulseCount += pulses;
        pulses= 0;
    }
    sei();

    // Calculate the final values, scaling for the number of samples and applying calibration coefficients.
    // The final values are deposited in global variables for extraction by the 'getter' functions.
    
    // The rms of a signal plus an offset is  sqrt( signal^2 + offset^2). 
    // Vrms still contains the fine voltage offset. Correct this by subtracting the "Offset V^2" before the sq. root.
    // Real Power is calculated by interpolating between the 'partial power' values, applying "trigonometric" coefficients to
    //  preserve the amplitude of the interpolated value.
    Vrms = sqrt(((double)protected_sum_Vsquared / protected_samplesDuringThisDatalogPeriod)
                - ((double)protected_cumV_deltas * protected_cumV_deltas / protected_samplesDuringThisDatalogPeriod / protected_samplesDuringThisDatalogPeriod)); 
    Vrms *= voltageCal;      

    for (int i=0; i<no_of_channels; i++)    // Current channels
    {
        double powerNow;
        double energyNow;
        double VA;
        int wattHoursRecent;
        double sumRealPower;

        
        // Apply combined phase & timing correction
        sumRealPower = (protected_sumPA[i] * x[i] + protected_sumPB[i] * y[i]); 
                
        // sumRealPower still contains the fine offsets of both V & I. Correct this by subtracting the "Offset Power": cumV_deltas * cumI_deltas
        powerNow = (sumRealPower / protected_samplesDuringThisDatalogPeriod - (double)protected_cumV_deltas * protected_cumI_deltas[i] 
                   / protected_samplesDuringThisDatalogPeriod / protected_samplesDuringThisDatalogPeriod) * voltageCal * currentCal[i];        
      
        //  root of mean squares, removing fine offset
        //  The rms of a signal plus an offset is  sqrt( signal^2 + offset^2). 
        //  Here (signal+offset)^2 = protected_sumIsquared / no of samples
        //       offset = cumI_deltas / no of samples
        Irms_CT[i] = sqrt(((double)protected_sumIsquared[i] / protected_samplesDuringThisDatalogPeriod) - ((double)protected_cumI_deltas[i] * protected_cumI_deltas[i] / protected_samplesDuringThisDatalogPeriod / protected_samplesDuringThisDatalogPeriod)); 
 
        Irms_CT[i] *= currentCal[i];    
    
        VA = Irms_CT[i] * Vrms;
        
        pf[i] = powerNow / VA;
        if (pf[i] > 1.05 || pf[i] < -1.05 || isnan(pf[i]))
          pf[i] = 0.0;
        realPower_CT[i] = powerNow + 0.5;                                                       // rounded to nearest Watt
        apparentPower_CT[i]   = VA + 0.5;                                                       // rounded to nearest VA
        energyNow = (powerNow * datalog_period_in_seconds) + residualEnergy_CT[i];              // fp for accuracy
        wattHoursRecent = energyNow / 3600;                                                     // integer assignment to extract whole Wh
        wh_CT[i]+= wattHoursRecent;                                                             // accumulated WattHours since start-up
        residualEnergy_CT[i] = energyNow - (wattHoursRecent * 3600.0);                          // fp for accuracy
    }
   
    //  Retrieve the temperatures, which should be stored inside each sensor
    if (temperatureEnabled)
    {
        retrieveTemperatures();
    }
   
}

bool EmonLibCM_Ready()
{
    if (startConvertTemperatures)
    {
        startConvertTemperatures = false;
        convertTemperatures();
    }
    
    if (datalogEventPending) 
    {
        datalogEventPending = false;
        
        EmonLibCM_get_readings();
        return true;
    }
    return false;
}


void EmonLibCM_confirmPolarity()
{
    /* This routine prevents a zero-crossing point from being declared until 
    * a certain number of consecutive samples in the 'other' half of the 
    * waveform have been encountered.  It forms part of the ISR.
    */ 
    static byte count = 0;
    
    if (polarityUnconfirmed != polarityConfirmedOfLastSampleV) 
    { 
        count++; 
    } 
    else 
    {
        count = 0; 
    }

    if (count >= POLARITY_CHECK_MAXCOUNT) {
        count = 0;
        polarityConfirmed = polarityUnconfirmed;
    }
}


/**************************************************************************************************
*
*   ADC Interrupt Handling
*
*
***************************************************************************************************/


void EmonLibCM_allGeneralProcessing_withinISR()
{
  if (stop) 
      EmonLibCM_StopADC();
  /* This routine deals with activities that are only required at specific points
   * within each mains cycle.  It forms part of the ISR.
   */ 
  static int cycleCountForDatalogging = 0;
  // a simple routine for checking the performance of this new ISR structure     

  if (acPresent)
  {
      if (polarityConfirmed == POSITIVE) 
      { 
          if (polarityConfirmedOfLastSampleV != POSITIVE)
          {
            /* Instantaneous power contributions are summed in accumulators during each 
             * datalogging period.  At the end of each period, copies are made of their 
             * content for use by the main code.  The accumulators, and any associated
             * counters are then reset for use during the next period.
             */       
            cycleCountForDatalogging ++;
            #ifdef INTEGRITY
                if (sampleSetsDuringThisMainsCycle < lowestNoOfSampleSetsPerMainsCycle)
                {
                    lowestNoOfSampleSetsPerMainsCycle = sampleSetsDuringThisMainsCycle;
                }
                sampleSetsDuringThisMainsCycle = 0;   
            #endif            
           
            // Used in stop start operation, discards the first partial cycle
            if (firstcycle==true && cycleCountForDatalogging >= min_startup_cycles)
            {
                firstcycle = false;
                cycleCountForDatalogging = 0;
                for (int i=0; i<no_of_channels; i++) 
                { 
                  sumPA_CT[i] = 0;
                  sumPB_CT[i] = 0;
                  sumIsquared_CT[i] = 0;
                  cumI_deltas_CT[i] = 0;
                }
                sum_Vsquared = 0;
                cumV_deltas = 0;
    #ifdef INTEGRITY
                lowestNoOfSampleSetsPerMainsCycle = 999;
    #endif          
                samplesDuringThisDatalogPeriod = 0;
            }
          
          // Start temperature conversion
            if (cycleCountForDatalogging == temperatureConversionDelayTime  && firstcycle==false)
            {
                // Only do it on this one cycle
                // convertTemperatures();
                startConvertTemperatures = true;
            }
            if (cycleCountForDatalogging >= datalogPeriodInMainsCycles && firstcycle==false) 
            { 
              cycleCountForDatalogging = 0;    
              for (int i=0; i<no_of_channels; i++) 
              {
                copyOf_sumPA_CT[i] = sumPA_CT[i]; 
                copyOf_sumPB_CT[i] = sumPB_CT[i]; 
                sumPA_CT[i] = 0;
                sumPB_CT[i] = 0;
                copyOf_sumIsquared_CT[i] = sumIsquared_CT[i]; 
                sumIsquared_CT[i] = 0;
                copyOf_cumI_deltas[i] = cumI_deltas_CT[i];
                cumI_deltas_CT[i] = 0;
              }
              copyOf_cumV_deltas = cumV_deltas;
              copyOf_sum_Vsquared = sum_Vsquared;
              sum_Vsquared = 0;
              cumV_deltas = 0;
              copyOf_samplesDuringThisDatalogPeriod = samplesDuringThisDatalogPeriod;
              samplesDuringThisDatalogPeriod = 0;
    #ifdef INTEGRITY
              copyOf_lowestNoOfSampleSetsPerMainsCycle = lowestNoOfSampleSetsPerMainsCycle; // (for diags only)
              lowestNoOfSampleSetsPerMainsCycle = 999;
    #endif
              
              datalogEventPending = true;
              
              // Stops the sampling at the end of the cycle if EmonLibCM_Stop() has been called
              // if (stop) EmonLibCM_StopADC();
            }
          } // end of processing that is specific to the first Vsample in each +ve half cycle   
      } // end of processing that is specific to samples where the voltage is positive
      
      else // the polarity of this sample is negative
      {     
        if (polarityConfirmedOfLastSampleV != NEGATIVE)
        {
          // This is the start of a new -ve half cycle (just after the zero-crossing point)
          //
          samplesDuringThisCycle = 0;    
          // check_RF_LED_status();       
        } // end of processing that is specific to the first Vsample in each -ve half cycle
      } // end of processing that is specific to samples where the voltage is positive
  }
  else
  {    
    // In the case where the voltage signal is missing this part counts ADC samples up to the
    // duration of the datalog period, at which point it will make the readings available.
    // The reporting interval is now dependent on the processor's internal clock

    
    // Start temperature conversion
    if (missing_Voltage > temperatureConversionDelaySamples && convertingTemperaturesNoAC == false)
    {
        // Only do it once per report
        startConvertTemperatures = true;
        convertingTemperaturesNoAC = true;
    }
    
    if (missing_Voltage > ADCsamples_per_datalog_period) 
    {
        missing_Voltage = 0;  // reset the missing samples count here.

        firstcycle = true;    // firstcycle reset to true so that next reading
                              // with voltage signal starts from the right place
        #ifdef INTEGRITY
            lowestNoOfSampleSetsPerMainsCycle = 999;
        #endif          
                          
        cycleCountForDatalogging = 0;    
        for (int i=0; i<no_of_channels; i++) 
        {
            copyOf_sumPA_CT[i] = 0;
            copyOf_sumPB_CT[i] = 0;
            sumPA_CT[i] = 0;
            sumPB_CT[i] = 0;
            copyOf_sumIsquared_CT[i] = sumIsquared_CT[i]; 
            sumIsquared_CT[i] = 0;
            copyOf_cumI_deltas[i] = cumI_deltas_CT[i];
            cumI_deltas_CT[i] = 0;
        }
        copyOf_sum_Vsquared = sum_Vsquared;
        sum_Vsquared = 0;
        copyOf_cumV_deltas = cumV_deltas;
        cumV_deltas = 0;
        copyOf_samplesDuringThisDatalogPeriod = samplesDuringThisDatalogPeriod;
        samplesDuringThisDatalogPeriod = 0;
#ifdef INTEGRITY
        copyOf_lowestNoOfSampleSetsPerMainsCycle = lowestNoOfSampleSetsPerMainsCycle; // (for diags only)
        // lowestNoOfSampleSetsPerMainsCycle = 999;
#endif        

        datalogEventPending = true;

        // Stops the sampling at the end of the cycle if EmonLibCM_Stop() has been called
        // if (stop) EmonLibCM_StopADC();                
    }
  }
 
}
// end of EmonLibCM_allGeneralProcessing_withinISR()

// This Interrupt Service Routine is for use when the ADC is in the free-running mode.
// It is executed whenever an ADC conversion has finished, approx every 104 us.  In 
// free-running mode, the ADC has already started its next conversion by the time that
// the ISR is executed.  The ISR therefore needs to "look ahead". 
//   At the end of conversion Type N, conversion Type N+1 will start automatically.  The ISR 
// which runs at this point therefore needs to capture the results of conversion Type N, 
// and set up the conditions for conversion Type N+2, and so on.  
//   Activities that are required for every new sample are performed here.  Activities
// that are only required at certain stages of the voltage waveform are performed within
// the helper function, EmonLibCM_allGeneralProcessing_withinISR().
//   A second helper function, confirmPolarity() is used to apply a persistence criterion
// when the polarity status of each voltage sample is checked. 
// 
void EmonLibCM_interrupt()  
{                                         
  int rawSample;
  static unsigned char sample_index = 0;
  unsigned char next = 0;
  static int sampleV_minusDC;
  static int lastSampleV_minusDC; 
  int sampleI_minusDC;
  
  static unsigned int acSense = 0;
  
#ifdef SAMPPIN
    digitalWrite(SAMPPIN,HIGH);
#endif
  
  rawSample = ADC;
  next = sample_index + 2;
  if (next>no_of_channels)                 // no_of_channels = count of Current channels in use. Voltage channel (0) is always read, so total is no_of_channels + 1
      next -= no_of_channels+1;
  
  ADMUX = 0x40 + ADC_Sequence[next];       // set up the next-but-one conversion
#ifdef SAMPPIN
    digitalWrite(SAMPPIN,LOW);
#endif
  // Count ADC samples for timing when voltage is unavailable
  missing_Voltage++;

  
  if (sample_index==0)                                               // ADC_Sample 0 is always the voltage channel.
  {
#ifdef SAMPPIN
    digitalWrite(SAMPPIN,HIGH);
#endif
      // Removing the d.c. offset - new method:
      // Rather than use a filter, which takes time to settle and will always contain a residual ripple, and which can lock
      // up under start-up conditions, it is possible to remove the effect of the offset at the final stage of measurement.
      // First take off the theoretical (constant) offset to reduce the size of the numbers (as Robin's original method).
      // Then accumulate the sum of the resulting values so as to be able at the end of the measurement period to 
      // recalculate the true rms based on the rms with the offset and the average remaining offset. The remaining offset 
      // should be only a few counts.
      //
      lastSampleV_minusDC = sampleV_minusDC;                         // required for phaseCal algorithm
      sampleV_minusDC = rawSample - (ADC_Counts >> 1);               // remove nominal offset (a small offset will remain)
      // Detect the ac input voltage. This is a 'rough&ready" rectifier/filter. It only needs to be good enough to detect
      //  sufficient voltage to provide assurance that the crossing detector will function properly
      acSense -= acSense >> 2;
      acSense += sampleV_minusDC > 0 ? sampleV_minusDC : -sampleV_minusDC;
      acPresent = acSense > acDetectedThreshold;
      
      //
      // deal with activities that are only needed at certain stages of each  
      // voltage cycle.

      if (sampleV_minusDC > 0) 
      { 
        polarityUnconfirmed = POSITIVE; 
      }
      else 
      { 
       polarityUnconfirmed = NEGATIVE; 
      }
      EmonLibCM_confirmPolarity();
      EmonLibCM_allGeneralProcessing_withinISR();
      //
      // for real power calculations
#ifdef INTEGRITY
      sampleSetsDuringThisMainsCycle++; 
#endif
      samplesDuringThisDatalogPeriod++;      
      samplesDuringThisCycle++;
      //
      // for the Vrms calculation 
      sum_Vsquared += ((long)sampleV_minusDC * sampleV_minusDC);     // cumulative V^2 (V_ADC x V_ADC)
      //
      // store items for later use
      cumV_deltas += sampleV_minusDC;                                // for use with offset removal

      polarityConfirmedOfLastSampleV = polarityConfirmed;            // for identification of half cycle boundaries
#ifdef SAMPPIN
        digitalWrite(SAMPPIN,LOW);
#endif
      
  }
  
  if (sample_index>=1 && sample_index <= no_of_channels) 
  {
      // Now do much the same for each current sample as it is read.
      // N.B. The Current channels are zero-based but offset by 1 from the ADC sample_index.
      //  That means Current Channel 0 is read from ADC Sample 1. 
      //   ADC_Sample 0 is always the voltage channel, handled in the section above.
      //   ADC_Sample 1 is always a current but not necessarily CT1.
      // Save the current sample for one sample set, so that the calculation normally uses voltage samples
      //   from each side of the current sample for interpolation in the phase shift algorithm.
    
#ifdef SAMPPIN
    digitalWrite(SAMPPIN,HIGH);
#endif

                                          
      static int lastRawSample[max_no_of_channels];  
      if (rawSample > 5)                                                                 // process sample only if a plug is inserted
      {
        ChannelInUse[sample_index-1] = true;
       
        // Offset removal for current is the same as for the voltage.
      
        lastRawSample[sample_index-1] -= (ADC_Counts >> 1);                              // remove nominal offset (a small offset will remain)
       
        sampleI_minusDC = lastRawSample[sample_index-1];
              
        // calculate the "partial real powers" in this sample pair and add to the accumulated sums - fine d.c. offsets are still present
        sumPA_CT[sample_index-1] += (long)sampleI_minusDC * lastSampleV_minusDC;         // cumulative power A
        sumPB_CT[sample_index-1] += (long)sampleI_minusDC * sampleV_minusDC;             // cumulative power B
          
        // for Irms calculation 
        sumIsquared_CT[sample_index-1] += (long)sampleI_minusDC * sampleI_minusDC;       // this has the fine d.c. offset still present
        cumI_deltas_CT[sample_index-1] += sampleI_minusDC;                               // for use with offset removal

      }
      else
          ChannelInUse[sample_index-1] = false;
      
      lastRawSample[sample_index-1] = rawSample;                                         // Delay everything by 1 sample
#ifdef SAMPPIN      
      digitalWrite(SAMPPIN,LOW);
#endif
  }
  
  sample_index++; // advance the control flag
  if (sample_index>no_of_channels) sample_index = 0;
}

/**************************************************************************************************
*
*   TEMPERATURES
*
*
***************************************************************************************************/


void EmonLibCM_setTemperatureDataPin(byte _dataPin)
{
    W1Pin = _dataPin;
}


void EmonLibCM_setTemperaturePowerPin(char _powerPin)
{
    DS18B20_PWR = _powerPin;
    if (DS18B20_PWR >= 0)
    {
        pinMode(DS18B20_PWR, OUTPUT);  
        digitalWrite(DS18B20_PWR, HIGH); 
    }
}


void EmonLibCM_setTemperatureResolution(byte _resolution)
{
    switch (_resolution)
    {
        case (9):  temperatureResolution = TEMPRES_9;
                   break;
        case (10): temperatureResolution = TEMPRES_10;
                   break;
        case (12): temperatureResolution = TEMPRES_12;
                   break;
        default:   temperatureResolution = TEMPRES_11;
                   break;
    }
}
    
    
void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray)
{
    temperatureSensors = addressArray; 
    temperatureSensors[0][0] = 0x00;            // Used by "TemperatureEnable" to trigger search for sensors
}


void EmonLibCM_setTemperatureAddresses(DeviceAddress *addressArray, bool keep)
{
    temperatureSensors = addressArray; 
    if (!keep)
        temperatureSensors[0][0] = 0x00;        // Used by "TemperatureEnable" to trigger search for sensors
}


void EmonLibCM_setTemperatureArray(int *temperatureArray)
{
    temperatures = temperatureArray;
}


void EmonLibCM_setTemperatureMaxCount(int _maxCount)
{
    temperatureMaxCount = _maxCount;
}


void EmonLibCM_TemperatureEnable(bool _enable)
{
  //Setup and test for presence of DS18B20s, fill address array, set device resolution & write to EEPROM

  
    if ( temperatureSensors == NULL || temperatures == NULL)
    {
        temperatureEnabled = false;                     // Could corrupt memory, try to limit damage
        numSensors = 0;
        return;                                         // & quit.
    }

    if (temperatureEnabled = _enable)
    {
        if (datalog_period_in_seconds < 1.0)            // Not enough time to convert between samples.
            temperatureResolution = TEMPRES_9;
        byte scratchpad[3];
        scratchpad[0] = 0;                              // low alarm
        scratchpad[1] = 0;                              // high alarm
        scratchpad[2] = temperatureResolution;          // resolution

        for(byte j=0; j<temperatureMaxCount; j++)
           temperatures[j] = UNUSED_TEMPERATURE;        // write 'Sensor never seen' marker to temperatures array

        sensors.begin();
        numSensors=(sensors.getDeviceCount()); 
        if (numSensors > temperatureMaxCount)
            numSensors = temperatureMaxCount;
        byte j=0;                                       // search for one wire devices and copy to device address array.
       
        if (temperatureSensors[0][0] != 0x28)           // 0x28 = signature of a DS18B20, so a pre-existing array - do not search for sensors
            while ((j < numSensors) && (oneWire.search(temperatureSensors[j]))) 
                j++;
        oneWire.reset();                                // write resolution to scratchpad 
        oneWire.write(SKIP_ROM);
        oneWire.write(WRITE_SCRATCHPAD);
        for(int i=0; i<3; i++)
            oneWire.write(scratchpad[i]);
        oneWire.reset();                                // copy to EEPROM 
        oneWire.write(SKIP_ROM);
        oneWire.write(COPY_SCRATCHPAD, true);
        delay(20);                                      // required by DS18B20
    }
    // Calculate number of cycles (or in the absence of ac, no. of samples) to allow after datalogEventPending has been set
    //  to true, so that temperature conversion will complete just before the next datalog event. Adjust the resolution if necessary
    //  so that conversion within one datalogging period is possible.
   
    int conversionLeadTime = (CONVERSION_LEAD_TIME >> (3 - ((temperatureResolution & 0x70) >> 5)));  
        // Should give 95 - 760 ms lead time, now convert to cycles (for a.c. present) or samples (for a.c. not present).
    temperatureConversionDelayTime = datalogPeriodInMainsCycles - (long)conversionLeadTime * cycles_per_second / 1000 - 1; 
        // '-1' to counter the effect of integer truncation, and make sure there is some spare time

    temperatureConversionDelaySamples = ((unsigned long)(datalog_period_in_seconds * 1000.0) 
        - (unsigned long)conversionLeadTime - 5) * 1000 / ADCDuration;
        // '- 5' extra 5 ms to make sure there is some spare time
    
}

void printTemperatureSensorAddresses(void)
{
    Serial.print("Temperature Sensors found = ");
    Serial.print(numSensors);
    Serial.print(" of ");
    Serial.print(temperatureMaxCount);
    
    if (numSensors)
    {
        Serial.println(", with addresses...");
            for (int j=0; j< numSensors; j++)
            {
                for (int i=0; i<8; i++)
                {
                    Serial.print(temperatureSensors[j][i], 16);
                    Serial.print(" ");
                }
                Serial.println();
                delay(5);
            }
    }
    Serial.println();
    Serial.print("Temperature measurement is");
    Serial.print(temperatureEnabled?"":" NOT");
    Serial.println(" enabled.");
    Serial.println();
    delay(5);
        
}

void convertTemperatures(void)
{
    if (temperatureEnabled)
    {
        if (DS18B20_PWR >= 0)
        {
            pinMode(DS18B20_PWR, OUTPUT);  
            digitalWrite(DS18B20_PWR, HIGH); 
        }
        oneWire.reset();
        oneWire.write(SKIP_ROM);
        oneWire.write(CONVERT_TEMPERATURE, true); 
    }        // start conversion - all sensors    
}


void retrieveTemperatures(void)
{
    if (temperatureEnabled)
    {
        for (byte j=0; j < numSensors; j++)
        {
            byte buf[9];
            int result;

            if (!oneWire.reset())
            {
                result=BAD_TEMPERATURE;
                break;
            }
            else
            {
                oneWire.write(MATCH_ROM);
                for(int i=0; i<8; i++) 
                    oneWire.write(temperatureSensors[j][i]);
                oneWire.write(READ_SCRATCHPAD);
                for(int i=0; i<9; i++) 
                    buf[i] = oneWire.read();
            }

            if(oneWire.crc8(buf,8)==buf[8])
            {
                result=(buf[1]<<8)|buf[0];
                // result is temperature x16, multiply by 6.25 to convert to temperature x100
                result=(result*6)+(result>>2);
            }
            else result=BAD_TEMPERATURE;
            if (result != BAD_TEMPERATURE && (result < -5500 || result > 12500))
                result = OUTOFRANGE_TEMPERATURE;
            temperatures[j] = result;
            delay(5);
        }
        if (DS18B20_PWR >= 0)
        {
            pinMode(DS18B20_PWR, OUTPUT);  
            digitalWrite(DS18B20_PWR, LOW); 
        }
        convertingTemperaturesNoAC = false;
    }
}


int EmonLibCM_getTemperatureSensorCount(void)
{
    if (temperatureEnabled)
        return(numSensors); 
    else
        return 0;
}

bool EmonLibCM_getTemperatureEnabled(void)
{
    return temperatureEnabled;
}


float EmonLibCM_getTemperature(char sensorNumber)
{
    int temp = temperatures[(unsigned char)sensorNumber];
    
    if (sensorNumber < 0 || sensorNumber >= temperatureMaxCount)
        return UNUSED_TEMPERATURE/100.0;
    if (temp >=30000)
        return temp/100.0;                                      // 300 series error codes are already set
    return (temp/100.0);
}

/**************************************************************************************************
*
*   PULSE INPUT ISR
*
*
***************************************************************************************************/


ISR(ADC_vect) 
{
    EmonLibCM_interrupt();
}

// The pulse interrupt routine - runs each time a falling (leading) edge of a pulse is detected
void onPulse()                  
{
    if (PulseMinPeriod)
    {
      if ((millis() - pulseTime) > PulseMinPeriod) {              // Check that contact bounce has finished
        pulses++;
      }
      pulseTime=millis();                                         // No 'debounce' required - electronic switch presumed    
    }
    else
        pulses++;                   
}

 
