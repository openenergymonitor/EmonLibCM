/*

"Maximal" sketch to demonstrate emonLibCM

This demonstrates the use of every API function.
This sketch provides an example of every Application Interface function. 
Many in fact set the default value for the emonTx V3.4, and are therefore
not needed in most cases. If you do need to change a value, the 
Application Interface section of the User Documentation gives full details.

*/

#include <Arduino.h>
#include "emonLibCM.h"

#define RF69_COMPAT 1                                      //  Set to 1 if using RFM69CW, or 0 if using RFM12B
#include <JeeLib.h>                                        //  https://github.com/jcw/jeelib - Tested with JeeLib 10 April 2017
// ISR(WDT_vect) { Sleepy::watchdogEvent(); } 

#define RF_freq RF12_433MHZ                                //  Frequency of radio module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. 
                                                           //    You must use the one matching the module you have.
const int nodeID = 10;                                     //  node ID for this emonTx. This sketch does NOT interrogate the DIP switch.

const int networkGroup = 210;                              //  wireless network group
                                                           //  - needs to be same as emonBase / emonPi and emonGLCD. OEM default is 210
bool recalibrate = false;                                  //  Do not demonstrate the recalibration functions
/*

emonhub.conf nodeid is 10 - switch is ignored)
See: https://github.com/openenergymonitor/emonhub/blob/emon-pi/configuration.md

[[10]]
    nodename = emontx1
    [[[rx]]]
       names = power1, power2, power3, power4, vrms, temp1, temp2, temp3, temp4, temp5, temp6, pulse
       datacode = h
       scales = 1,1,1,1,0.01,0.1,0.1,0.1,0.1,0.1,0.1,1
       units =W,W,W,W,V,C,C,C,C,C,C,p

*/       

typedef struct {int power1, power2, power3, power4, Vrms, T1, T2, T3, T4, T5, T6, P; } PayloadTX;        // package the data for RF comms

PayloadTX emontx;                                          // create an instance

 
DeviceAddress allAddresses[6];                             // Array to receive temperature sensor addresses
/* Example - how to define temperature sensors, prevents an automatic search

DeviceAddress allAddresses[6] = {       
    {0x28, 0x81, 0x43, 0x31, 0x7, 0x0, 0x0, 0xD9}, 
    {0x28, 0x8D, 0xA5, 0xC7, 0x5, 0x0, 0x0, 0xD5},         // Use the actual addresses, as many as required
    {0x28, 0xC9, 0x58, 0x32, 0x7, 0x0, 0x0, 0x89}          //  up to a maximum of 6
};

*/
int allTemps[6];                                           // Array to receive temperature measurements

void setup() 
{  

  Serial.begin(9600);
  Serial.println("Set baud=115200");
  Serial.end();
  Serial.begin(115200);
  
  Serial.println("\nEmonTx v3.4 EmonLibCM Continuous Monitoring Maximal Demo"); 
  Serial.print("\nAssumed voltage for apparent power calculations when no a.c. is detected: ");  Serial.println(EmonLibCM_getAssumedVrms()); 
  Serial.print("\nValues will be reported every ");  Serial.print(EmonLibCM_getDatalog_period()); Serial.println(" seconds");
  
  EmonLibCM_SetADC_VChannel(0, 268.97);                    // ADC Input channel, voltage calibration - for Ideal UK Adapter = 268.97 
  EmonLibCM_SetADC_IChannel(1, 90.91, 4.2);                // ADC Input channel, current calibration, phase calibration
  EmonLibCM_SetADC_IChannel(2, 90.91, 4.2);                //  The current channels will be read in this order
  EmonLibCM_SetADC_IChannel(3, 90.91, 4.2);                //  90.91 for 100 A : 50 mA c.t. with 22R burden - v.t. leads c.t by ~4.2 degrees
  EmonLibCM_SetADC_IChannel(4, 16.67, 1.0);                //  16.67 for 100 A : 50 mA c.t. with 120R burden - v.t. leads c.t by ~1 degree

  EmonLibCM_setADC(10, 104);                               // ADC Bits (10 for emonTx & Arduino except Due=12 bits, ADC Duration 104 us for 16 MHz operation)
  EmonLibCM_ADCCal(3.3);                                   // ADC Reference voltage, (3.3 V for emonTx,  5.0 V for Arduino)
  
  EmonLibCM_setAssumedVrms(240.0);                         // Assumed voltage when no a.c. detected 
  EmonLibCM_cycles_per_second(50);                         // mains frequency 50Hz, 60Hz
  EmonLibCM_datalog_period(10);                            // period of readings in seconds - normal value for emoncms.org
  
  EmonLibCM_min_startup_cycles(10);                        // number of cycles to let ADC run before starting first actual measurement

  EmonLibCM_setPulseEnable(true);                          // Enable pulse counting. See the documentation for 2-channel versions of these functions.
  EmonLibCM_setPulsePin(3, 1);
  EmonLibCM_setPulseMinPeriod(20, (byte)FALLING);          // 20 ms debounce period, count on falling edge (e.g. switch closing)
  EmonLibCM_setPulseCount(0);                              // Initialise to pulse count to zero

  EmonLibCM_setWattHour(0, 0);                             // Wh counters set to zero
  EmonLibCM_setWattHour(1, 0);
  EmonLibCM_setWattHour(2, 0);
  EmonLibCM_setWattHour(3, 0);

  EmonLibCM_setTemperatureDataPin(5);                      // OneWire data pin (emonTx V3.4)
  EmonLibCM_setTemperaturePowerPin(19);                    // Temperature sensor Power Pin - 19 for emonTx V3.4  (-1 = Not used. No sensors, or sensor are permanently powered.)
  EmonLibCM_setTemperatureResolution(11);                  // Resolution in bits, allowed values 9 - 12. 11-bit resolution, reads to 0.125 degC
  EmonLibCM_setTemperatureAddresses(allAddresses);         // Name of array of temperature sensors
  EmonLibCM_setTemperatureArray(allTemps);                 // Name of array to receive temperature measurements
  EmonLibCM_setTemperatureMaxCount(6);                     // Max number of sensors, limited by wiring and array size.
  
  EmonLibCM_TemperatureEnable(true); 
  printTemperatureSensorAddresses();                       // Show which sensors are connected


  rf12_initialize(nodeID, RF_freq, networkGroup);          // initialize radio module

  EmonLibCM_Init();                                        // Start continuous monitoring.

}

void loop()             
{

  if (recalibrate)                                         // recalibrate should be set when new calibration values become available
  {
      
    EmonLibCM_ReCalibrate_VChannel(268.97);                // ADC Input channel, voltage calibration new value 
    EmonLibCM_ReCalibrate_IChannel(1, 90.91, 4.2);         // ADC Input channel, current calibration, phase calibration new values
    EmonLibCM_ReCalibrate_IChannel(2, 90.91, 4.2);         //  It is only necessary to use one of these functions if that calibration 
    EmonLibCM_ReCalibrate_IChannel(3, 90.91, 4.2);         //  value needs to be changed.
    EmonLibCM_ReCalibrate_IChannel(4, 16.67, 1.0);         //  
    recalibrate = false;                                   // Do it once only.
  }
  
  if (EmonLibCM_Ready())   
  {

    Serial.println(EmonLibCM_acPresent()?"AC present ":"AC missing ");
    delay(5);

    emontx.power1 = EmonLibCM_getRealPower(0);   // Copy the desired variables ready for transmission 
    emontx.power2 = EmonLibCM_getRealPower(1); 
    emontx.power3 = EmonLibCM_getRealPower(2);
    emontx.power4 = EmonLibCM_getRealPower(3);
    emontx.Vrms   = EmonLibCM_getVrms() * 100;

    emontx.T1 = allTemps[0];
    emontx.T2 = allTemps[1];
    emontx.T3 = allTemps[2];
    emontx.T4 = allTemps[3];
    emontx.T5 = allTemps[4];
    emontx.T6 = allTemps[5];
    
    emontx.P  = EmonLibCM_getPulseCount();
   
    rf12_sendNow(0, &emontx, sizeof emontx);     //send data

    delay(50);

    Serial.print(" V=");Serial.print(EmonLibCM_getVrms());
    Serial.print(" f=");Serial.println(EmonLibCM_getLineFrequency(),2);           

    for (byte ch=0; ch<4; ch++)
    {
        Serial.print("Ch ");Serial.print(ch+1);
        Serial.print(" I=");Serial.print(EmonLibCM_getIrms(ch),3);
        Serial.print(" W=");Serial.print(EmonLibCM_getRealPower(ch));
        Serial.print(" VA=");Serial.print(EmonLibCM_getApparentPower(ch));
        Serial.print(" Wh=");Serial.print(EmonLibCM_getWattHour(ch));
        Serial.print(" pf=");Serial.print(EmonLibCM_getPF(ch),4);   
        Serial.println();
        delay(10);
    } 

    Serial.print(" pulses=");Serial.println(EmonLibCM_getPulseCount());
    delay(10);       

    if (EmonLibCM_getTemperatureEnabled())
    {
        Serial.println("Temperatures:");
        for (byte j=0; j<EmonLibCM_getTemperatureSensorCount(); j++)
        {
            Serial.print("  ");
            Serial.print(j+1);
            Serial.print(" = ");
            Serial.println(EmonLibCM_getTemperature(j),4);
            delay(10);
        }   
    }
  }
  else
    rf12_recvDone();

}
