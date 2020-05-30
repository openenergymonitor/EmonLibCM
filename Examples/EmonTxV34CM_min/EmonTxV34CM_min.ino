/*

"Minimal" sketch to demonstrate emonLibCM

This sketch assumes that the default values for the emonTx V3.4 are 
applicable, that no input calibration is required, mains frequency 
is 50 Hz and data logging period interval is 10 s, pulse counting 
and temperature monitoring are not required, and that 4 'standard' 
100 A CTs and the UK a.c. adapter from the OEM Shop are being used.

*/
#include <Arduino.h>
#include "emonLibCM.h"

#define RF69_COMPAT 1                                      //  Set to 1 if using RFM69CW, or 0 if using RFM12B
#include <JeeLib.h>                                        //  https://github.com/jcw/jeelib - Tested with JeeLib 10 April 2017
// ISR(WDT_vect) { Sleepy::watchdogEvent(); } 

#define RF_freq RF12_433MHZ                                //  Frequency of radio module can be RF12_433MHZ, RF12_868MHZ or RF12_915MHZ. 
                                                           //  - You must use the one matching the module you have.
const int nodeID = 10;                                     //  node ID for this emonTx. This sketch does NOT interrogate the DIP switch.

const int networkGroup = 210;                              //  wireless network group
                                                           //  - needs to be same as emonBase / emonPi and emonGLCD. OEM default is 210

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

typedef struct {int power1, power2, power3, power4, Vrms, T1, T2, T3, T4, T5, T6; unsigned long pulseCount; } PayloadTX;        // package the data for RF comms

PayloadTX emontx;                                          // create an instance

 
void setup() 
{  

  Serial.begin(9600);
  Serial.println("Set baud=115200");
  Serial.end();
  Serial.begin(115200);
  
  Serial.println("\nEmonTx v3.4 EmonLibCM Continuous Monitoring Minimal Demo"); 

  rf12_initialize(nodeID, RF_freq, networkGroup);          // initialize radio module

  EmonLibCM_Init();                                        // Start continuous monitoring.

}

void loop()             
{

  if (EmonLibCM_Ready())   
  {
 
    Serial.println(EmonLibCM_acPresent()?"AC present ":"AC missing ");
    delay(5);

    emontx.power1 = EmonLibCM_getRealPower(0);             // Copy the desired variables ready for transmission
    emontx.power2 = EmonLibCM_getRealPower(1);
    emontx.power3 = EmonLibCM_getRealPower(2);
    emontx.power4 = EmonLibCM_getRealPower(3);
    emontx.Vrms   = EmonLibCM_getVrms() * 100;
    
    rf12_sendNow(0, &emontx, sizeof emontx);               //send data

    delay(50);
 
    Serial.print(" V=");Serial.println(EmonLibCM_getVrms());

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

    delay(10);
        

  }
}
