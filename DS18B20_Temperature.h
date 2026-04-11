/*
 * Read the temperature of one or more precise sensors
*/

#define ONE_WIRE_BUS 32
#define AMOUNT_DS18B20 2 // amount of possible connection

#include "OneWire.h"
#include "DallasTemperature.h"

#define RESOLUTION_DS18B20 12
DeviceAddress DS18B20_Address;
 
OneWire oneWire(ONE_WIRE_BUS); 
DallasTemperature myDS18B20(&oneWire); 
 

float Fallback_Value = -127.8;
static float Temperatur[AMOUNT_DS18B20]; 
// float TemperaturFallback[AMOUNT_DS18B20]; 
int amountOfReallyConnectedSensors = 0;

void initTempSensor(void) {
  L_PRINTLN("-initTempSensor-----------------");
  
  if ((AMOUNT_DS18B20 > 0)) {
    myDS18B20.begin();
    L_PRINT("Amount of connected External Temperature Sensors: ");
    amountOfReallyConnectedSensors = myDS18B20.getDeviceCount();
    L_PRINTLN(String(amountOfReallyConnectedSensors));
 
    for(byte i=0 ;i < amountOfReallyConnectedSensors; i++) {
      // TemperaturFallback[i] = Fallback_Value;
      if(myDS18B20.getAddress(DS18B20_Address, i)) {
        myDS18B20.setResolution(DS18B20_Address, RESOLUTION_DS18B20);
      }
    }
  }
}

void readExternalTemperatureSensors(void) {
  L_PRINTLN("-readTempSensor-External Temperature-------------");
  if ((AMOUNT_DS18B20 > 0)) {
    myDS18B20.requestTemperatures();
    for(byte i=0 ;i < AMOUNT_DS18B20; i++) {
      if (i < amountOfReallyConnectedSensors) {
        
        Temperatur[i] = myDS18B20.getTempCByIndex(i);
        if (Temperatur[i] == DEVICE_DISCONNECTED_C) {
          // if error occures, set a value as fallback
          L_PRINTLN("ERROR: DS18B20 Tempsensor read error, set dummy value -127.8");
          Temperatur[i] = -127.8;
        }
      }
    }
  }
}

int getSensorAmount() {
  return amountOfReallyConnectedSensors;
}

float * getTemperatureValues(void){
  return Temperatur;
}