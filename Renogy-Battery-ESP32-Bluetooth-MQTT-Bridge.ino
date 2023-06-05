/**
 * A BLE bridge from Renogy battery to MQTT
 * based on excellence work from: https://github.com/chadj/renogy-smart-battery
 */

#include "config.h"
#include <NimBLEDevice.h>
#include "wifiBridge.h"
#include "DHT.h"

#define DHT11PIN 15

DHT dht(DHT11PIN, DHT11);

#define RENOGYHEADERSIZE 3 // drop first 3 bytes of response

// The remote service, we wish to connect to.
static BLEUUID serviceWriteUUID("0000ffd0-0000-1000-8000-00805f9b34fb"); // WRITE
static BLEUUID serviceReadUUID("0000fff0-0000-1000-8000-00805f9b34fb"); // READ

static BLEUUID WRITE_UUID("0000ffd1-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

String callData = "getLevels";
String responseData = "";
String RENOGYpower="";
String RENOGYcurrent="";
String RENOGYvoltage="";
String RENOGYcurrentDebug="0";
String RENOGYvoltageDebug="0";
String RENOGYchargeLevel="";
String RENOGYcapacity="";
String RENOGYtemperature="";
String RENOGYtimer="00:00";
String wifiSSIDValue="noSSID";
String actualTimeStamp="00:00:00";
float DHThumi;
float DHTtemp;
float DHTheatindex;
uint8_t firstRun = 1;
int flexiblePollingSpeed = 20000;
uint16_t timerCounterStart = 0;
uint16_t timerCounterActual = 0;
boolean timerIsRunning = false;

static uint32_t timerTickerDisplay = millis();
uint16_t whatchDogTicks = 0;

static boolean tryReconnect = false;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static uint32_t timerTickerForWhatchDog = millis();
static uint32_t timerTickerForEggTimer = millis();
static uint32_t timerTicker2 = millis();

BLERemoteService* pRemoteWriteService;
BLERemoteService* pRemoteReadService;
BLERemoteCharacteristic* pRemoteWriteCharacteristic;
BLERemoteCharacteristic* pRemoteNotifyCharacteristic;
// BLEAdvertisedDevice* myDevice;

BLEClient* pClient;
// BLEScan* pBLEScan;

#define DEVICEAMOUNT 2

// Address of my BT battery devices
static const char* deviceAddresses[DEVICEAMOUNT] = {
  "60:98:66:ed:cb:8b",
  "60:98:66:f9:3a:0f"
};
static float current[DEVICEAMOUNT] = {
  0,0
};
static double voltage = 0.0;
static int16_t power[DEVICEAMOUNT] = {
  0,0
};
uint8_t deviceAddressesNumber=0;

int mqtt_server_count = sizeof(mqtt_server) / sizeof(mqtt_server[0]);
//Address of the peripheral device. Address will be found during scanning...
// static BLE pServerAddress;
TaskHandle_t Task1;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {

    // Serial.println("#####################");
    // Serial.println("Get notification from BLE device:");

    uint32_t tempvalueI;

    if(responseData=="getLevels") {
      // int16_t valueSigned;
      // we get Current as signed 2 bytes
      int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE+0] << 8) | pData[RENOGYHEADERSIZE+1];
      RENOGYcurrent = String((float)valueSigned * 0.01);

      // we get voltage as uint 2 bytes
      tempvalueI = ((int16_t)pData[RENOGYHEADERSIZE+2] << 8) | pData[RENOGYHEADERSIZE+3];
      RENOGYvoltage = tempvalueI * 0.1;

      // we get lavel as uint 4 bytes
      tempvalueI = ((uint8_t)pData[RENOGYHEADERSIZE+4] << 24) | ((uint8_t)pData[RENOGYHEADERSIZE+5] << 16) | ((uint8_t)pData[RENOGYHEADERSIZE+6] << 8) | (uint8_t)pData[RENOGYHEADERSIZE+7];
      RENOGYchargeLevel = tempvalueI * 0.001;

      // we get capacity as uint 4 bytes
      tempvalueI = ((uint8_t)pData[RENOGYHEADERSIZE+8] << 24) | ((uint8_t)pData[RENOGYHEADERSIZE+9] << 16) | ((uint8_t)pData[RENOGYHEADERSIZE+10] << 8) | (uint8_t)pData[RENOGYHEADERSIZE+11];
      RENOGYcapacity = tempvalueI * 0.001;

      // Serial.println("Get Levels ########");
      // Serial.println("Current:");
      // Serial.println(RENOGYcurrent);
      // Serial.println("Voltage:");
      // Serial.println(RENOGYvoltage);
      // Serial.println("RENOGYchargeLevel:");
      // Serial.println(RENOGYchargeLevel);
      // Serial.println("RENOGYcapacity:");
      // Serial.println(RENOGYcapacity);

      // for debug, no hardware is needed: s for start, e for end
      if (RENOGYcurrentDebug!="0") {
        RENOGYvoltage = RENOGYvoltageDebug;
        RENOGYcurrent = RENOGYcurrentDebug;
      }

      // compareValuesForTimer();
      updateEggTimer();
      calculatePower();

      callData="getTemperatures";
      flexiblePollingSpeed = 2000; // next call for data in 2s
    }

    if(responseData=="getTemperatures") {
      uint8_t numberSensors = ((int16_t)pData[RENOGYHEADERSIZE+0] << 8) | pData[RENOGYHEADERSIZE+1];

      // Serial.println("Get Temperatures ########");

      int16_t averageTemp = 0;
      for (int i=1; i<=numberSensors; i++){
        int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE+(2*i)] << 8) | pData[RENOGYHEADERSIZE+1+(2*i)];
        averageTemp += valueSigned;
      }

      RENOGYtemperature = String((float)(averageTemp/numberSensors) * 0.1);

      // Serial.println("Temperature");
      // Serial.println(RENOGYtemperature);

      flexiblePollingSpeed = 20000; // next call for host switch in 20s
      callData="connectToAnotherHost";
      // callData="getLevels";
    }

    if(responseData=="getCellVolts") {
      // uint8_t numberSensors = ((int16_t)pData[RENOGYHEADERSIZE+0] << 8) | pData[RENOGYHEADERSIZE+1];

      Serial.println("Get Cell Volts ########");
      // int16_t averageTemp = 0;
      // for (int i=1; i<=numberSensors; i++){
      //   int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE+(2*i)] << 8) | pData[RENOGYHEADERSIZE+1+(2*i)];
      //   averageTemp += valueSigned;
      // }

      // RENOGYtemperature = String((float)(averageTemp/numberSensors) * 0.1);
      Serial.println("getCellVolts");
      // Serial.println(RENOGYtemperature);

      callData="getLevels";
    }
    /* pData Debug... */
    // Serial.println("Hex data received:"); 
    // for (int i=1; i<=length; i++){
    //   Serial.printf("%02x", pData[i-1]);
    //   if(i % 2 == 0){
    //     Serial.print(" "); 
    //   }
    // }
    Serial.println(" "); 
    Serial.println("END notifyCallback ########");
    delay(5);
    sendMqttData();
}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

void updateEggTimer() {
  if (timerIsRunning) {
    char buffer[5];
    timerCounterActual = getEpochTime() - timerCounterStart;
    Serial.println("timerIsRunning: timerCounterActual");
    Serial.println(timerCounterActual);
    sprintf (buffer, "%02u:%02u", timerCounterActual / 60, timerCounterActual % 60);
    Serial.println(buffer);
    RENOGYtimer = buffer;
    RENOGYtimer.remove(5);
  }
}

void compareValuesForTimer() {
  Serial.println("compareValuesForTimer");
  Serial.printf("dVoltage: %f dCurrent: %f Power %f ISRunning: %s", 
    (RENOGYvoltage.toFloat()), RENOGYcurrent.toFloat(),
    power[deviceAddressesNumber], timerIsRunning?"true":"false"
  );
  Serial.println("");
  Serial.printf("dVoltage: %f dCurrent: %f Power %f ISRunning: %s", 
    ((float)voltage - RENOGYvoltage.toFloat()), abs(current[deviceAddressesNumber] - RENOGYcurrent.toFloat()),
    power[deviceAddressesNumber], timerIsRunning?"true":"false"
  );
  Serial.println("");
  if (timerIsRunning) {
    // buffer 90s, till first attempt to stop timer. to avoid issues while fetching data from other device
    if (
        (abs(power[deviceAddressesNumber]) <= 100 && timerCounterActual > 90)
         ||
         // force stop of timer, after 1800s/30min in case it is still running
         timerCounterActual > 1800
        ) {
      timerIsRunning = false;
      RENOGYtimer = "00:00";
    }
  }

  if (!timerIsRunning) {
    // check if there is a massive voltage drop between 2 measurements
    // 13.5 - 13.1 = 0.4
    if ((float)voltage - RENOGYvoltage.toFloat() >= 0.1) {
      // check if there is also massive current drop between 2 measurements
      if (abs(current[deviceAddressesNumber] - RENOGYcurrent.toFloat()) >= 10) {
        timerCounterStart = getEpochTime();
        timerIsRunning = true;
        
        // for interpolation, set current equal in every device
        for (int i = 0; i < DEVICEAMOUNT; i++){
          current[deviceAddressesNumber] = RENOGYcurrent.toFloat();
        }
      }
    }
  }

  voltage = RENOGYvoltage.toFloat();
  // current[deviceAddressesNumber] = RENOGYcurrent.toFloat();
  // power[deviceAddressesNumber] = RENOGYcurrent.toFloat()*RENOGYvoltage.toFloat();

  // RENOGYpower = "0";
  // int powerTemp = 0;
  // for (int i = 0; i < DEVICEAMOUNT; i++)
  // {
  //   Serial.print("current:");
  //   Serial.println(i);
  //   Serial.println(current[i]);
  //   Serial.print("power:");
  //   Serial.println(i);
  //   Serial.println(power[i]);
  //   powerTemp += power[i];
  // }
  // RENOGYpower = String(powerTemp);
}

void calculatePower() {
  current[deviceAddressesNumber] = RENOGYcurrent.toFloat();
  power[deviceAddressesNumber] = RENOGYcurrent.toFloat()*RENOGYvoltage.toFloat();

  RENOGYpower = "0";
  int powerTemp = 0;
  Serial.println("");
  for (int i = 0; i < DEVICEAMOUNT; i++)
  {
    Serial.print(i);
    Serial.print(" current:");
    Serial.print(current[i]);
    Serial.print(" power:");
    Serial.print(power[i]);
    powerTemp += power[i];
  }
  Serial.println("");
  RENOGYpower = String(powerTemp);
}

bool connectToServer() {
    callData = "getLevels";
    Serial.print("Forming a connection to ");
    // Serial.println(myDevice->getAddress().toString().c_str());
    Serial.println(deviceAddresses[deviceAddressesNumber]);
    
    BLEDevice::setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
    pClient = NimBLEDevice::createClient(NimBLEAddress(deviceAddresses[deviceAddressesNumber]));
    Serial.println(" - Created client");
    // delay(700);
    pClient->setClientCallbacks(new MyClientCallback());
    // delay(700);
    // Connect to the remove BLE Server.
    pClient->connect();  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    delay(700);
    // pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    pRemoteWriteService = pClient->getService(serviceWriteUUID);
    // if (true) {
    if (pRemoteWriteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceWriteUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our pRemoteWriteService");
    pRemoteReadService = pClient->getService(serviceReadUUID);
    if (pRemoteReadService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceReadUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our pRemoteReadService");
    // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteWriteCharacteristic = pRemoteWriteService->getCharacteristic(WRITE_UUID);
    if (pRemoteWriteCharacteristic == nullptr) {
      Serial.print(F("Failed to find our characteristic UUID: "));
      Serial.println(WRITE_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(F(" - Found our Write characteristic"));
        // Obtain a reference to the characteristic in the service of the remote BLE server.
    pRemoteNotifyCharacteristic = pRemoteReadService->getCharacteristic(NOTIFY_UUID);
    if (pRemoteNotifyCharacteristic == nullptr) {
      Serial.print(F("Failed to find our characteristic UUID: "));
      Serial.println(NOTIFY_UUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(F(" - Found our characteristic for notifications"));

    if(pRemoteNotifyCharacteristic->canNotify()) {
      Serial.println("Subscribe to characteristic...");
      pRemoteNotifyCharacteristic->registerForNotify(notifyCallback);
    }

    // BLEDevice::getScan()->clearResults();
    connected = true;
    flexiblePollingSpeed = 6000; // next call for data in 2s
    return true;
}

void setupDeviceAndConnect() {
  doConnect = true;
  connected = false;
  doScan = true;
  delay(1000);

  BLEDevice::init("client");
}


void setup() {
  xTaskCreatePinnedToCore(
    myWhatchdog,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */    

  Serial.begin(19200);
  Serial.println("Starting Arduino BLE Client application...");

  if (startWiFiMulti()) {
    Serial.println("Wifi connected make next step...");
    Serial.println();  

    setClock();
    if ( startMQTT()) {
      wifiSSIDValue = WiFi.SSID();
      wifiSSIDValue = wifiSSIDValue + " " + WiFi.localIP().toString();
      Serial.println(WiFi.localIP());
      delay(1000);
      setupDeviceAndConnect();
      return;
    }

  }

  Serial.println("");
  Serial.println("Wait 30s and than restart...");
  getClockTime();
  delay(30000);
  ESP.restart();
} // End of setup.


// This is the Arduino main loop function.
void loop() {
  espMQTT.update();  // should be called

  if(Serial.available()){
    char charE = Serial.read();
    if(charE == 's') {
      RENOGYcurrentDebug="-10";
      RENOGYvoltageDebug="13.0";
    }

    if(charE == 'e') {
      RENOGYcurrentDebug="0";
      RENOGYvoltageDebug="0";
    }

  }
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
      Serial.println("################################################");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
      tryReconnect = true;
    }
    doConnect = false;
  }

  if (millis() > timerTickerForWhatchDog + 300000*2) {
      // force a restart, if there is a problem somewhere, while we dont sent data after 60s
      Serial.println("");
      Serial.println("Timeout exceeded, ready for reset in 15s...");
      getClockTime();
      delay(15000);
      ESP.restart();
  }

  if (tryReconnect) {
    Serial.println("failed during connection... wait 10s and try again.");
    delay(2000);
  }

  if ((millis() > timerTicker2 + 10000) && tryReconnect) {
    tryReconnect = false;
    // force a restart, if there is a problem somewhere, while we dont sent data after 60s
    pClient->disconnect();
    responseData = "";
    callData = "";
    Serial.println("");
    Serial.println("re-connect to host...");
    switchDdeviceAddressesNumber();
    setupDeviceAndConnect();
  }

  if ((millis() > timerTickerForEggTimer + 10000) && timerIsRunning) {
    Serial.println("Eggtimer is running. Send new timer data a bit more often.");
    timerTickerForEggTimer = millis();
    updateEggTimer();
    sendMqttData();
  }

  if (connected) {
    espUpdater();

    if (millis() > timerTicker2 + flexiblePollingSpeed) {
    // if (millis() > timerTicker2 + 10000) {
      // If we are connected to a peer BLE Server, update the characteristic
      byte commands[3][8] = {
        {0x30, 0x03, 0x13, 0xB2, 0x00, 0x06, 0x65, 0x4A}, // Levels
        {0x30, 0x03, 0x13, 0x88, 0x00, 0x11, 0x05, 0x49}, // Cell volts
        {0x30, 0x03, 0x13, 0x99, 0x00, 0x05, 0x55, 0x43}, // Temperatures 
      };
      // String newValue = "Time since boot: " + String(millis()/1000);
      Serial.println("Send new characteristic value:");
      
      actualTimeStamp = getClockTime();
      if (callData == "getLevels") {
        if (checkWiFiConnection()) {
          responseData = "getLevels";
          callData = "";
          Serial.print("Request Level and Voltage Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[0], sizeof(commands[0]));
        }
      }

      if (callData == "getCellVolts") {
        if (checkWiFiConnection()) {
          responseData = "getCellVolts";
          callData = "";
          Serial.print("Request CellVolts Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[1], sizeof(commands[1]));
        }
      }

      if (callData == "getTemperatures") {
        if (checkWiFiConnection()) { 
          responseData = "getTemperatures";
          callData = "";
          Serial.print("Request Temperature Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[2], sizeof(commands[2]));
        }
      }

      if (callData == "connectToAnotherHost") {
        // if the last device was called, return to the first one
        switchDdeviceAddressesNumber();

        pClient->disconnect();
        responseData = "";
        callData = "";
        Serial.println("");
        Serial.println("connect To Another Host...");

        setupDeviceAndConnect();
      }

      timerTicker2 = millis();
      timerTickerForWhatchDog = millis();
    }
  }
  
} // End of loop

void switchDdeviceAddressesNumber() {
  if ( deviceAddressesNumber >= (DEVICEAMOUNT-1)) {
    deviceAddressesNumber=0;
  } else {
    deviceAddressesNumber++;
  }
}

// check for wifi and mqtt connection, true if connectes
// restart device, if connection is gone
boolean checkWiFiConnection() {
  // connected = false;
  if ( checkWiFi()) {
    Serial.println("Wifi connection still exist.");
    // in case mqtt connection is lost, restart device
    if (!espMQTT.isConnected()) {
      delay(10000);
      // after 10s, check if wifi is available
      // if ( checkWiFi()) {
      // try to reconnect to mqtt
      if (!startMQTT()) {
        Serial.println("MQTT Connection lost, restart system");
        ESP.restart(); 
      } else {
        return true;

      }

    } else {
      return true;
    }

  } else {
    Serial.println("Wifi Connection lost, restart system");
    ESP.restart();
    return false;
  }
}

void updateDhtTemperature() {
  DHThumi = dht.readHumidity();
  DHTtemp = dht.readTemperature();
  DHTheatindex = dht.computeHeatIndex(DHTtemp, DHThumi, false);
  Serial.println("");
  Serial.println("updateDhtTemperature");
  Serial.println(DHThumi);
  Serial.println(DHTtemp);
  Serial.println("");
}

void myWhatchdog( void * pvParameters ){
  for(;;){
    // lockVariable();
    if ((millis() > timerTickerDisplay + 60000)) {
    // if ((millis() > timerTickerDisplay + 1000)) {
        
        whatchDogTicks++; // will increment ticks every minute
        timerTickerDisplay = millis();
        Serial.print("whatchDogTicks ");
        Serial.println(whatchDogTicks);

        if (whatchDogTicks > 10) { // restart after 10*60s if no mqtt was sent succesfully
          Serial.println("State undefined: Restart controller now.");
          ESP.restart();
        }
    }
    // unlockVariable();
    vTaskDelay(5);
  }
}

void sendMqttData() {
    updateDhtTemperature();
    whatchDogTicks = 0;
    Serial.println("Send MQTT data...");
    mqttSend("/renogy/sensor/renogy_last_update", actualTimeStamp);
    mqttSend("/renogy/sensor/renogy_current", String(RENOGYcurrent));
    mqttSend("/renogy/sensor/renogy_power", RENOGYpower);
    mqttSend("/renogy/sensor/renogy_voltage", String(RENOGYvoltage));
    mqttSend("/renogy/sensor/renogy_chargelevel", String(RENOGYchargeLevel));
    mqttSend("/renogy/sensor/renogy_capacity", String(RENOGYcapacity));
    mqttSend("/renogy/sensor/renogy_temperature", String(RENOGYtemperature));
    mqttSend("/renogy/sensor/dht_hidxtemperature", String(DHTheatindex));
    mqttSend("/renogy/sensor/dht_temperature", String(DHTtemp));
    mqttSend("/renogy/sensor/dht_humidity", String(DHThumi));
    mqttSend("/renogy/sensor/renogy_timer", String(RENOGYtimer));
    mqttSend("/renogy/sensor/renogy_deviceaddressesnumber", String(deviceAddressesNumber));

    mqttSend("/renogy/sensor/renogy_adress", deviceAddresses[deviceAddressesNumber]);
    mqttSend("/renogy/sensor/renogy_wifi_ssid", wifiSSIDValue);
    Serial.println("Mqtt data was send, return...");
}