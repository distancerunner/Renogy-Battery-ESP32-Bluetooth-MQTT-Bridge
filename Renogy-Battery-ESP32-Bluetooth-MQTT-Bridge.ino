/**
 * A BLE bridge from Renogy battery to MQTT
 * based on excellence work from: https://github.com/chadj/renogy-smart-battery
 */

#include "BLEDevice.h"
#include "config.h"
#include "wifiBridge.h"

#define RENOGYHEADERSIZE 3 // drop first 3 bytes of response

#define bleServerAddress "60:98:66:f9:3a:0f" // Address of my BT battery device

// The remote service, we wish to connect to.
static BLEUUID serviceWriteUUID("0000ffd0-0000-1000-8000-00805f9b34fb"); // WRITE
static BLEUUID serviceReadUUID("0000fff0-0000-1000-8000-00805f9b34fb"); // READ

static BLEUUID WRITE_UUID("0000ffd1-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

static String callData = "getLevels";
String responseData = "";
String RENOGYcurrent="";
String RENOGYvoltage="";
String RENOGYchargeLevel="";
String RENOGYcapacity="";
String RENOGYtemperature="";
uint8_t firstRun = 1;

static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;

static BLERemoteCharacteristic* pRemoteWriteCharacteristic;
static BLERemoteCharacteristic* pRemoteNotifyCharacteristic;
static BLEAdvertisedDevice* myDevice;

#define DEVICEAMOUNT 2
static const char* deviceAddresses[DEVICEAMOUNT] = {
  "60:98:66:ed:cb:8b",
  "60:98:66:f9:3a:0f"
};
uint8_t deviceAddressesNumber=0;

int mqtt_server_count = sizeof(mqtt_server) / sizeof(mqtt_server[0]);
//Address of the peripheral device. Address will be found during scanning...
// static BLE pServerAddress;

static void notifyCallback(
  BLERemoteCharacteristic* pBLERemoteCharacteristic,
  uint8_t* pData,
  size_t length,
  bool isNotify) {
    // Serial.print("Notify callback for characteristic ");
    // Serial.print(pBLERemoteCharacteristic->getUUID().toString().c_str());
    // Serial.print(" of data length ");
    // Serial.println(length);
    Serial.println("#####################");
    Serial.println("Get notification from BLE device:");
    // int byteCounter=0;
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


      Serial.println("Get Levels ########");
      Serial.println("Current:");
      Serial.println(RENOGYcurrent);
      Serial.println("Voltage:");
      Serial.println(RENOGYvoltage);
      Serial.println("RENOGYchargeLevel:");
      Serial.println(RENOGYchargeLevel);
      Serial.println("RENOGYcapacity:");
      Serial.println(RENOGYcapacity);

      callData="getTemperatures";
    }

    if(responseData=="getTemperatures") {
      uint8_t numberSensors = ((int16_t)pData[RENOGYHEADERSIZE+0] << 8) | pData[RENOGYHEADERSIZE+1];

      Serial.println("Get Temperatures ########");
      int16_t averageTemp = 0;
      for (int i=1; i<=numberSensors; i++){
        int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE+(2*i)] << 8) | pData[RENOGYHEADERSIZE+1+(2*i)];
        averageTemp += valueSigned;
      }

      RENOGYtemperature = String((float)(averageTemp/numberSensors) * 0.1);
      Serial.println("Temperature");
      Serial.println(RENOGYtemperature);

      // callData="getCellVolts"; // exclude cellVolts for now
      // callData="connectToAnotherHost";
      callData="getLevels";
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
    Serial.println("Hex data received:"); 
    for (int i=1; i<=length; i++){
      Serial.printf("%02x", pData[i-1]);
      if(i % 2 == 0){
        Serial.print(" "); 
      }
    }
    Serial.println(" "); 
    Serial.println("END notifyCallback ########");
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

// bool disconnectToServer() {
//   BLEDevice::
// }

bool connectToServer() {
    Serial.print("Forming a connection to ");
    Serial.println(myDevice->getAddress().toString().c_str());
    
    BLEClient* pClient = BLEDevice::createClient();
    Serial.println(" - Created client");

    pClient->setClientCallbacks(new MyClientCallback());

    // Connect to the remove BLE Server.
    pClient->connect(myDevice);  // if you pass BLEAdvertisedDevice instead of address, it will be recognized type of peer device address (public or private)
    Serial.println(" - Connected to server");
    // pClient->setMTU(517); //set client to request maximum MTU from server (default is 23 otherwise)
  
    // Obtain a reference to the service we are after in the remote BLE server.
    BLERemoteService* pRemoteWriteService = pClient->getService(serviceWriteUUID);
    if (pRemoteWriteService == nullptr) {
      Serial.print("Failed to find our service UUID: ");
      Serial.println(serviceWriteUUID.toString().c_str());
      pClient->disconnect();
      return false;
    }
    Serial.println(" - Found our pRemoteWriteService");

    BLERemoteService* pRemoteReadService = pClient->getService(serviceReadUUID);
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
    Serial.println(F(" - Found our Notifyite characteristic"));    

    if(pRemoteNotifyCharacteristic->canNotify()) {
      pRemoteNotifyCharacteristic->registerForNotify(notifyCallback);
    }

    connected = true;
    return true;
}
/**
 * Scan for BLE servers and find the first one that advertises the service we are looking for.
 */
class MyAdvertisedDeviceCallbacks: public BLEAdvertisedDeviceCallbacks {
 /**
   * Called for each advertising BLE server.
   */
  void onResult(BLEAdvertisedDevice advertisedDevice) {
    Serial.println("BLE Advertised Device found: ");
    // Serial.printf("%s", advertisedDevice.getName().c_str());
    Serial.println(advertisedDevice.toString().c_str());

    // We have found a device, let us now see if it contains the service we are looking for.
    if (strcmp(deviceAddresses[deviceAddressesNumber], advertisedDevice.getAddress().toString().c_str()) == 0) {
      Serial.println("BLE Advertised Device found with serviceWriteUUID");
      Serial.println("");
      BLEDevice::getScan()->stop();
      myDevice = new BLEAdvertisedDevice(advertisedDevice);
      doConnect = true;
      doScan = true;

    } // Found our server
  } // onResult
}; // MyAdvertisedDeviceCallbacks

void setupDeviceAnConnect() {
  doConnect = false;
  connected = false;
  doScan = false;

  // Retrieve a Scanner and set the callback we want to use to be informed when we
  // have detected a new device.  Specify that we want active scanning and start the
  // scan to run for 5 seconds.
  BLEScan* pBLEScan = BLEDevice::getScan();
  pBLEScan->setAdvertisedDeviceCallbacks(new MyAdvertisedDeviceCallbacks());
  pBLEScan->setInterval(1349);
  pBLEScan->setWindow(449);
  pBLEScan->setActiveScan(true);
  pBLEScan->start(5, false);
}


void setup() {
  Serial.begin(19200);
  Serial.println("Starting Arduino BLE Client application...");

  if (startWiFiMulti()) {
    Serial.println("Wifi connected make next step...");
    Serial.println();  

    setClock();
    if ( startMQTT()) {
      BLEDevice::init("");
      setupDeviceAnConnect();
      return;
    }

  }

  Serial.println("Wait 30s and than restart...");
  delay(30000);
  ESP.restart();
} // End of setup.


// This is the Arduino main loop function.
void loop() {
  static uint32_t timerTicker2 = millis();
  // If the flag "doConnect" is true then we have scanned for and found the desired
  // BLE Server with which we wish to connect.  Now we connect to it.  Once we are 
  // connected we set the connected flag to be true.
  if (doConnect == true) {
    if (connectToServer()) {
      Serial.println("We are now connected to the BLE Server.");
      Serial.println("################################################");
    } else {
      Serial.println("We have failed to connect to the server; there is nothin more we will do.");
    }
    doConnect = false;
  }

  if (connected) {
    espUpdater();

    if (millis() > timerTicker2 + 10000) {
      // If we are connected to a peer BLE Server, update the characteristic
      byte commands[3][8] = {
        {0x30, 0x03, 0x13, 0xB2, 0x00, 0x06, 0x65, 0x4A}, // Levels
        {0x30, 0x03, 0x13, 0x88, 0x00, 0x11, 0x05, 0x49}, // Cell volts
        {0x30, 0x03, 0x13, 0x99, 0x00, 0x05, 0x55, 0x43}, // Temperatures 
      };
      // String newValue = "Time since boot: " + String(millis()/1000);
      Serial.println("Send new characteristic value:");      

      if (callData == "getLevels") {
        responseData = "getLevels";
        callData = "";
        Serial.print("Request Level and Voltage Information: ");
        pRemoteWriteCharacteristic->writeValue(commands[0], sizeof(commands[0]));
      }

      if (callData == "getCellVolts") {
        responseData = "getCellVolts";
        callData = "";
        Serial.print("Request CellVolts Information: ");
        pRemoteWriteCharacteristic->writeValue(commands[1], sizeof(commands[1]));
      }

      if (callData == "getTemperatures") {
        responseData = "getTemperatures";
        callData = "";
        Serial.print("Request Temperature Information: ");
        pRemoteWriteCharacteristic->writeValue(commands[2], sizeof(commands[2]));
      }

      if (callData == "connectToAnotherHost") {
        BLEDevice::deinit(false);
        responseData = "";
        callData = "";
        Serial.print("rennecto to another host: ");
        setupDeviceAnConnect();
        // pRemoteWriteCharacteristic->writeValue(commands[2], sizeof(commands[2]));
      }

      timerTicker2 = millis();
    }
  }else if(doScan){
    BLEDevice::getScan()->start(0);  // this is just example to start scan after disconnect, most likely there is better way to do it in arduino
  }
  
  // delay(10000); // Delay a second between loops.
  // ESP.restart();
} // End of loop

void sendMqttData() {
  Serial.println("Send MQTT data...");

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
      }
    }
  } else {
    Serial.println("MQTT Connection lost, restart system");
    ESP.restart();       
  }
    // connected = true;
    Serial.println();

    // mqttSend("/victron/sensor/watt", String(counter));
    mqttSend("/renogy/sensor/renogy_current", String(RENOGYcurrent));
    mqttSend("/renogy/sensor/renogy_voltage", String(RENOGYvoltage));
    mqttSend("/renogy/sensor/renogy_chargelevel", String(RENOGYchargeLevel));
    mqttSend("/renogy/sensor/renogy_capacity", String(RENOGYcapacity));
    mqttSend("/renogy/sensor/renogy_temperature", String(RENOGYtemperature));

    // mqttSend("/victron/sensor/ve_state", VEStatus[VE_state].value);
    // mqttSend("/victron/sensor/ve_error", VEError[VE_error].value + (String((int)tickslower) + "/" + String((int)tickfaster)));
    // mqttSend("/victron/sensor/ve_error", VEError[VE_error].value);
    mqttSend("/renogy/sensor/renogy_last_update", getClockTime());
    mqttSend("/renogy/sensor/renogy_wifi_ssid", WiFi.SSID());
}