/**
 * A BLE bridge from Renogy battery to MQTT
 * based on excellence work from: https://github.com/chadj/renogy-smart-battery
 * 
Board: ESP32 Dev Module
 */
#include "config.h"
#include <NimBLEDevice.h>
#include <HTTPClient.h>
#include "wifiBridge.h"
#include "DS18B20_Temperature.h"

#define RENOGYHEADERSIZE 3 // drop first 3 bytes of response

// The remote service, we wish to connect to.
static BLEUUID serviceWriteUUID("0000ffd0-0000-1000-8000-00805f9b34fb"); // WRITE
static BLEUUID serviceReadUUID("0000fff0-0000-1000-8000-00805f9b34fb"); // READ

static BLEUUID WRITE_UUID("0000ffd1-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

byte commands[3][8] = {
  {0x30, 0x03, 0x13, 0xB2, 0x00, 0x06, 0x65, 0x4A}, // Levels
  {0x30, 0x03, 0x13, 0x88, 0x00, 0x11, 0x05, 0x49}, // Cell volts
  {0x30, 0x03, 0x13, 0x99, 0x00, 0x05, 0x55, 0x43}, // Temperatures 
};

String callData = "getLevels";
String responseData = "";
String RENOGYpower="";
String RENOGYAvgCHARGELEVEL="";
String RENOGYcurrent="";
String RENOGYvoltage="";
String RENOGYcurrentDebug="0";
String RENOGYvoltageDebug="0";
String RENOGYchargeLevel="";
String RENOGYcapacity="";
String RENOGYtemperature="";
String RENOGYcellvolts="";
String RENOGYCHARGELEVELString="";
String RENOGYpowerString="";
String RENOGYtemperatureString="";
String RENOGYvoltageString="";

// String RENOGYtimer="00:00";
String wifiSSIDValue="noSSID";
String actualTimeStamp="00:00:00";

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
static float *temperaturArray;
float tempsensor1 = 999.9;
float tempsensor2 = 999.9;
boolean allBatteriesRead = false;

BLERemoteService* pRemoteWriteService;
BLERemoteService* pRemoteReadService;
BLERemoteCharacteristic* pRemoteWriteCharacteristic;
BLERemoteCharacteristic* pRemoteNotifyCharacteristic;
// BLEAdvertisedDevice* myDevice;
// #define MAX_CLIENTS 2
#define DEVICEAMOUNT 2
NimBLEClient* pClients[DEVICEAMOUNT]; // Array für mehrere Clients
HTTPClient http;

struct RenogyDevice {
    NimBLEClient* pClient;
    BLERemoteCharacteristic* pWriteChar;
    BLERemoteCharacteristic* pNotifyChar;
    bool connected = false;
};

RenogyDevice myDevices[DEVICEAMOUNT]; // Platz für 2 Geräte

// BLEClient* pClient;
// BLEScan* pBLEScan;


// Address of my BT battery devices
static const char* deviceAddresses[DEVICEAMOUNT] = {
  "60:98:66:ed:cb:8b",
  "60:98:66:f9:3a:0f"
};
static float temperature[DEVICEAMOUNT] = {
  0,0
};
static float voltageA[DEVICEAMOUNT] = {
  0,0
};
static float current[DEVICEAMOUNT] = {
  0,0
};
static double voltage = 0.0;
static int16_t power[DEVICEAMOUNT] = {
  0,0
};
static int16_t chargelevel[DEVICEAMOUNT] = {
  -1,-1
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
  bool isNotify
  ) {

    Serial.println("-notifyCallback-----------------");
    // Serial.println("Get notification from BLE device:");
    std::string addr = pBLERemoteCharacteristic->getRemoteService()->getClient()->getPeerAddress().toString();
    
    Serial.printf("Daten von %s: ", addr.c_str());
    Serial.println("");

    deviceAddressesNumber = -1; // Standardmäßig -1 (nicht gefunden)

    // 2. Das Adress-Array durchlaufen und vergleichen
    for (int i = 0; i < DEVICEAMOUNT; i++) {
        if (addr == deviceAddresses[i]) {
            deviceAddressesNumber = i;
            break; // Gefunden, Schleife abbrechen
        }
    }

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


      Serial.println("----------");
      Serial.println("Get Levels ########");
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

      calculatePower(deviceAddressesNumber);
      calculateAvgCHARGELEVEL(deviceAddressesNumber);
   
      flexiblePollingSpeed = 500; // next call for data in 2s
    }

    if(responseData=="getTemperatures") {
      uint8_t numberSensors = ((int16_t)pData[RENOGYHEADERSIZE+0] << 8) | pData[RENOGYHEADERSIZE+1];

      Serial.println("----------");
      Serial.println("Get Temperatures ########");

      int16_t averageTemp = 0;
      for (int i=1; i<=numberSensors; i++){
        int16_t valueSigned = ((int16_t)pData[RENOGYHEADERSIZE+(2*i)] << 8) | pData[RENOGYHEADERSIZE+1+(2*i)];
        averageTemp += valueSigned;
      }

      RENOGYtemperature = String((float)(averageTemp/numberSensors) * 0.1);

      Serial.print("Temperatur: ");
      Serial.println(RENOGYtemperature);
      
      flexiblePollingSpeed = 500; // next call for host switch in 20s
    }

    if(responseData=="getCellVolts") {

      Serial.println("----------");
      Serial.println("Get Cell Volts ########");
      float cellVolts[4];
      // Wir prüfen, ob das Paket lang genug ist (Modbus Header + 4 Zellen)
      if (length > 10 && pData[2] == 0x22) {
        RENOGYcellvolts="";
        for (int i = 0; i < 4; i++) {
          // Wir starten bei Index 5 (Zelle 1)
          // Index 0:ID, 1:Code, 2:Len, 3-4:Zellanzahl(00 04), 5-6:Zelle 1...
          int offset = 5 + (i * 2);
          
          // WICHTIG: Big Endian Zusammensetzung
          // Erstes Byte ist High-Byte (0x0D), zweites ist Low-Byte (0x21)
          uint16_t rawVoltage = (uint16_t)pData[offset] << 8 | pData[offset + 1];
          
          // Umrechnen in Volt
          cellVolts[i] = rawVoltage/10.0;
          
          // Serial.printf("Device %d - Zelle %d: %.2f V\n", deviceAddressesNumber, i + 1, cellVolts[i]);
          RENOGYcellvolts += String(cellVolts[i], 2);
          if (i < 3) {
            RENOGYcellvolts += ", ";
          }
        }
        Serial.print("Cellvolts: ");
        Serial.println(RENOGYcellvolts);
      }
      flexiblePollingSpeed = 500; // next call for host switch in 20s
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
    Serial.print("Verbunden mit: ");
    Serial.println(pClients[0]->getPeerAddress().toString().c_str());
  }

  void onDisconnect(BLEClient* pclient) {
    connected = false;
    Serial.println("onDisconnect");
  }
};

void calculatePower(int deviceIdxLocal ) {
  current[deviceIdxLocal] = RENOGYcurrent.toFloat();
  power[deviceIdxLocal] = RENOGYcurrent.toFloat()*RENOGYvoltage.toFloat();
  voltageA[deviceIdxLocal] = RENOGYvoltage.toFloat();
  temperature[deviceIdxLocal] = RENOGYtemperature.toFloat();

  RENOGYpowerString = "";
  RENOGYvoltageString = "";
  RENOGYtemperatureString = "";
  RENOGYpower = "0";

  int powerTemp = 0;
  Serial.println("----");
  for (int i = 0; i < DEVICEAMOUNT; i++)
  {
    Serial.print("Power Battery Number-");
    Serial.print(i);
    Serial.print(": ");
    Serial.print(" current:");
    Serial.print(current[i]);
    Serial.print(" power:");
    Serial.println(power[i]);
    powerTemp += power[i];
    RENOGYpowerString += String(power[i]) + "W ";
    RENOGYvoltageString += String(voltageA[i]) + "V ";
    RENOGYtemperatureString += String(temperature[i]) + "°C ";
  }

  Serial.print("Summ of power: ");
  Serial.print(powerTemp);
  Serial.println("");
  RENOGYpower = String(powerTemp);
  
}

void calculateAvgCHARGELEVEL(int deviceIdxLocal) {
  chargelevel[deviceIdxLocal] = RENOGYchargeLevel.toFloat();
  RENOGYCHARGELEVELString = "";
  RENOGYAvgCHARGELEVEL = "0";
  int avgLVLTemp = 0;
  allBatteriesRead = true;
  Serial.println("----");
  for (int i = 0; i < DEVICEAMOUNT; i++)
  {
    Serial.print("CHARGELEVEL Battery Number-");
    Serial.print(i);
    Serial.print(": ");
    Serial.println(chargelevel[i]);
    avgLVLTemp += chargelevel[i];
    RENOGYCHARGELEVELString += String(chargelevel[i]) + "% ";
    if(chargelevel[i] == -1 && allBatteriesRead) {
      allBatteriesRead = false;
    }

  }
  avgLVLTemp = avgLVLTemp/DEVICEAMOUNT;
  Serial.print("Avg CHARGELEVEL: ");
  Serial.print(avgLVLTemp);
  Serial.print(" sending unlocked: ");
  Serial.println(allBatteriesRead);
  RENOGYAvgCHARGELEVEL = String(avgLVLTemp);
}

bool connectToDevice(int id, NimBLEAddress address) {
  Serial.println("-connectToDevice-----------------");
  Serial.printf("Verbindung zu Gerät %d (%s)...\n", id, address.toString().c_str());

  // 1. Client erstellen
  myDevices[id].pClient = NimBLEDevice::createClient();
  
  // 2. Verbinden
  if (!myDevices[id].pClient->connect(address)) {
      Serial.println("Verbindung fehlgeschlagen.");
      return false;
  }

  // 3. Services suchen
  BLERemoteService* pWriteSvc = myDevices[id].pClient->getService(serviceWriteUUID);
  BLERemoteService* pReadSvc  = myDevices[id].pClient->getService(serviceReadUUID);

  if (pWriteSvc == nullptr || pReadSvc == nullptr) {
      Serial.println("Services nicht gefunden.");
      myDevices[id].pClient->disconnect();
      return false;
  }

  // 4. Characteristics holen & speichern
  myDevices[id].pWriteChar  = pWriteSvc->getCharacteristic(WRITE_UUID);
  myDevices[id].pNotifyChar = pReadSvc->getCharacteristic(NOTIFY_UUID);

  // 5. Notify aktivieren
  if (myDevices[id].pNotifyChar && myDevices[id].pNotifyChar->canNotify()) {
      // WICHTIG: Im Callback wissen wir sonst nicht, von wem die Daten kommen.
      // NimBLE erlaubt es, dem Callback den Client mitzugeben.
      myDevices[id].pNotifyChar->subscribe(true, notifyCallback);
      Serial.println("Notifications aktiviert.");
  }

  myDevices[id].connected = true;
  return true;
}

// runs after ESP restart and after a complete read out of all devices
void setupDeviceAndConnect() {
  Serial.println("-setupDeviceAndConnect-----------------");
  doConnect = true;
  connected = false;
  doScan = true;
  readTempSensor();

  // BLEDevice::deleteAllBonds();
  BLEDevice::init("client");

  for(int i = 0; i < DEVICEAMOUNT; i++) {
      connectToDevice(i, NimBLEAddress(deviceAddresses[i]));
  }

  // read external Sensors after every device reconnect
  getExternalTemperatureSensors();

  callData = "getLevels";
  connected = true;
  flexiblePollingSpeed = 6000; // next call for data in 2s

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
      initTempSensor();
      // tempsensor1 = 0;
      // tempsensor2 = 0;
      sendRenogyDiscovery();
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

  if (millis() > timerTickerForWhatchDog + 300000*2) {
      // force a restart, if there is a problem somewhere, while we dont sent data after 60s
      Serial.println("");
      Serial.println("watchdog: Timeout exceeded, ready for reset in 15s...");
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
    // pClient->disconnect();
    BLEDevice::deinit();
    responseData = "";
    callData = "";
    Serial.println("");
    Serial.println("re-connect to host...");
    // switchDdeviceAddressesNumber();
    setupDeviceAndConnect();
  }


  if (connected) {
    espUpdater();

    if (millis() > timerTicker2 + flexiblePollingSpeed) {
      actualTimeStamp = getClockTime();
      Serial.println("-loop: Devices connected, asking devices for values:-----------------");
      Serial.println(callData);

      if (callData == "connectToAnotherHost") {
        for(int i = 0; i < DEVICEAMOUNT; i++) {
          // Prüfen, ob der Client-Zeiger existiert (nicht NULL ist)
            if (myDevices[i].pClient != nullptr) {
                // Nur disconnecten, wenn er auch wirklich verbunden ist
                if (myDevices[i].pClient->isConnected()) {
                    myDevices[i].pClient->disconnect();
                    Serial.printf("Gerät %d erfolgreich getrennt.\n", i);
                }
                myDevices[i].connected = false; // Status-Flag zurücksetzen
            }
        }
        connected = false;
        Serial.println("Warte einige Sekuden bis zum neu Verbinden...");
        checkDataConnection();
        delay(3000);
        setupDeviceAndConnect();
      }

      if (callData == "getTemperatures") {
        if (checkWiFiConnection()) { 
          callData = "";
          // Serial.println("Request Temperature Information for both devices:");
          sendCommandToDevice(0, 2,"getTemperatures"); 
          delay(1000);
          sendCommandToDevice(1, 2,"getTemperatures");
        }
        callData="connectToAnotherHost";
      }

      if (callData == "getCellVolts") {
          if (checkWiFiConnection()) {
              callData = "";
              sendCommandToDevice(0, 1,"getCellVolts"); // Gerät 0, Command "CellVolts"
              delay(1000);
              sendCommandToDevice(1, 1,"getCellVolts"); // Gerät 1, Command "CellVolts"
          }
          callData="getTemperatures";
      }

      if (callData == "getLevels") {
          if (checkWiFiConnection()) {
              callData = "";
              sendCommandToDevice(0, 0, "getLevels"); // Gerät 0, Command "Levels"
              delay(1000);
              sendCommandToDevice(1, 0, "getLevels"); // Gerät 1, Command "Levels"
          }
          callData="getCellVolts";
      }

      timerTicker2 = millis();
      timerTickerForWhatchDog = millis();
    }
  }
  
} // End of loop

void sendCommandToDevice(int deviceIdx, int commandIdx, String responseDataLocal) {
  Serial.println("-sendCommandToDevice-----------------");
  responseData = responseDataLocal;
  // Sicherheitscheck: Ist der Index gültig und das Gerät verbunden?
  if (deviceIdx >= 2 || !myDevices[deviceIdx].connected || myDevices[deviceIdx].pWriteChar == nullptr) {
      Serial.printf("Fehler: Gerät %d nicht bereit!\n", deviceIdx);
      return;
  }

  Serial.printf("Sende Command %d an Gerät %d...\n", commandIdx, deviceIdx);
  
  // Hier wird die Charakteristik des spezifischen Geräts aus dem Array genutzt
  myDevices[deviceIdx].pWriteChar->writeValue(commands[commandIdx], 8); 
}

// check for wifi and mqtt connection, true if connectes
// restart device, if connection is gone
boolean checkWiFiConnection() {
  Serial.println("-checkWiFiConnection-----------------");
  // connected = false;
  checkDataConnection();
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

void getExternalTemperatureSensors() {
  Serial.println("-getExternalTemperatureSensors-----------------");

  temperaturArray = getTemperatureValues();
  bool dontSend = false;

  for(byte i=0 ;i < getSensorAmount(); i++) {
    float aktuellerWert = *(temperaturArray + i);
    if (aktuellerWert < -50.0) {
      dontSend = true;
      Serial.println("dontSend = true");
    }
    // Prüfen, ob der Wert gültig ist (nicht -127.8)
    if (aktuellerWert > -50.0) {
      if(i==0) {
          tempsensor1 = aktuellerWert;
      }
      if(i==1) {
          tempsensor2 = aktuellerWert;
      }
    }
    Serial.print("TempSensor ");
    Serial.print(i+1);
    Serial.print(": ");
    Serial.print(*(temperaturArray+i));
    Serial.println("   ");
  }

  if (!dontSend){
    sendMqttDataExternalTemp();
  }
  
  Serial.println("--End external Temperature Sensors--");
}

void myWhatchdog( void * pvParameters ){
  for(;;){
    // lockVariable();
    if ((millis() > timerTickerDisplay + 60000)) {

    // if ((millis() > timerTickerDisplay + 10000)) {
        Serial.println("-myWhatchdog-----------------");
        whatchDogTicks++; // will increment ticks every minute
        timerTickerDisplay = millis();
        Serial.print("watchdog: whatchDogTicks ");
        Serial.print(whatchDogTicks);
        Serial.println(": will increment ticks every minute, restart after 10*60s if webpagecall was not successful with 200 was sent succesfully.");
        Serial.println("");

        if (whatchDogTicks > 10) { // restart after 10*60s if no mqtt was sent succesfully
          Serial.println("State undefined: Restart controller now.");
          ESP.restart();
        }
    }
    // unlockVariable();
    vTaskDelay(5);
  }
}

void sendRenogyDiscovery() {
  Serial.println("-sendRenogyDiscovery-----------------");
  // Gemeinsames Device-Objekt für alle Renogy-Sensoren
  String renogyDevice = ",\"dev\":{\"ids\":[\"renogy_system_esp32\"],\"name\":\"Renogy Battery System\",\"mf\":\"Renogy\",\"mdl\":\"Smart Lithium\"}";

  // 1. Elektrische Werte
  publishSensor("renogy_voltage", "Renogy Spannung", "voltage", "V", renogyDevice, "renogy");
  publishSensor("renogy_cell_voltage", "Renogy Zell Spannung", "", "", renogyDevice, "renogy");
  publishSensor("renogy_current", "Renogy Strom", "current", "A", renogyDevice, "renogy");
  publishSensor("renogy_power", "Renogy Leistung", "power", "W", renogyDevice, "renogy");
  publishSensor("renogy_power_dual", "Renogy Leistung alle", "", "", renogyDevice, "renogy");
  publishSensor("renogy_voltage_dual", "Renogy Spannung alle", "", "", renogyDevice, "renogy");
  
  // 2. Batteriestatus & Kapazität
  publishSensor("renogy_chargelevel", "Renogy Ladestand", "battery", "%", renogyDevice, "renogy");
  publishSensor("renogy_chargelevel_dual", "Renogy Ladestand alle", "", "", renogyDevice, "renogy");
  publishSensor("renogy_average_chargelevel", "Renogy Durchschnitt Ladestand", "battery", "%", renogyDevice, "renogy");
  publishSensor("renogy_capacity", "Renogy Kapazität", "", "Ah", renogyDevice, "renogy");

  // 3. Temperaturen
  publishSensor("renogy_temperature_dual", "Renogy Temperatur Intern alle", "", "", renogyDevice, "renogy");
  publishSensor("renogy_temperature", "Renogy Temperatur Intern", "temperature", "°C", renogyDevice, "renogy");
  publishSensor("external_temperature1", "Extern Temp 1", "temperature", "°C", renogyDevice, "renogy");
  publishSensor("external_temperature2", "Extern Temp 2", "temperature", "°C", renogyDevice, "renogy");

  // 4. System-Infos
  publishSensor("renogy_last_update", "Renogy Letztes Update", "", "", renogyDevice, "renogy");
  publishSensor("renogy_wifi_ssid", "Renogy WiFi SSID", "", "", renogyDevice, "renogy");
  publishSensor("renogy_adress", "Renogy Aktuelle Adresse", "", "", renogyDevice, "renogy");
  publishSensor("renogy_deviceaddressesnumber", "Renogy Geräte Nummer", "", "", renogyDevice, "renogy");
}

// Hilfsfunktion zum Senden der Config
void publishSensor(String id, String name, String dev_cla, String unit, String device, String prefix) {
    // Topic für Home Assistant Discovery
    String configTopic = "homeassistant/sensor/" + id + "/config";
    
    // Das Topic, auf dem der ESP tatsächlich seine Daten sendet
    String stateTopic = prefix + "/sensor/" + id;
    
    String payload = "{";
    payload += "\"name\":\"" + name + "\",";
    payload += "\"stat_t\":\"" + stateTopic + "\",";
    payload += "\"uniq_id\":\"" + id + "_esp32\",";
    
    if (dev_cla != "") payload += "\"dev_cla\":\"" + dev_cla + "\",";
    if (unit != "")    payload += "\"unit_of_meas\":\"" + unit + "\",";
    
    // Wichtig für das Energy Dashboard oder Langzeit-Statistiken
    if (dev_cla == "power" || dev_cla == "battery") {
        payload += "\"stat_cla\":\"measurement\",";
    }

    // Wichtig für das Energy Dashboard oder Langzeit-Statistiken
    if (dev_cla == "energy") {
        payload += "\"stat_cla\":\"total\",";
    }

    payload += "\"val_tpl\":\"{{ value }}\"";
    payload += device;
    payload += "}";

    // Senden mit Retain (3. Parameter true)
    espMQTT.publish(configTopic, payload, true, 1);
}

void sendMqttData() {
    Serial.println("-sendMqttData-----------------");
    mqttSend("renogy/sensor/renogy_last_update", actualTimeStamp);
    mqttSend("renogy/sensor/renogy_current", String(RENOGYcurrent));
    if(allBatteriesRead) {
      mqttSend("renogy/sensor/renogy_power", RENOGYpower);
      mqttSend("renogy/sensor/renogy_power_dual", RENOGYpowerString);
      mqttSend("renogy/sensor/renogy_temperature_dual", RENOGYtemperatureString);
      mqttSend("renogy/sensor/renogy_voltage_dual", RENOGYvoltageString);
      mqttSend("renogy/sensor/renogy_average_chargelevel", RENOGYAvgCHARGELEVEL);
      mqttSend("renogy/sensor/renogy_temperature", String(RENOGYtemperature));
      mqttSend("renogy/sensor/renogy_chargelevel_dual", String(RENOGYCHARGELEVELString));
    }
    mqttSend("renogy/sensor/renogy_voltage", String(RENOGYvoltage));
    mqttSend("renogy/sensor/renogy_cell_voltage", String(RENOGYcellvolts));
    mqttSend("renogy/sensor/renogy_chargelevel", String(RENOGYchargeLevel));
    mqttSend("renogy/sensor/renogy_capacity", String(RENOGYcapacity));

    mqttSend("renogy/sensor/renogy_deviceaddressesnumber", String(deviceAddressesNumber));

    mqttSend("renogy/sensor/renogy_adress", deviceAddresses[deviceAddressesNumber]);
    mqttSend("renogy/sensor/renogy_wifi_ssid", wifiSSIDValue);
    
    Serial.println("Mqtt data was send...");
    Serial.println("---------------------");
}

void sendMqttDataExternalTemp() {
    Serial.println("-sendMqttDataExternalTemp-----------------");
    mqttSend("renogy/sensor/renogy_last_update", actualTimeStamp);
    mqttSend("renogy/sensor/external_temperature1", String(tempsensor1));
    mqttSend("renogy/sensor/external_temperature2", String(tempsensor2));
    mqttSend("renogy/sensor/renogy_wifi_ssid", wifiSSIDValue);    
    Serial.println("Mqtt data was send...");
    Serial.println("---------------------");
}

void checkDataConnection() {

    Serial.println("-checkDataConnection-----------------");
    // http.begin("https://megabyte-programmierung.de/test-1.php"); //Specify the URL
    http.begin("https://strom.megabyte-programmierung.de/"); //Specify the URL
    int httpCode = http.GET(); //Make the request
  
    if (httpCode == 200) { // Check for the returning code
      whatchDogTicks = 0;
      Serial.println("watchdog: HTTP request for watchdog was 200");
    }
    else {
      Serial.println("watchdog: Error on HTTP request for watchdog");
    }
  
    http.end();
}