/**
 * A BLE bridge from Renogy battery to MQTT
 * based on excellence work from: https://github.com/chadj/renogy-smart-battery
 * 
 * Board: ESP32 Dev Module, Tools > Partition Scheme > Huge App
 * Core: 3.3.7
 * NimBLE 2.5
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

String wifiSSIDValue="noSSID";
String actualTimeStamp="00:00:00";


int flexiblePollingSpeed = 500;
uint8_t watchdogSuccessCounter = 0;

static uint32_t timerTickerDisplay = millis();
uint16_t watchDogTicks = 0;

static boolean doBatteryCall = false;
static uint32_t timerTickerForWatchDog = millis();
static uint32_t timerTicker2 = millis();
static float *temperaturArray;
float externalTempsensor1 = 999.9;
float externalTempsensor2 = 999.9;

BLERemoteService* pRemoteWriteService;
BLERemoteService* pRemoteReadService;
BLERemoteCharacteristic* pRemoteWriteCharacteristic;
BLERemoteCharacteristic* pRemoteNotifyCharacteristic;

#define DEVICEAMOUNT 2
NimBLEClient* pClients[DEVICEAMOUNT]; // Array für mehrere Clients

struct RenogyDevice {
    NimBLEClient* pClient;
    BLERemoteCharacteristic* pWriteChar;
    BLERemoteCharacteristic* pNotifyChar;
    bool connected = false;
};

RenogyDevice myDevices[DEVICEAMOUNT]; // Platz für 2 Devicee

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

static void notifyCallback
  (
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
   
      // flexiblePollingSpeed = 500; // next call for data in 2s
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
      
      // flexiblePollingSpeed = 500; // next call for host switch in 20s
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
      // flexiblePollingSpeed = 500; // next call for host switch in 20s
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

bool startWifiConnectionProcess() {
  while (!startWiFiMulti() && connectionRetryCount < connectionMaxRetries) {
    Serial.printf("Verbindungsversuch %d/%d \n", connectionRetryCount+1, connectionMaxRetries);
    connectionRetryCount++;
  }
  
  if (connectionRetryCount >= 20) {
    Serial.println("Wifi was not connected, wait 30s and than restart ESP...");
    delay(30000);
    ESP.restart();
  } else {
    wifiSSIDValue = WiFi.SSID() + " " + WiFi.localIP().toString();
    Serial.printf("Wifi connection established: %s \n", wifiSSIDValue.c_str());
    setClock();

    if ( startMQTT()) {
      initTempSensor();
      sendRenogyMqttDiscovery();
      delay(1000);
      setupDeviceAndConnect();
      return true;
    }
    return false;
  }

}

class MyClientCallback : public BLEClientCallbacks {
  void onConnect(BLEClient* pclient) {
    Serial.print("Verbunden mit: ");
    Serial.println(pClients[0]->getPeerAddress().toString().c_str());
  }

  void onDisconnect(BLEClient* pclient) {
    doBatteryCall = false;
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
    Serial.print("AVG Battery Seperate Power  (");
    Serial.print(deviceAddresses[i]);
    Serial.print("): ");
    Serial.print(" current:");
    Serial.print(current[i]);
    Serial.print(" power:");
    Serial.println(power[i]);
    powerTemp += power[i];
    RENOGYpowerString += String(power[i]) + "W ";
    RENOGYvoltageString += String(voltageA[i]) + "V ";
    RENOGYtemperatureString += String(temperature[i]) + "°C ";
  }

  Serial.print("AVG Battery Summ of power: ");
  Serial.print(powerTemp);
  Serial.println("");
  RENOGYpower = String(powerTemp);
  
}

void calculateAvgCHARGELEVEL(int deviceIdxLocal) {
  chargelevel[deviceIdxLocal] = RENOGYchargeLevel.toFloat();
  RENOGYCHARGELEVELString = "";
  RENOGYAvgCHARGELEVEL = "0";
  int avgLVLTemp = 0;
  Serial.println("----");
  for (int i = 0; i < DEVICEAMOUNT; i++)
  {
    Serial.print("AVG Battery Seperate Chargelevel (");
    Serial.print(deviceAddresses[i]);
    Serial.print("): ");
    Serial.println(chargelevel[i]);
    avgLVLTemp += chargelevel[i];
    RENOGYCHARGELEVELString += String(chargelevel[i]) + "% ";
  }
  avgLVLTemp = avgLVLTemp/DEVICEAMOUNT;
  Serial.print("AVG Battery CHARGELEVEL: ");
  Serial.print(avgLVLTemp);
  RENOGYAvgCHARGELEVEL = String(avgLVLTemp);
}

bool connectToDevice(int id, NimBLEAddress address) {
  Serial.println("-connectToDevice-----------------");
  Serial.printf("Verbindung zu Device %d (%s)...\n", id, address.toString().c_str());

// --- SCHRITT 0: Aufräumen (Sehr wichtig für Stabilität) ---
  // Falls für diese ID noch ein alter Client existiert, löschen wir ihn sauber.
  if (myDevices[id].pClient != nullptr) {
      Serial.println("Alten Client gefunden, lösche ihn...");
      NimBLEDevice::deleteClient(myDevices[id].pClient);
      myDevices[id].pClient = nullptr;
      myDevices[id].connected = false;
  }

  // --- SCHRITT 1: Client erstellen ---
  myDevices[id].pClient = NimBLEDevice::createClient();
  
  if (!myDevices[id].pClient) {
      Serial.println("Konnte keinen neuen Client erstellen (Speichermangel?)");
      return false;
  }
  
  // --- SCHRITT 2: Verbinden ---
  // Wir nutzen direkt das 'address' Objekt, das wir bekommen haben
  if (!myDevices[id].pClient->connect(address, 15000)) {
      Serial.println("Verbindung zu Device fehlgeschlagen.");
      NimBLEDevice::deleteClient(myDevices[id].pClient);
      myDevices[id].pClient = nullptr;
      return false;
  }

  // 3. Services suchen
  BLERemoteService* pWriteSvc = myDevices[id].pClient->getService(serviceWriteUUID);
  BLERemoteService* pReadSvc  = myDevices[id].pClient->getService(serviceReadUUID);

  if (pWriteSvc == nullptr || pReadSvc == nullptr) {
      Serial.println("Services nicht gefunden.");
      myDevices[id].pClient->disconnect();
      NimBLEDevice::deleteClient(myDevices[id].pClient);
      myDevices[id].pClient = nullptr;
      return false;
  }

  // --- SCHRITT 4: Characteristics holen & speichern ---
  myDevices[id].pWriteChar  = pWriteSvc->getCharacteristic(WRITE_UUID);
  myDevices[id].pNotifyChar = pReadSvc->getCharacteristic(NOTIFY_UUID);

  if (myDevices[id].pWriteChar == nullptr || myDevices[id].pNotifyChar == nullptr) {
      Serial.println("Characteristics nicht gefunden.");
      myDevices[id].pClient->disconnect();
      NimBLEDevice::deleteClient(myDevices[id].pClient);
      myDevices[id].pClient = nullptr;
      return false;
  }

  // --- SCHRITT 5: Notify aktivieren ---
  if (myDevices[id].pNotifyChar->canNotify()) {
      // NimBLE 2.x nutzt 'subscribe' statt 'registerForNotifications'
      if (!myDevices[id].pNotifyChar->subscribe(true, notifyCallback)) {
          Serial.println("Subscribe fehlgeschlagen.");
          myDevices[id].pClient->disconnect();
          NimBLEDevice::deleteClient(myDevices[id].pClient);
          myDevices[id].pClient = nullptr;
          return false;
      }
      Serial.println("Verbindung zu Device und Notifications aktiviert.");
  }

  myDevices[id].connected = true;
  return true;
}

// runs after ESP restart and after a complete read out of all devices
void setupDeviceAndConnect() {
  Serial.println("-setupDeviceAndConnect-----------------");
  doBatteryCall = false;
  readExternalTemperatureSensors();
 
  size_t ramVor = ESP.getFreeHeap();
  Serial.print("Heap setupDeviceAndConnect: "); Serial.println(ESP.getFreeHeap());  
  Serial.println("-Starte Verbindung zu Device-----------");
  for(int i = 0; i < DEVICEAMOUNT; i++) {
    connectToDevice(i, NimBLEAddress(deviceAddresses[i], 0));
  }
  
  // read external Sensors after every device reconnect
  getExternalTemperatureSensors();
  Serial.println("Warte einige Sekunden bis zum Abfragen der Werte...");
  delay(5000);
  callData = "getLevels";
  doBatteryCall = true;
}


void setup() {
  xTaskCreatePinnedToCore(
    myWatchdog,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */    

  // Bluetooth-Controller-Speicher freigeben, falls er "feststeckt"
  // (Nur nötig, wenn ein BT Fehler nach einem Soft-Reset auftritt)
  // esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT);

  if (!NimBLEDevice::isInitialized()) {
    NimBLEDevice::init("ESP32_Renogy_Monitor");
  }

  // ESP_PWR_LVL_P9 entspricht +9dBm (Maximum)
  // NimBLEDevice::setPower(ESP_PWR_LVL_P9);
  
  // für schnellere Datenübertragung
  NimBLEDevice::setMTU(247);

  Serial.begin(115200);
  Serial.println("Starting Arduino BLE Client application...");

  if(startWifiConnectionProcess()){
    return;
  }

  Serial.println("");
  Serial.println("Mqtt was not connected, wait 30s and than restart ESP...");
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

  if (millis() > timerTickerForWatchDog + 300000*2) {
    // force a restart, if there is a problem somewhere, while we dont sent data after 60s
    Serial.println("");
    Serial.println("watchdog: Timeout exceeded, ready for reset in 15s...");
    getClockTime();
    delay(15000);
    ESP.restart();
  }

  if (doBatteryCall) {
    espUpdater();

    if (millis() > timerTicker2 + flexiblePollingSpeed) {
      actualTimeStamp = getClockTime();
      Serial.println("-loop: Devices connected, asking devices for values:-----------------");
      Serial.println(callData);

      if (callData == "getLevels") {
       if (checkWiFiConnection()) {
          for (int i = 0; i < DEVICEAMOUNT; i++) {
            sendCommandToDevice(i, 0,callData);
            delay(1000);
          }
        }
        callData = "getCellVolts";
      }
      else if (callData == "getCellVolts") {
          if (checkWiFiConnection()) {
          for (int i = 0; i < DEVICEAMOUNT; i++) {
            sendCommandToDevice(i, 1,callData);
            delay(1000);
          }
        }
        callData = "getTemperatures";
      }
      else if (callData == "getTemperatures") {
        if (checkWiFiConnection()) { 
          for (int i = 0; i < DEVICEAMOUNT; i++) {
            sendCommandToDevice(i, 2,callData);
            delay(1000);
          }
        }
        callData = "endConnectionAndStartAgain";
      }
      else if (callData == "endConnectionAndStartAgain") {
        for(int i = 0; i < DEVICEAMOUNT; i++) {
        // Prüfen, ob der Client-Zeiger existiert (nicht NULL ist)
          if (myDevices[i].pClient != nullptr) {
          // Nur disconnecten, wenn er auch wirklich verbunden ist
            if (myDevices[i].pClient != nullptr && myDevices[i].pClient->isConnected()) 
            {
              String macAddr = myDevices[i].pClient->getPeerAddress().toString().c_str();
              // 1. Verbindung sauber trennen
              myDevices[i].pClient->disconnect();
              // 2. Den Client komplett aus dem Speicher löschen
              NimBLEDevice::deleteClient(myDevices[i].pClient);
              // 3. Den Zeiger auf NULL setzen, damit keine Geister-Zugriffe passieren
              myDevices[i].pClient = nullptr;
              myDevices[i].connected = false;
              Serial.printf("Device %d [%s] erfolgreich getrennt und Speicher freigegeben.\n", i, macAddr.c_str());
            }
            myDevices[i].connected = false; // Status-Flag zurücksetzen
          }
        }
        checkDataConnection();
        doBatteryCall = false;
        setupDeviceAndConnect();
      }

      timerTicker2 = millis();
      timerTickerForWatchDog = millis();
    }
  }
} // End of loop

void sendCommandToDevice(int deviceIdx, int commandIdx, String responseDataLocal) {
  Serial.println("-sendCommandToDevice-----------------");
  responseData = responseDataLocal;
  // Sicherheitscheck: Ist der Index gültig und das Device verbunden?
  if (deviceIdx >= DEVICEAMOUNT || !myDevices[deviceIdx].connected || myDevices[deviceIdx].pWriteChar == nullptr) {
      Serial.printf("Fehler: Device %d nicht bereit!\n", deviceIdx);
      return;
  }

  Serial.printf("Sende Command %d an Device %d...\n", commandIdx, deviceIdx);
  
  // Hier wird die Charakteristik des spezifischen Devices aus dem Array genutzt
  myDevices[deviceIdx].pWriteChar->writeValue(commands[commandIdx], 8);
}

// check for wifi and mqtt connection, true if connectes
// restart device, if connection is gone
boolean checkWiFiConnection() {
  Serial.println("-checkWiFiConnection-----------------");

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
          externalTempsensor1 = aktuellerWert;
      }
      if(i==1) {
          externalTempsensor2 = aktuellerWert;
      }
    }
    Serial.print("External Temperature Sensor ");
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

void sendRenogyMqttDiscovery() {
  Serial.println("-sendRenogyMqttDiscovery-----------------");
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
  publishSensor("renogy_deviceaddressesnumber", "Renogy Devicee Nummer", "", "", renogyDevice, "renogy");
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
    mqttSend("renogy/sensor/renogy_power", RENOGYpower);
    mqttSend("renogy/sensor/renogy_power_dual", RENOGYpowerString);
    mqttSend("renogy/sensor/renogy_temperature_dual", RENOGYtemperatureString);
    mqttSend("renogy/sensor/renogy_voltage_dual", RENOGYvoltageString);
    mqttSend("renogy/sensor/renogy_average_chargelevel", RENOGYAvgCHARGELEVEL);
    mqttSend("renogy/sensor/renogy_temperature", String(RENOGYtemperature));
    mqttSend("renogy/sensor/renogy_chargelevel_dual", String(RENOGYCHARGELEVELString));
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
    mqttSend("renogy/sensor/external_temperature1", String(externalTempsensor1));
    mqttSend("renogy/sensor/external_temperature2", String(externalTempsensor2));
    mqttSend("renogy/sensor/renogy_wifi_ssid", wifiSSIDValue);    
    Serial.println("Mqtt data was send...");
    Serial.println("---------------------");
}


void myWatchdog( void * pvParameters ){
  for(;;){
    // lockVariable();
    if ((millis() > timerTickerDisplay + 60000)) {
        Serial.println("-myWatchdog-----------------");
        watchDogTicks++; // will increment ticks every minute
        timerTickerDisplay = millis();
        Serial.print("watchdog: watchDogTicks ");
        Serial.print(watchDogTicks);
        //restart after x*60s if webpagecall was not successful with 200 was sent succesfully.
        Serial.println(": will increment ticks every minute"); 
        Serial.println("");

        if (watchDogTicks > 5) { // restart after x*60s if no http request was succesfully
          Serial.println("State undefined: Restart controller now.");
          ESP.restart();
        }
    }
    vTaskDelay(5);
  }
}

void checkDataConnection() {

  Serial.println("-checkDataConnection-----------------");
  size_t ramVor = ESP.getFreeHeap();

  // Scope starten
  {
    WiFiClientSecure sslClient;
    HTTPClient http;

    sslClient.setInsecure();
    if(watchdogSuccessCounter>99){
      watchdogSuccessCounter=0;
    }

    if (http.begin(sslClient, "https://strom.megabyte-programmierung.de/")) {
    // if (http.begin(sslClient, "https://megabyte-programmierung.de/test-1.php")) {
        http.setTimeout(2000);
        int httpCode = http.GET();
        if (httpCode == 200) { // Check for the returning code
          String payload = http.getString();
          watchDogTicks = 0;
          watchdogSuccessCounter++;
          Serial.printf(" watchdog: HTTP request for watchdog was 200. Counter: %d\n", watchdogSuccessCounter);
          Serial.printf(" watchdog: %d, RAM frei: %d\n", httpCode, ESP.getFreeHeap());
        }
        else {
          // Serial.println(httpCode);
          Serial.printf(" watchdog: HTTP request for watchdog has an Error. Counter: %d\n", watchdogSuccessCounter);
          Serial.printf(" watchdog: %d, RAM frei: %d\n", httpCode, ESP.getFreeHeap());
        }

        http.end(); // Schließt die Verbindung, behält aber den Buffer bei
    }
    sslClient.stop(); // Verbindung hart beenden
  // Hier werden die Destruktoren von http und sslClient gerufen
  } 

  size_t ramNach = ESP.getFreeHeap();
  Serial.printf("watchdog: Block-Leck: %d Bytes | RAM frei: %d\n", (int)ramVor - (int)ramNach, ramNach);
}
