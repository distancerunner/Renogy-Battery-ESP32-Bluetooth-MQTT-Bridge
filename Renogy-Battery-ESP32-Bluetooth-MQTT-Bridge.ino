/**
 * A BLE bridge from Renogy battery to MQTT
 * based on excellence work from: https://github.com/chadj/renogy-smart-battery
 */

#include "config.h"
#include <NimBLEDevice.h>
#include "wifiBridge.h"

#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

Adafruit_MPU6050 mpu;
#include "DHT.h"
#define DHT11PIN 15

DHT dht(DHT11PIN, DHT11);

uint8_t buttonState = 0;
// Variables will change:
int lastState = LOW;  // the previous state from the input pin
int currentState;     // the current reading from the input pin
struct Button {
	const uint8_t BUTTON_PIN;
	uint32_t numberKeyPresses;
	bool pressed;
};
Button button1 = {19, 2, false};
//variables to keep track of the timing of recent interrupts
unsigned long button_time = 0;  
unsigned long last_button_time = 0;

const unsigned char wifiIcon [] PROGMEM = {
 0x00, 0xff, 0x00, 0x7e, 0x00, 0x18, 0x00, 0x00
   };

const unsigned char mqttIcon [] PROGMEM = {
  0x00, 0x63, 0x77, 0x5D, 0x49, 0x41, 0x41, 0x00
   };

#define SCREEN_WIDTH 128 // OLED display width, in pixels
#define SCREEN_HEIGHT 64 // OLED display height, in pixels
#define OLED_RESET     -1 // Reset pin # (or -1 if sharing Arduino reset pin)
#define SCREEN_ADDRESS 0x3C ///< See datasheet for Address; 0x3D for 128x64, 0x3C for 128x32
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

#define RENOGYHEADERSIZE 3 // drop first 3 bytes of response

// The remote service, we wish to connect to.
static BLEUUID serviceWriteUUID("0000ffd0-0000-1000-8000-00805f9b34fb"); // WRITE
static BLEUUID serviceReadUUID("0000fff0-0000-1000-8000-00805f9b34fb"); // READ

static BLEUUID WRITE_UUID("0000ffd1-0000-1000-8000-00805f9b34fb");
static BLEUUID NOTIFY_UUID("0000fff1-0000-1000-8000-00805f9b34fb");

String callData = "getLevels";
String responseData = "";
String RENOGYpower="0";
String RENOGYcurrent="0";
String RENOGYvoltage="0";
String RENOGYcurrentDebug="0";
String RENOGYvoltageDebug="0";
String RENOGYchargeLevel="0";
String RENOGYcapacity="0";
String RENOGYtemperature="0";
String RENOGYtimer="00:00";
String menuItem="----";
String wifiSSIDValue="noSSID";
String wifiSSID="noSSID";
String wifiIP="noSSID";
String actualTimeStamp="00:00:00";
uint8_t firstRun = 1;
int flexiblePollingSpeed = 20000;
int reconnectTimeOut = 10000;
uint16_t timerCounterStart = 0;
uint16_t timerCounterActual = 0;
boolean timerIsRunning = false;
boolean wifiExist = false;
boolean MQTTexist = false;

static String btActualStatus = "BT unf";
static boolean issueWithBT = false;
static boolean tryReconnect = false;
static boolean doConnect = false;
static boolean connected = false;
static boolean doScan = false;
static uint32_t timerTickerForWhatchDog = millis();
static uint32_t timerTickerForEggTimer = millis();
static uint32_t timerTicker2 = millis();

static uint32_t timerTickerDisplay = millis();
uint32_t duration = 200;

BLERemoteService* pRemoteWriteService;
BLERemoteService* pRemoteReadService;
BLERemoteCharacteristic* pRemoteWriteCharacteristic;
BLERemoteCharacteristic* pRemoteNotifyCharacteristic;
// BLEAdvertisedDevice* myDevice;

BLEClient* pClient;
// BLEScan* pBLEScan;

#define DEVICEAMOUNT 2

TaskHandle_t Task1;

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

void IRAM_ATTR myInterrupt() {
  button_time = millis();
  if (button_time - last_button_time > 250)
  {
    if (button1.numberKeyPresses < 3) {
      button1.numberKeyPresses++;
    } else {
      button1.numberKeyPresses=0;
    }
    duration=0;
    last_button_time = button_time;
  }
	// button1.pressed = true;
}

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

      compareValuesForTimer();
      updateEggTimer();

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
  current[deviceAddressesNumber] = RENOGYcurrent.toFloat();
  power[deviceAddressesNumber] = RENOGYcurrent.toFloat()*RENOGYvoltage.toFloat();

  RENOGYpower = "0";
  int powerTemp = 0;
  for (int i = 0; i < DEVICEAMOUNT; i++)
  {
    Serial.print("current:");
    Serial.println(i);
    Serial.println(current[i]);
    Serial.print("power:");
    Serial.println(i);
    Serial.println(power[i]);
    powerTemp += power[i];
  }
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
  issueWithBT = true;
  delay(700);
  if (!pClient->isConnected()) {
    btActualStatus = "BT err";
    pClient->disconnect();
    return false;
  }

  // Obtain a reference to the service we are after in the remote BLE server.
  pRemoteWriteService = pClient->getService(serviceWriteUUID);
  // if (true) {
  if (pRemoteWriteService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceWriteUUID.toString().c_str());
    btActualStatus = "BT err";
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our pRemoteWriteService");
  pRemoteReadService = pClient->getService(serviceReadUUID);
  if (pRemoteReadService == nullptr) {
    Serial.print("Failed to find our service UUID: ");
    Serial.println(serviceReadUUID.toString().c_str());
    btActualStatus = "BT err";
    pClient->disconnect();
    return false;
  }
  Serial.println(" - Found our pRemoteReadService");
  // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteWriteCharacteristic = pRemoteWriteService->getCharacteristic(WRITE_UUID);
  if (pRemoteWriteCharacteristic == nullptr) {
    Serial.print(F("Failed to find our characteristic UUID: "));
    Serial.println(WRITE_UUID.toString().c_str());
    btActualStatus = "BT err";
    pClient->disconnect();
    return false;
  }
  Serial.println(F(" - Found our Write characteristic"));
      // Obtain a reference to the characteristic in the service of the remote BLE server.
  pRemoteNotifyCharacteristic = pRemoteReadService->getCharacteristic(NOTIFY_UUID);
  if (pRemoteNotifyCharacteristic == nullptr) {
    Serial.print(F("Failed to find our characteristic UUID: "));
    Serial.println(NOTIFY_UUID.toString().c_str());
    btActualStatus = "BT err";
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
  issueWithBT = false;
  btActualStatus = "BT run";
  flexiblePollingSpeed = 6000; // next call for data in 2s
  return true;
}

void setupDeviceAndConnect() {
  doConnect = true;
  connected = false;
  doScan = true;

  btActualStatus = "BT ct...";
  delay(1000);

  BLEDevice::init("client");
}


void setup() {
  Serial.begin(19200);
  Serial.println("Starting Arduino BLE Client application...");

  display.setRotation(2);
  dht.begin();

  if(!display.begin(SSD1306_SWITCHCAPVCC, SCREEN_ADDRESS)) {
    Serial.println(F("SSD1306 allocation failed"));
    for(;;); // Don't proceed, loop forever
  }

  display.display();
  display.clearDisplay();
  display.setTextSize(2);
  display.setTextColor(WHITE);

  // Try to initialize!
  if (!mpu.begin()) {
    Serial.println("Failed to find MPU6050 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("MPU6050 Found!");

  mpu.setAccelerometerRange(MPU6050_RANGE_2_G);
  mpu.setGyroRange(MPU6050_RANGE_250_DEG);
  mpu.setFilterBandwidth(MPU6050_BAND_21_HZ);

  Serial.println("");
  pinMode(button1.BUTTON_PIN, INPUT_PULLUP);
  attachInterrupt(button1.BUTTON_PIN, myInterrupt, RISING);

  display.setTextSize(1);
  display.clearDisplay();
  display.setCursor(0, 0);
  display.println("Scan for WIFI...");
  display.display();

  xTaskCreatePinnedToCore(
    displayMenu,   /* Task function. */
    "Task1",     /* name of task. */
    10000,       /* Stack size of task */
    NULL,        /* parameter of the task */
    1,           /* priority of the task */
    &Task1,      /* Task handle to keep track of created task */
    0);          /* pin task to core 0 */                  
  delay(500);
  btActualStatus = "Wifi ct";
  if (startWiFiMulti()) {
    Serial.println("Wifi connected make next step...");
    display.println("Wifi connected make next step...");
    wifiExist = true;
    Serial.println();

    btActualStatus = "MQTT ct";

    setClock();
    if ( startMQTT()) {
      display.println("MQTT connected make next step...");
      btActualStatus = "MQTT suc";
      MQTTexist = true;
      startupDeviceAfterConnect();
      return;
    } else {
      btActualStatus = "MQTT err";
      startupDeviceAfterConnect();
      return;
    }

  } else { 
    btActualStatus = "WIFI err";
    startupDeviceAfterConnect();
    return;
  }

  Serial.println("");
  Serial.println("Wait 30s and than restart...");
  getClockTime();
  delay(30000);
  ESP.restart();
} // End of setup.

void startupDeviceAfterConnect() {
  if (wifiExist) {
    wifiSSID = WiFi.SSID();
    wifiIP = WiFi.localIP().toString();
    wifiSSIDValue = wifiSSID + " " + wifiIP;
    Serial.println(WiFi.localIP());
  }
  delay(1000);
  setupDeviceAndConnect();
}


// This is the Arduino main loop function.
void loop() {

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
    reconnectTimeOut = 10000; // wait 60s until next connection try

    if (issueWithBT) {
      // in case, the BT server is not reachable
      reconnectTimeOut = 60000; // wait 60s until next connection try
    }
    delay(2000);
  }

  if ((millis() > timerTicker2 + reconnectTimeOut) && tryReconnect) {
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
      btActualStatus = "BT call";
      // String newValue = "Time since boot: " + String(millis()/1000);
      Serial.println("Send new characteristic value:");      

      actualTimeStamp = getClockTime();
      if (callData == "getLevels") {
          checkWiFiConnection(); 
          responseData = "getLevels";
          callData = "";
          Serial.print("Request Level and Voltage Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[0], sizeof(commands[0]));
      }

      if (callData == "getCellVolts") {
          checkWiFiConnection(); 
          responseData = "getCellVolts";
          callData = "";
          Serial.print("Request CellVolts Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[1], sizeof(commands[1]));
      }

      if (callData == "getTemperatures") {
          checkWiFiConnection(); 
          responseData = "getTemperatures";
          callData = "";
          Serial.print("Request Temperature Information: ");
          pRemoteWriteCharacteristic->writeValue(commands[2], sizeof(commands[2]));
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
  
  // displayMenu();
} // End of loop

void displayMenu( void * pvParameters ){
  for(;;){

  if ((millis() > timerTickerDisplay + duration)) {
      /* Get new sensor events with the readings */
      sensors_event_t a, g, temp;
      mpu.getEvent(&a, &g, &temp);

      float DHThumi = dht.readHumidity();
      float DHTtemp = dht.readTemperature();
      float DHTheatindex = dht.computeHeatIndex(DHTtemp, DHThumi, false);
      display.clearDisplay();

      if (wifiExist) {
        display.drawBitmap(60, 55, wifiIcon , 8, 8, WHITE);
      }
      if (MQTTexist) {
        display.drawBitmap(70, 55, mqttIcon , 8, 8, WHITE);
      }
      display.setTextSize(1);
      display.setCursor(0, 55);
      display.print(menuItem);

      display.setCursor(85, 55);
      display.print(btActualStatus);

      display.setCursor(0, 0);
     
      if (button1.numberKeyPresses == 0) {
        menuItem="O---";

        display.println("FRONT");
        display.setCursor(90, 0);
        display.println("BACK");

        const int pixelFY0 = 17-constrain((a.acceleration.z*3),-5,+5);
        const int pixelFY1 = 17+constrain((a.acceleration.z*3),-5,+5);
        display.drawLine(5, pixelFY0, display.width()-5, pixelFY1, SSD1306_WHITE);
        
        display.setCursor(0, 30);
        display.println("LEFT");
        display.setCursor(90, 30);
        display.println("DRIVER");

        const int pixelLY0 = 50-constrain((a.acceleration.x*3),-5,+5);
        const int pixelLY1 = 50+constrain((a.acceleration.x*3),-5,+5);
        display.drawLine(5, pixelLY0, display.width()-5, pixelLY1, SSD1306_WHITE);
      }

      if (button1.numberKeyPresses == 1) {
        menuItem="-O--";

        display.println("Thermometer");
        display.setTextSize(2);
        display.setCursor(0, 10);
        display.print(String(DHTtemp).substring(0,String(DHTtemp).indexOf(".")));
        display.print((char)247);
        display.print("C");
        display.setCursor(70, 10);
        display.print(String(DHThumi).substring(0,String(DHThumi).indexOf(".")));
        display.print("%");

        display.setCursor(0, 30);
        display.print(RENOGYtimer);
        display.print(" min");
      }

      if (button1.numberKeyPresses == 2) {
        menuItem="--O-";

        display.println("Batterie");
        display.setTextSize(2);
        // row 1
        display.setCursor(0, 10);
        display.print(String(RENOGYchargeLevel).substring(0,String(RENOGYchargeLevel).indexOf(".")));
        display.print("%");
        display.setCursor(60, 10);
        display.print(String(RENOGYcurrent).substring(0,String(RENOGYcurrent).indexOf(".")));
        display.print("A");
        // row 2
        display.setCursor(0, 30);
        display.print(RENOGYvoltage);
        display.println("V");
        display.setCursor(60, 30);
        display.print(String(RENOGYpower).substring(0,String(RENOGYpower).indexOf(".")));
        display.print("W");
      }

      if (button1.numberKeyPresses == 3) {
        menuItem="---O";

        display.println("Info");
        // row 1
        display.setCursor(0, 10);
        display.println(actualTimeStamp);
        display.println(wifiSSID);
        display.println(wifiIP);
        display.print(String(deviceAddressesNumber));
        display.print("/");
        display.println(deviceAddresses[deviceAddressesNumber]);
      }

      display.display(); 
      duration = 200;
      timerTickerDisplay = millis();
  }

  vTaskDelay(5);
  }
}


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
  wifiExist = false;
  MQTTexist = false;
  // connected = false;
  if ( checkWiFi()) {
    Serial.println("Wifi connection still exist.");
    wifiExist = true;
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
        MQTTexist = true;
      }

    }

  } else {
    // Serial.println("Wifi Connection lost, restart system");
    // ESP.restart();
    return false;
  }
}

void sendMqttData() {
  if (MQTTexist) {
    Serial.println("Send MQTT data...");
    mqttSend("/renogy/sensor/renogy_last_update", actualTimeStamp);
    mqttSend("/renogy/sensor/renogy_current", String(RENOGYcurrent));
    mqttSend("/renogy/sensor/renogy_power", RENOGYpower);
    mqttSend("/renogy/sensor/renogy_voltage", String(RENOGYvoltage));
    mqttSend("/renogy/sensor/renogy_chargelevel", String(RENOGYchargeLevel));
    mqttSend("/renogy/sensor/renogy_capacity", String(RENOGYcapacity));
    mqttSend("/renogy/sensor/renogy_temperature", String(RENOGYtemperature));
    mqttSend("/renogy/sensor/renogy_timer", String(RENOGYtimer));
    mqttSend("/renogy/sensor/renogy_deviceaddressesnumber", String(deviceAddressesNumber));

    mqttSend("/renogy/sensor/renogy_adress", deviceAddresses[deviceAddressesNumber]);
    mqttSend("/renogy/sensor/renogy_wifi_ssid", wifiSSIDValue);
    Serial.println("Mqtt data was send, return...");
  }
}