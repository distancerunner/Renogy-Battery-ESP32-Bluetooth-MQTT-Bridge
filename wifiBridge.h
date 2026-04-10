#include <WiFi.h>
#include <WiFiMulti.h>
#include "ESPTelnet.h"
#include <PubSubClient.h>

ESPTelnet telnet;
bool isAuthenticated = false;
const String TELNET_PW = telnetpassword;

// Das Makro definiert ein neues "Log"-Kommando, das beide Kanäle bedient
#define L_PRINT(...)      { Serial.print(__VA_ARGS__);   if(isAuthenticated) telnet.print(__VA_ARGS__);   }
#define L_PRINTLN(...)    { Serial.println(__VA_ARGS__); if(isAuthenticated) telnet.println(__VA_ARGS__); }
#define L_PRINTF(f, ...)  { Serial.printf(f, __VA_ARGS__); if(isAuthenticated) telnet.printf(f, __VA_ARGS__); }

WiFiMulti WiFiMultiElement;

const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)

#include <time.h>

WiFiClient espClient;
PubSubClient espMQTT(espClient);

int connectionRetryCount = 0;
int connectionMaxRetries = 20;

int ssid_count = sizeof(ssid) / sizeof(ssid[0]);

String getClockTime()
{
  L_PRINTLN("-getClockTime-----------------");
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    L_PRINTLN("No time available (yet)");
    return "No Time set";
  }

  char timeString[64]; // Puffer für den formatierten Text
  strftime(timeString, sizeof(timeString), "%A, %B %d %Y %H:%M:%S", &timeinfo);
  L_PRINTLN(String(timeString));
  return String(asctime(&timeinfo));
}

// Function that gets current epoch time
unsigned long getEpochTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //L_PRINTLN("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");  // UTC

  L_PRINT("setClock: Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    now = time(nullptr);
  }

  L_PRINT("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  getClockTime();
}

boolean startWiFiMulti() {
  L_PRINTLN("-startWiFiMulti-----------------");
  L_PRINTLN("Number of ssid: " + String(ssid_count));
  // 1. Hostnamen festlegen (bevor die Verbindung aufgebaut wird)
  WiFi.setHostname(host_name);
  // add all ssid's to WiFiMulti
  for (int i = 0; i < ssid_count; i++) {
    WiFiMultiElement.addAP(ssid[i], pw[i]);
    Serial.printf("Select Wifi SSID %s \n", ssid[i]);
  }

  // try connecting 4 times, with an timeout between
  for (int i = 0; i < 5; i++) {
    L_PRINTF("connect to wifi, try number: %d \n", i+1);
    delay(1000*i);

    if ((WiFiMultiElement.run() == WL_CONNECTED)) {
      L_PRINTLN("WiFi connected!!!");
      return true;
    }
  }

  L_PRINTLN("WiFi could not be started");
  return false;
}

// check if WiFi is still connected 
// if not, try to reconnect
boolean checkWiFi(){

  if ( WiFi.status() != WL_CONNECTED ){
    return startWiFiMulti();
  }
  return true;
}

void espUpdater() {
    espMQTT.loop();  // should be called
}

bool isConnected() {
  return espMQTT.connected();
}

void mqttSend(String sensor, String value) {
    espMQTT.publish(sensor.c_str(), value.c_str(),true);
}

boolean startMQTT() {
    L_PRINTLN("-startMQTT-----------------");
    L_PRINTLN("connecting to mqtt host...");
    while (!espMQTT.connected() && connectionRetryCount < connectionMaxRetries) {
        L_PRINTF("connect to mqtt, try number: %d/%d \n", connectionRetryCount + 1, connectionMaxRetries);
        
        if (espMQTT.connect(mqtt_clientID[0], mqtt_username[0], mqtt_pw[0])) {
            L_PRINTLN("mqtt is connected!");
            return true;
        } else {
            L_PRINTF("failed, rc=%d - retry in 5s\n", espMQTT.state());
            delay(5000);
        }
        connectionRetryCount++;
    }
    return false;
}

// Callback: Wird aufgerufen, wenn jemand per Telnet etwas tippt
void onTelnetInput(String str) {
  if (!isAuthenticated) {
    if (str == TELNET_PW) {
      isAuthenticated = true;
      telnet.println("✅ Passwort korrekt! Logs gestartet...");
    } else {
      telnet.print("❌ Falsches Passwort! Versuche es erneut: ");
    }
  }
}

// Callback: Begrüßung bei Verbindung
void onTelnetConnect(String ip) {
  Serial.print("Telnet-Verbindung von: ");
  Serial.println(ip);
  telnet.println("Willkommen beim ESP32-Log-Server.");
  if (!isAuthenticated) telnet.print("Bitte Passwort eingeben: ");
}

void onTelnetDisconnect(String ip) {
  isAuthenticated = false; // Bei Trennung wieder sperren
}