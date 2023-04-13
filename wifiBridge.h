#include <WiFi.h>
#include <WiFiMulti.h>
WiFiMulti WiFiMultiElement;


const char* time_zone = "CET-1CEST,M3.5.0,M10.5.0/3";  // TimeZone rule for Europe/Rome including daylight adjustment rules (optional)


#include <WebSocketsClient.h>  // include before MQTTPubSubClient.h
#include <MQTTPubSubClient.h>
#include <time.h>

WebSocketsClient client;
MQTTPubSubClient espMQTT;

int ssid_count = sizeof(ssid) / sizeof(ssid[0]);

String getClockTime()
{
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("No time available (yet)");
    return "No Time set";
  }
  Serial.println(&timeinfo, "%A, %B %d %Y %H:%M:%S");
  // Serial.print("Hour: ");
  // Serial.println(&timeinfo, "%H");
  // Serial.print("Minute: ");
  // Serial.println(&timeinfo, "%M");
  // Serial.print("Second: ");
  // Serial.println(&timeinfo, "%S");

  return String(asctime(&timeinfo));
}

// Function that gets current epoch time
unsigned long getEpochTime() {
  time_t now;
  struct tm timeinfo;
  if (!getLocalTime(&timeinfo)) {
    //Serial.println("Failed to obtain time");
    return(0);
  }
  time(&now);
  return now;
}

// Set time via NTP, as required for x.509 validation
void setClock() {
  configTime(3600, 3600, "pool.ntp.org", "time.nist.gov");  // UTC

  Serial.print("Waiting for NTP time sync: ");
  time_t now = time(nullptr);
  while (now < 8 * 3600 * 2) {
    yield();
    delay(500);
    now = time(nullptr);
  }

  Serial.print("");
  struct tm timeinfo;
  gmtime_r(&now, &timeinfo);
  // Serial.print("NTP time" + String(asctime(&timeinfo)));
  getClockTime();
}

boolean startWiFiMulti() {
  Serial.print("Number of ssid's" + String(ssid_count));
  Serial.println("");
  // add all ssid's to WiFiMulti
  for (int i = 0; i < ssid_count; i++) {
    WiFiMultiElement.addAP(ssid[i], pw[i]);
  }

  // try connecting 4 times, with an timeout between
  for (int i = 0; i < 5; i++) {
    Serial.print("try to connect to wifi...");
    Serial.println(i+1);
    delay(1000*i);

    if ((WiFiMultiElement.run() == WL_CONNECTED)) {
      Serial.print("WiFi connected!!!");
      return true;
    }
  }

  Serial.print("WiFi could not be started");
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
    espMQTT.update();  // should be called
}

void mqttSend(String sensor, String value) {
    // Serial.print("Sending " + sensor);
    // Serial.println();
    espMQTT.publish(sensor, value);
}

//
// Startup
//
boolean startMQTT() {
    Serial.println("connecting to mqtt host...");
    client.begin(mqtt_server[0], mqtt_port[0], "/mqtt", "mqtt");  // "mqtt" is required
    client.setReconnectInterval(2000);

    // initialize mqtt client
    espMQTT.begin(client);
    delay(1000);
    int maxretries = 5;
    while (!espMQTT.isConnected() && maxretries-- > 0) {
      Serial.println(".");
      if (espMQTT.connect(mqtt_clientID[0], mqtt_username[0], mqtt_pw[0])) {
        return true;
      } else {
        delay(2000);
      }
    }
    return false;
}
