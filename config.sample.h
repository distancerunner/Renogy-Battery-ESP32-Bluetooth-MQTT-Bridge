 
// #define ESP32

/*
   WiFi parameters
*/

// WiFi SSID'S and passwords
// the strongest WiFi station will be used
const char* ssid[] = {"LALALALALA", "LALALALALA", "LALALALALA"};
const char* pw[] = {"LALALALALA", "LALALALALA", "LALALALALA"};

/*
   MQTT parameters
   you can have more than one MQTT server, the first one that answers will have the connection
   it is strongly recommended to use SSL if you send a username and password over the internet
   ATTENTION: use a unique client id to connect to MQTT or you will be kicked out by another device
   using your id
*/
#define MQTT_MAX_RETRIES 3   // maximum retires to reach a MQTT broker
const char* mqtt_server[] = {"mqtt.LALALALALA.de"};
// no SSL ports
const uint16_t mqtt_port[] = {9001};
// SSL ports
const char* mqtt_clientID[] = {"LALALALALA-mppt-client"};
const char* mqtt_username[] = {"LALALALALA"};
const char* mqtt_pw[] = {"LALALALALA"};
