#include <FS.h>
#include <ESP8266WiFi.h>
#include <PubSubClient.h>
#include <ESP8266mDNS.h>
#include <WiFiUdp.h>
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include <ArduinoOTA.h>
#include <EEPROM.h>
#include <WiFiManager.h>
#include <ArduinoJson.h>
#include "Arduino.h"

extern "C"
{
  #include "user_interface.h"
}

/*
 * Variables
 */
os_timer_t Timer1;         
int Counter = 0;
           
bool TickOccured = true;

bool doorbellTriggered = false;

char MQTT_BROKER[40] = "x.x.x.x";
char MQTT_PORT[6] = "1883";

const char*    statusTopic = "/home/haustuer/klingel/status";
const char*    doorOpenerTopic = "/home/haustuer/klingel/dooropener";
const char*    modeTopic = "/home/haustuer/klingel/mode";
const char*    doorbellTopic ="/home/haustuer/klingel";

const int doorbellInput = D1;
const int modePin = D3;
const int dooropenerPin = D4;

bool shouldSaveConfig = false;

char msgToPublish[60];

/*
 * Handler
 */
WiFiManager wifiManager;
WiFiClient espClient;
PubSubClient client(espClient);

/*
 * FUNCTIONS
 */

/*
 * Callback for WifiManager Settings
 */
void saveConfigCallback () {
  Serial.println("Should save config");
  shouldSaveConfig = true;
}

/*
 * Timer Callback for timed Interrupt
 */
void timerCallback(void *pArg)
{
  TickOccured = true;
  *((int *) pArg) += 1;
} 

/*
 * Callback function for MQTT Subscribe
 */
void callback(char* topic, byte* payload, unsigned int length) {
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();

  if (strcmp(topic,modeTopic) == 0 && (char)payload[0] == '1')
  {
    digitalWrite(modePin,HIGH);
    Serial.println("switch to HA");  
    EEPROM.write(0, 1);
    TickOccured = true;
    
  }
  else if (strcmp(topic,modeTopic) == 0 && (char)payload[0] == '0')
  {
    digitalWrite(modePin,LOW);
    Serial.println("Switch to doorbell");
    EEPROM.write(0, 0);
    TickOccured = true;
  
  }
  else if (strcmp(topic,doorOpenerTopic) == 0 && (char)payload[0] == '1')
  {
    Serial.println("door opener triggered");
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(dooropenerPin, HIGH);
    delay(4000);
    Serial.println("door opener end");
    digitalWrite(BUILTIN_LED, HIGH);  
    digitalWrite(dooropenerPin, LOW);
  }
  EEPROM.commit();
}

/*
 * Interrupt function for detecting falling edge on Input pin
 */
void ICACHE_RAM_ATTR doorbellFallingEdge()
{
  doorbellTriggered = true;
}

/*
 * SETUP ROUTINE
 */
void setup() {
  pinMode(BUILTIN_LED, OUTPUT);  
  pinMode(modePin, OUTPUT);
  pinMode(dooropenerPin, OUTPUT);
  pinMode(doorbellInput, INPUT_PULLUP);

  digitalWrite(BUILTIN_LED, HIGH); 
      
  EEPROM.begin(512);
    
  Serial.begin(115200);

  Serial.println("mounting FS...");

  if (SPIFFS.begin()) {
    Serial.println("mounted file system");
    if (SPIFFS.exists("/config.json"))
    {
      Serial.println("reading config file");
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        Serial.println("opened config file");
        size_t size = configFile.size();
        std::unique_ptr<char[]> buf(new char[size]);
        configFile.readBytes(buf.get(), size);
        
      #ifdef ARDUINOJSON_VERSION_MAJOR >= 6
        DynamicJsonDocument json(1024);
        auto deserializeError = deserializeJson(json, buf.get());
        serializeJson(json, Serial);
        if ( ! deserializeError )
        {
      #else
          DynamicJsonBuffer jsonBuffer;
          JsonObject& json = jsonBuffer.parseObject(buf.get());
          json.printTo(Serial);
          if (json.success())
          {
      #endif
            Serial.println("\nparsed json");
            strcpy(MQTT_BROKER, json["mqtt_server"]);
            strcpy(MQTT_PORT, json["mqtt_port"]);
          } else
          {
            Serial.println("failed to load json config");
        }
        configFile.close();
      }
    }
  } else {
    Serial.println("failed to mount FS");
  }

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", MQTT_BROKER, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", MQTT_PORT, 6);

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);
  
  wifiManager.autoConnect("Haustuer");

  strcpy(MQTT_BROKER, custom_mqtt_server.getValue());
  strcpy(MQTT_PORT, custom_mqtt_port.getValue());

  //save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.println("saving config");
  #ifdef ARDUINOJSON_VERSION_MAJOR >= 6
    DynamicJsonDocument json(1024);
  #else
    DynamicJsonBuffer jsonBuffer;
    JsonObject& json = jsonBuffer.createObject();
  #endif
    json["mqtt_server"] = MQTT_BROKER;
    json["mqtt_port"] = MQTT_PORT;

    File configFile = SPIFFS.open("/config.json", "w");
    if (!configFile)
    {
      Serial.println("failed to open config file for writing");
    }

  #ifdef ARDUINOJSON_VERSION_MAJOR >= 6
    serializeJson(json, Serial);
    serializeJson(json, configFile);
  #else
    json.printTo(Serial);
    json.printTo(configFile);
  #endif
    configFile.close();
  }

  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  
  ArduinoOTA.setHostname("Haustuer");
  ArduinoOTA.begin();

  Serial.println("Connecting MQTT");
  Serial.print((char *)MQTT_BROKER);
  Serial.println(atoi(MQTT_PORT));
  client.setServer((char *)MQTT_BROKER, atoi(MQTT_PORT));
  client.setCallback(callback);
  client.connect("HaustuerV2");
  client.subscribe(modeTopic);
  client.subscribe(doorOpenerTopic);
  
  os_timer_setfn(&Timer1, timerCallback, &Counter);
  os_timer_arm(&Timer1, 30000, true);

  digitalWrite(modePin,EEPROM.read(0));

  attachInterrupt(digitalPinToInterrupt(doorbellInput), doorbellFallingEdge, FALLING);
}

/*
 * Loop
 */
void loop()
{
  if (!client.connected())
  {
    Serial.println("disconnected, retrying");
    client.connect("HaustuerV2");
    client.subscribe(modeTopic);
    client.subscribe(doorOpenerTopic);
  }

  if (TickOccured)
  {
    Serial.println("Heartbeat Time!");
    if (client.connected())
    {
      snprintf (msgToPublish, 60, "%d", EEPROM.read(0));
      client.publish(statusTopic, msgToPublish);
    }
    TickOccured = false;
  }

  if (doorbellTriggered)
  {
    Serial.println("hat geklingelt");
    client.publish(doorbellTopic, "DingDong");
    delay(2000);
    client.publish(doorbellTopic, " ");    
    doorbellTriggered = false;
  }

  client.loop();
  ArduinoOTA.handle();
}

/*
 * End.
 */
