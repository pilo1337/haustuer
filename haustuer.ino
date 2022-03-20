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

/*
 * Un-Comment for Serial Debug Output
 */
//#define SerialDebug

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

bool shouldSaveConfig = false;
char msgToPublish[60];

bool bufState;
long lastDebounceTime = 0;
long debounceDelay = 150;

/*
 * Placeholders for MQTT Variables, these are entered in WiFiManager Portal, no need to change it here.
 */
char MQTT_BROKER[40] = "x.x.x.x";
char MQTT_PORT[6] = "1883";

/*
 * These are the topics for the doorbell
 */
const char*    statusTopic = "/home/haustuer/klingel/status";
const char*    doorOpenerTopic = "/home/haustuer/klingel/dooropener";
const char*    modeTopic = "/home/haustuer/klingel/mode";
const char*    doorbellTopic ="/home/haustuer/klingel";

/*
 * Input declarations
 */
const int doorbellInput = D1;
const int modePin = D3;
const int dooropenerPin = D4;

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
  #ifdef SerialDebug
  Serial.println("Should save config");
  #endif
  shouldSaveConfig = true;
}

/*
 * Timer Callback for timed Interrupt every 30 seconds (Heartbeat)
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
  #ifdef SerialDebug
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");
  for (int i=0;i<length;i++)
  {
    Serial.print((char)payload[i]);
  }
  Serial.println();
  #endif
  if (strcmp(topic,modeTopic) == 0 && (char)payload[0] == '1')
  {
    digitalWrite(modePin,LOW);
    #ifdef SerialDebug
    Serial.println("switch to HA");  
    #endif
    EEPROM.write(0, 1);
    TickOccured = true;
    
  }
  else if (strcmp(topic,modeTopic) == 0 && (char)payload[0] == '0')
  {
    digitalWrite(modePin,HIGH);
    #ifdef SerialDebug
    Serial.println("Switch to doorbell");
    #endif
    EEPROM.write(0, 0);
    TickOccured = true;
  
  }
  else if (strcmp(topic,doorOpenerTopic) == 0 && (char)payload[0] == '1')
  {
    #ifdef SerialDebug
    Serial.println("door opener triggered");
    #endif
    digitalWrite(BUILTIN_LED, LOW);
    digitalWrite(dooropenerPin, LOW);
    delay(4000);
    #ifdef SerialDebug
    Serial.println("door opener end");
    #endif
    digitalWrite(BUILTIN_LED, HIGH);  
    digitalWrite(dooropenerPin, HIGH);
  }
  EEPROM.commit();
}

/*
 * Interrupt function for debounce on Input pin
 */
void ICACHE_RAM_ATTR doorbellDebounce()
{
  #ifdef SerialDebug
  Serial.println("Interrupt: entered");
  #endif
  
  int reading = digitalRead(doorbellInput);

  if(reading == bufState) return;
  boolean debounce = false;

  if((millis() - lastDebounceTime) <= debounceDelay)
  {
    debounce = true;
  }
  lastDebounceTime = millis();
  if(debounce) return;  

  bufState = reading;

  if (reading==1) doorbellTriggered = true;
  #ifdef SerialDebug
  Serial.println("Interrupt: button: " + String(reading));
  #endif
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

  #ifdef SerialDebug
  Serial.println("DEBUG MODE");
  Serial.println("mounting FS...");
  wifiManager.setDebugOutput(true);
  #else
  wifiManager.setDebugOutput(false);
  #endif
  
  if (SPIFFS.begin()) {
    #ifdef SerialDebug
    Serial.println("mounted file system");
    #endif
    if (SPIFFS.exists("/config.json"))
    {
      #ifdef SerialDebug
      Serial.println("reading config file");
      #endif
      File configFile = SPIFFS.open("/config.json", "r");
      if (configFile)
      {
        #ifdef SerialDebug
        Serial.println("opened config file");
        #endif
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
            #ifdef SerialDebug
            Serial.println("\nparsed json");
            #endif
            strcpy(MQTT_BROKER, json["mqtt_server"]);
            strcpy(MQTT_PORT, json["mqtt_port"]);
          } else
          {
            #ifdef SerialDebug
            Serial.println("failed to load json config");
            #endif
        }
        configFile.close();
      }
    }
  } else {
    #ifdef SerialDebug
    Serial.println("failed to mount FS");
    #endif
  }

  WiFiManagerParameter custom_mqtt_server("server", "mqtt server", MQTT_BROKER, 40);
  WiFiManagerParameter custom_mqtt_port("port", "mqtt port", MQTT_PORT, 6);

  wifiManager.setSaveConfigCallback(saveConfigCallback);

  wifiManager.addParameter(&custom_mqtt_server);
  wifiManager.addParameter(&custom_mqtt_port);

  wifiManager.autoConnect("Haustuer");

  strcpy(MQTT_BROKER, custom_mqtt_server.getValue());
  strcpy(MQTT_PORT, custom_mqtt_port.getValue());

  if (shouldSaveConfig)
  {
    #ifdef SerialDebug
    Serial.println("saving config");
    #endif
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
      #ifdef SerialDebug
      Serial.println("failed to open config file for writing");
      #endif
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
  
  #ifdef SerialDebug
  Serial.println("local ip");
  Serial.println(WiFi.localIP());
  #endif
  
  ArduinoOTA.setHostname("Haustuer");
  ArduinoOTA.begin();

  #ifdef SerialDebug
  Serial.println("Connecting MQTT");
  Serial.print((char *)MQTT_BROKER);
  Serial.print(":");
  Serial.println(atoi(MQTT_PORT));
  #endif
  client.setServer((char *)MQTT_BROKER, atoi(MQTT_PORT));
  client.setCallback(callback);
  client.connect("HaustuerV2");
  client.subscribe(modeTopic);
  client.subscribe(doorOpenerTopic);
  
  os_timer_setfn(&Timer1, timerCallback, &Counter);
  os_timer_arm(&Timer1, 30000, true);

  digitalWrite(dooropenerPin, HIGH);
  digitalWrite(modePin,!EEPROM.read(0));

  attachInterrupt(digitalPinToInterrupt(doorbellInput), doorbellDebounce, CHANGE);
}

/*
 * Loop
 */
void loop()
{
  if (!client.connected())
  {
    #ifdef SerialDebug
    Serial.println("disconnected, retrying");
    #endif
    client.connect("HaustuerV2");
    client.subscribe(modeTopic);
    client.subscribe(doorOpenerTopic);
  }

  if (TickOccured)
  {
    #ifdef SerialDebug
    Serial.println("Heartbeat Time!");
    #endif
    if (client.connected())
    {
      snprintf (msgToPublish, 60, "%d", EEPROM.read(0));
      client.publish(statusTopic, msgToPublish);
    }
    TickOccured = false;
  }

  if (doorbellTriggered)
  {
    #ifdef SerialDebug
    Serial.println("hat geklingelt");
    #endif
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
