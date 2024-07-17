#include <Arduino.h>
#include <secrets.h>

#define HOST_NAME "esp8266_poolheater_controller"

// UART PROTOCOL CONSTANT
#define UART_RTS 2
#define UART_TX 1 // 5
#define UART_RX 3 // 4
#define PORT_SPEED 9600

// COMMUNICATIONS PARAMETERS
#define USE_MDNS true
#define MAX_BUFFER_SIZE 195
#define MAX_MSG_SIZE 132

// IMPORT
#include <SoftwareSerial.h>
#include "RemoteDebug.h"
#include <ESP8266WiFi.h>
#include <ArduinoOTA.h>
#include <PubSubClient.h>

// COMMUNICATIONS VAR
SoftwareSerial PS;
WiFiClient wifiClient;
PubSubClient pubsubClient(wifiClient);

// DEBUGGER VAR
RemoteDebug Debug;

// BUFFER
String message = "";
const char *c_msg = "";

// MQTT TOPICS
char *MQTT_TOPIC_RAW_MSG_HEX = "poolheater/modbus/raw/pump/hex";
char *MQTT_TOPIC_DEBUG_MSG = "poolheater/debug";
char *MQTT_TOPIC_STATUS = "poolheater/status";
char *MQTT_TOPIC_COMMAND_FRAME_TIMEOUT = "poolheater/command/frame/timeout";
// MQTT VALUES
char *MQTT_TOPIC_VALUES_WATER_IN_TEMP = "poolheater/values/water_in_temp";
char *MQTT_TOPIC_VALUES_WATER_OUT_TEMP = "poolheater/values/water_out_temp";
char *MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP = "poolheater/values/air_ambient_temp";
char *MQTT_TOPIC_VALUES_COIL_TEMP = "poolheater/values/coil_temp";
char *MQTT_TOPIC_VALUES_GAZ_TEMP = "poolheater/values/gaz_temp";
char *MQTT_TOPIC_VALUES_ACTIVE_STATUS = "poolheater/values/active_status";


// FUNC DECLARATIONS
void sniffing();
void connectToMQTTBroker();
void pushMQTTMessage(char *topic, const char *message);
void mqttReceiveCallback(char *topic, byte *payload, unsigned int length);
void pushMQTTValue(char *topic, int value);

void setup()
{

  // Setting up CPU Freq
  system_update_cpu_freq(160);

  // Init WIFI
  Serial.println("**** Setup: initializing ...");
  WiFi.setOutputPower(20.5); // Full power
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("");

  // Waiting for Wifi to initialize
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(1000);
    Serial.print(".");
  }

  // Wifi Status
  Serial.println("");
  Serial.print("Connected to IP address: ");
  Serial.println(WiFi.localIP());
  WiFi.setAutoReconnect(true);
  // WiFi.persistent(true);

  // Init Debugger -- REMOTE APP : http://joaolopesf.net/remotedebugapp
  Serial.println("Remote debugger is initializing...");
  Serial.println(HOST_NAME);
  Debug.begin(HOST_NAME);
  Debug.setResetCmdEnabled(true);
  Debug.showProfiler(true);
  Debug.showColors(true);
  Serial.println("Remote debugger initialized.");

  // OTA
  ArduinoOTA.setHostname(HOST_NAME);
  ArduinoOTA.begin();
  ArduinoOTA.onStart([]()
                     { Serial.println("Starting OTA..."); });
  ArduinoOTA.onEnd([]()
                   { Serial.println("\nOTA Ended."); });
  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
                        { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); });
  ArduinoOTA.onError([](ota_error_t error)
                     {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });
  ArduinoOTA.begin();

  // Init MQTT
  pubsubClient.setServer(MQTT_BROKER_ADDR, 1883);
  pubsubClient.setCallback(mqttReceiveCallback);
  pubsubClient.setBufferSize(512);
  connectToMQTTBroker();

  // Init UART Protocol
  Serial.println("UART Protocol is initializing...");
  pinMode(UART_RTS, OUTPUT);
  digitalWrite(UART_RTS, LOW);
  PS.setTimeout(50);
  PS.begin(PORT_SPEED, SWSERIAL_8N1, UART_RX, UART_TX, false);
  Serial.println("UART Protocol is listening...");
}

void loop()
{

  Debug.handle();
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WIFI disconnected. Restarting ESP");
    ESP.restart();
  }

  if (!pubsubClient.connected())
  {
    connectToMQTTBroker();
  }

  // sniffing UART
  sniffing();
}

int offset = 0;
void sniffing()
{

  const int BUFFER_SIZE = 250;
  char buf[BUFFER_SIZE];
  int size = 0;
  String msg_dump;
  int values[6];

  while (PS.available() > 0)
  {
    Debug.handle();
    ArduinoOTA.handle();
    pubsubClient.loop();

    size = PS.readBytes(buf, BUFFER_SIZE);
    msg_dump = "";

    // Debugging - print all messages 
    if (size > 0)
    {
      debugV("size of buffer : %d", size);

      for (uint8_t i = 0; i < size; i++)
      {
        if (buf[i] < 0x10)
          msg_dump += "0";
        msg_dump += String(buf[i], HEX);
        msg_dump += " ";
      }
      msg_dump.toUpperCase();
      debugV("%s", msg_dump.c_str());
    }

    // From pump 
    if (size >= 49 && size <= 51)
    {
      // Frame type : 1C DD DD D3 C9 FF FF FF FF 01
      debugI("Trame containing values : %s", msg_dump.c_str());
      //water_in_temp       = ((0xFF - myFrame[1]) >> 3) & 0b111111
      values[0] = ((0xFF - buf[0]) >> 3) & 0b111111;
      //air_ambient_temp    = ((0xFF - myFrame[2]) >> 1) & 0b111111
      values[1] = ((0xFF - buf[1]) >> 1) & 0b111111;
      //coil_temp           = ((0xFF - myFrame[3]) >> 1) & 0b111111
      values[2] = ((0xFF - buf[2]) >> 1) & 0b111111;
      //gas_exhaust_temp    = ((0xFF - myFrame[4]) >> 1) & 0b111111
      values[3] = ((0xFF - buf[3]) >> 1) & 0b111111;
      //water_out_temp      = ((0xFF - myFrame[5]) >> 1) & 0b111111
      values[4] = ((0xFF - buf[4]) >> 1) & 0b111111;
      //active              = ((0xFF - myFrame[7]) >> 4) & 0b000001
      values[5] = ((0xFF - buf[6]) >> 4) & 0b000001;
      // Push to MQTT
      debugI("Pushing values, water in : %d, water out : %d, coil temp : %d, air temp : %d", values[0], values[4], values[2], values[1]);
      pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP, values[0]);
      pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP, values[4]);
      pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP, values[2]);
      pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP, values[3]);
      pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, values[1]);
      pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS, values[5]);
    }

    // Slowing the loop
    yield();
    // very important : need to be synchronized with your frame size and rx speed 
    delay(100);
  }
}

void connectToMQTTBroker()
{

  if (!pubsubClient.connected())
  {
    Debug.printf("Connecting to MQTT Broker as %s.....\n", HOST_NAME);
    if (pubsubClient.connect(HOST_NAME, HOST_NAME, HOST_NAME))
    {
      Debug.println("Connected to MQTT broker.");
      pubsubClient.publish(MQTT_TOPIC_STATUS, "ON", true);
      pubsubClient.subscribe(MQTT_TOPIC_COMMAND_FRAME_TIMEOUT);
    }
    else
    {
      Debug.print("Failed to connect to MQTT broker, rc=");
      Debug.print(pubsubClient.state());
      Debug.println("Trying again to connect to MQTT broker... next loop");
      yield();
    }
  }
}

void pushMQTTMessage(char *topic, const char *msg)
{
  if (!pubsubClient.connected())
  {
    debugE("Pubsub client not connected - avoiding to push message %s : %s", topic, msg);
  }
  else
  {
    pubsubClient.publish(topic, msg);
  }
}

void pushMQTTValue(char *topic, int value)
{
  if (!pubsubClient.connected())
  {
    debugE("Pubsub client not connected - avoiding to push values.");
  }
  else
  {
    pubsubClient.publish(String(topic).c_str(), String(value).c_str());
  }
}

void mqttReceiveCallback(char *topic, byte *payload, unsigned int length)
{
  debugV("Receiving new message from MQTT from %s, value size: %d", topic, length);
  String messageTemp;

  if(strcmp(topic, MQTT_TOPIC_COMMAND_FRAME_TIMEOUT) == 0)
  {
    char buff_p[length];
    int timeout = 100;
    for (int i = 0; i < length; i++)
    {
      buff_p[i] = (char)payload[i];
    }
    buff_p[length] = '\0';
    String msg_p = String(buff_p);
    timeout = msg_p.toInt();
    
    debugI("Receiving new frame timeout : %d", timeout);
    // receiving frame timeout in ms
    PS.setTimeout(timeout);
  }
}