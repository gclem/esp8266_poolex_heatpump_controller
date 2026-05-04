#include <Arduino.h>
#include <secrets.h>

#define HOST_NAME "esp8266_poolheater_controller"

// UART PROTOCOL — GPIO (Wemos D1 Mini)
// IMPORTANT: GPIO2 est HIGH au boot (strapping pin) → MAX485 aurait été en TX pendant le démarrage.
// GPIO14/D5, GPIO5/D1, GPIO4/D2 sont LOW/haute-impédance au boot → sûrs pour le bus RS485.
// ⚠️  Recâbler en conséquence : DE/RE → D5, DI → D1, RO → D2
#define UART_RTS 14  // GPIO14 = D5 : MAX485 DE/RE — LOW au boot, MAX485 en RX dès le démarrage
#define UART_TX   5  // GPIO5  = D1 : MAX485 DI    — découplé du Serial hardware (GPIO1)
#define UART_RX   4  // GPIO4  = D2 : MAX485 RO
#define PORT_SPEED 9600

// Protocol bus — trames identifiées par analyse des captures (sniffing1..6.txt)
//
//  Cycle bus (~190ms) :
//   [remote→PAC] 0x19 (12B) : heartbeat télécommande, constant
//   [PAC→remote] 0xFD (13B) : ACK PAC
//   [PAC→remote] 0xFF [flags] 0x2C ... (11-15B) : trame config courte (setpoint ?)
//   [PAC→remote] 0xFF [flags] <data> (29-36B)   : trame capteurs (températures)
//
//  Séparateur inter-trames : octets 0x01 (idle bus)
//
//  Décodage températures (trame capteurs, après FF + flag bytes) :
//    data[0] → water_in_temp    = ((0xFF - data[0]) >> 3) & 0b111111
//    data[1] → air_ambient_temp = ((0xFF - data[1]) >> 1) & 0b111111
//    data[2] → coil_temp        = ((0xFF - data[2]) >> 1) & 0b111111
//    data[3] → gas_temp         = ((0xFF - data[3]) >> 1) & 0b111111
//    data[4] → water_out_temp   = ((0xFF - data[4]) >> 1) & 0b111111
//    data[6] → active_status    = ((0xFF - data[6]) >> 4) & 0b000001
#define FRAME_IDLE_BYTE    0x01  // séparateur inter-trames sur le bus
#define FRAME_SYNC_BYTE    0xFF  // octet de synchronisation précédant les trames PAC
#define FRAME_HDR_REMOTE   0x19  // heartbeat télécommande → PAC (12B)
#define FRAME_HDR_REMOTE_2 0x04  // 2ème octet fixe du heartbeat
#define FRAME_HDR_PAC_ACK  0xFD  // ACK PAC → télécommande (13B)
#define FRAME_HDR_PAC_ACK2 0xF5  // 2ème octet fixe du ACK
#define SENSOR_FRAME_MIN_LEN 25  // longueur minimale d'un burst FF pour contenir des capteurs
#define SENSOR_TEMP_MIN      5   // °C — borne basse plausible pour validation
#define SENSOR_TEMP_MAX     45   // °C — borne haute plausible pour validation

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
void processBurst(const uint8_t *burst, int len);
bool processSensorBurst(const uint8_t *burst, int len);
String hexDump(const uint8_t *buf, int len);
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

// Retourne un dump hexadécimal lisible d'un buffer.
String hexDump(const uint8_t *buf, int len)
{
  String out = "";
  for (int i = 0; i < len; i++)
  {
    if (buf[i] < 0x10) out += "0";
    out += String(buf[i], HEX);
    out += " ";
  }
  out.toUpperCase();
  return out;
}

// Lit le bus RS485 et découpe le flux en bursts (séquences non-0x01).
// Chaque burst est traité indépendamment par processBurst().
void sniffing()
{
  const int BUFFER_SIZE = 250;
  uint8_t buf[BUFFER_SIZE];
  int size = 0;

  while (PS.available() > 0)
  {
    Debug.handle();
    ArduinoOTA.handle();
    pubsubClient.loop();

    size = PS.readBytes((char *)buf, BUFFER_SIZE);
    if (size <= 0) { yield(); continue; }

    debugV("raw(%d): %s", size, hexDump(buf, size).c_str());

    // Découpe le buffer en bursts séparés par les octets 0x01 (idle bus).
    int i = 0;
    while (i < size)
    {
      while (i < size && buf[i] == FRAME_IDLE_BYTE) i++;
      if (i >= size) break;

      int burst_start = i;
      while (i < size && buf[i] != FRAME_IDLE_BYTE) i++;

      int burst_len = i - burst_start;
      if (burst_len >= 3)
        processBurst(buf + burst_start, burst_len);
    }

    yield();
    // Délai de synchronisation avec le rythme du bus (9600 baud, ~98ms entre cycles)
    delay(100);
  }
}

// Identifie et traite un burst selon son type.
void processBurst(const uint8_t *burst, int len)
{
  // Heartbeat télécommande → PAC : 19 04 FF C7 C9 C9 FB FF AF DD 0B FD (12B, constant)
  if (len == 12 && burst[0] == FRAME_HDR_REMOTE && burst[1] == FRAME_HDR_REMOTE_2)
  {
    debugV("Remote HB  : %s", hexDump(burst, len).c_str());
    return;
  }

  // ACK PAC → télécommande : FD F5 C3 D3 FF ... (13B)
  if (len >= 13 && burst[0] == FRAME_HDR_PAC_ACK && burst[1] == FRAME_HDR_PAC_ACK2)
  {
    debugV("PAC ACK    : %s", hexDump(burst, len).c_str());
    return;
  }

  // Trames PAC préfixées FF [flags] <data> : capteurs (>=25B) ou config courte (<25B)
  if (burst[0] == FRAME_SYNC_BYTE)
  {
    if (len >= SENSOR_FRAME_MIN_LEN)
    {
      if (!processSensorBurst(burst, len))
        debugV("FF-long INVALID(%dB): %s", len, hexDump(burst, len).c_str());
    }
    else
    {
      // Trame courte (config/setpoint PAC → remote) : à décoder dans une future itération
      debugV("FF-short   (%2dB): %s", len, hexDump(burst, len).c_str());
    }
    return;
  }

  // Trame courte 3B inconnue (sync/horloge ?)
  if (len == 3 && burst[2] == FRAME_SYNC_BYTE)
  {
    debugV("Short(3B)  : %s", hexDump(burst, len).c_str());
    return;
  }

  debugV("Unknown    (%2dB): %s", len, hexDump(burst, len).c_str());
}

// Décode la trame capteurs depuis un burst FF-préfixé.
// Structure : FF [0x00|0x80] [flag optionnel: 0x06/0x08/0xFC] <data...>
// Retourne true si les valeurs sont plausibles et ont été publiées sur MQTT.
bool processSensorBurst(const uint8_t *burst, int len)
{
  // Sauter le FF + les flag bytes
  int d = 1;
  if (d < len && (burst[d] == 0x00 || burst[d] == 0x80))        d++;
  if (d < len && (burst[d] == 0xFC || burst[d] == 0x06 ||
                  burst[d] == 0x08 || burst[d] == 0x28))         d++;
  if (d + 7 > len) return false;

  int water_in    = ((0xFF - burst[d    ]) >> 3) & 0b111111;
  int air_ambient = ((0xFF - burst[d + 1]) >> 1) & 0b111111;
  int coil_temp   = ((0xFF - burst[d + 2]) >> 1) & 0b111111;
  int gas_temp    = ((0xFF - burst[d + 3]) >> 1) & 0b111111;
  int water_out   = ((0xFF - burst[d + 4]) >> 1) & 0b111111;
  int active      = ((0xFF - burst[d + 6]) >> 4) & 0b000001;

  if (water_in  < SENSOR_TEMP_MIN || water_in  > SENSOR_TEMP_MAX) return false;
  if (water_out < SENSOR_TEMP_MIN || water_out > SENSOR_TEMP_MAX) return false;

  debugI("Sensor — water_in:%d air:%d coil:%d gas:%d water_out:%d active:%d",
         water_in, air_ambient, coil_temp, gas_temp, water_out, active);

  pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP,    water_in);
  pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP,   water_out);
  pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP,        coil_temp);
  pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP,         gas_temp);
  pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient);
  pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS,    active);
  return true;
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