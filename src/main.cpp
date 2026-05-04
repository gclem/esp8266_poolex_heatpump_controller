#include <Arduino.h>
#include <secrets.h>

#define HOST_NAME "esp8266_poolheater_controller"

// UART PROTOCOL — GPIO (Wemos D1 Mini)
// Câblage actuel : RO→D2, DI→D1, DE/RE→D5
#define UART_RTS 14  // GPIO14 = D5 : MAX485 DE/RE
#define UART_TX   5  // GPIO5  = D1 : MAX485 DI
#define UART_RX   4  // GPIO4  = D2 : MAX485 RO
#define PORT_SPEED 9600

// Protocol bus — trames identifiées par analyse live (mai 2026)
//
//  Cycle bus (~190ms) — pattern alternant :
//   [PAC→remote] [flags] [état ~14B][capteurs 9B] : trame complète
//   [remote→PAC] XX 04 FF ... (12B) : heartbeat télécommande, constant
//   [PAC→remote] FD XX ... (13B) : ACK PAC
//   ... (le cycle se répète)
//
//  Séparateur inter-trames : octets 0x01 (idle bus)
//
//  Structure trame PAC : [flag1: 00|80] [flag2 opt: FC|06|08|88|5C] <état> <capteurs 9B>
//  Les 9 DERNIERS octets = capteurs (terminés par FF FF) :
//    [0] water_in_temp    = ((0xFF - x) >> 3) & 0x3F
//    [1] air_ambient_temp = ((0xFF - x) >> 1) & 0x3F
//    [2] coil_temp        = ((0xFF - x) >> 1) & 0x3F
//    [3] gas_temp         = ((0xFF - x) >> 1) & 0x3F
//    [4] water_out_temp   = ((0xFF - x) >> 1) & 0x3F
//    [5] constant 0xFF
//    [6] active_status    = ((0xFF - x) >> 4) & 0x01
//    [7-8] constant FF FF (terminaison)
//
//  La partie AVANT les 9B = état/config machine (consigne, mode — à décoder)
#define FRAME_IDLE_BYTE       0x01  // séparateur inter-trames sur le bus
#define FRAME_SYNC_BYTE       0xFF  // parfois capturé isolément en début de burst
#define SENSOR_TAIL_LEN        9    // capteurs = 9 derniers octets du burst (terminé FF FF)
#define SENSOR_TEMP_MIN        5    // °C — borne basse plausible pour validation
#define SENSOR_TEMP_MAX       45    // °C — borne haute plausible pour validation

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

// TX PROBE STATE
volatile bool probe_pending = false;
static uint8_t probe_counter = 0x01;
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
char *MQTT_TOPIC_VALUES_SETPOINT = "poolheater/values/setpoint";
char *MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED = "poolheater/values/setpoint_confirmed";
// Partie état/config (burst long sans les 9B capteurs) — pour analyse future
char *MQTT_TOPIC_VALUES_STATE_RAW = "poolheater/values/state_raw";
// MQTT COMMANDS
char *MQTT_TOPIC_COMMAND_PROBE = "poolheater/command/probe";


// FUNC DECLARATIONS
void sniffing();
void processBurst(const uint8_t *burst, int len);
bool decodeSensorTail(const uint8_t *tail);
void sendProbeHB();
String hexDump(const uint8_t *buf, int len);
void connectToMQTTBroker();
void pushMQTTMessage(char *topic, const char *message);
void pushMQTTValue(char *topic, int value);
void mqttReceiveCallback(char *topic, byte *payload, unsigned int length);
void pushMQTTValue(char *topic, int value);

void setup()
{
  // MAX485 en mode réception IMMÉDIATEMENT — avant tout le reste
  // pour ne jamais corrompre le bus RS485, même si le WiFi échoue.
  pinMode(UART_RTS, OUTPUT);
  digitalWrite(UART_RTS, LOW);

  Serial.begin(115200);
  delay(1000);

  // Setting up CPU Freq
  system_update_cpu_freq(160);

  // Init WIFI
  Serial.println("**** Setup: initializing ...");
  WiFi.setOutputPower(20.5); // Full power
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  Serial.println("");

  // Waiting for Wifi to initialize (timeout 30s)
  int wifi_attempts = 0;
  while (WiFi.status() != WL_CONNECTED && wifi_attempts < 30)
  {
    delay(1000);
    Serial.print(".");
    wifi_attempts++;
  }

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("\nWiFi failed, restarting...");
    ESP.restart();
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

  pubsubClient.loop();

  // Si un probe est en attente, l'envoyer maintenant
  if (probe_pending)
  {
    sendProbeHB();
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

// Buffer résiduel — conserve les bytes non terminés entre les appels readBytes.
// Nécessaire car le timeout de 50ms coupe souvent une trame en plusieurs lectures.
static uint8_t residual[300];
static int residual_len = 0;

// Lit le bus RS485 et découpe le flux en bursts (séquences non-0x01).
// Accumule les bytes dans un buffer résiduel pour gérer les trames coupées par le timeout.
void sniffing()
{
  const int READ_SIZE = 250;
  uint8_t buf[READ_SIZE];

  if (PS.available() <= 0) return;

  Debug.handle();
  ArduinoOTA.handle();
  pubsubClient.loop();

  int size = PS.readBytes((char *)buf, READ_SIZE);
  if (size <= 0) return;

  debugV("raw(%d): %s", size, hexDump(buf, size).c_str());

  // Ajoute les nouvelles données au buffer résiduel
  int copy_len = min(size, (int)(sizeof(residual) - residual_len));
  memcpy(residual + residual_len, buf, copy_len);
  residual_len += copy_len;

  // Traite les bursts complets (terminés par au moins un 0x01)
  int i = 0;
  int last_processed = 0;

  while (i < residual_len)
  {
    // Sauter les idle bytes
    while (i < residual_len && residual[i] == FRAME_IDLE_BYTE) i++;
    if (i >= residual_len) { last_processed = i; break; }

    int burst_start = i;
    while (i < residual_len && residual[i] != FRAME_IDLE_BYTE) i++;

    if (i >= residual_len)
    {
      // Pas de 0x01 trouvé après ce burst → incomplet, garder pour la prochaine lecture
      last_processed = burst_start;
      break;
    }

    // Burst complet (suivi d'un 0x01)
    int burst_len = i - burst_start;
    if (burst_len >= 3)
      processBurst(residual + burst_start, burst_len);

    last_processed = i;
  }

  // Compacte le résiduel : garde uniquement les bytes non traités
  if (last_processed > 0)
  {
    residual_len -= last_processed;
    if (residual_len > 0)
      memmove(residual, residual + last_processed, residual_len);
  }

  // Sécurité : si le buffer déborde sans aucun idle byte, on le vide
  if (residual_len >= (int)sizeof(residual) - READ_SIZE)
  {
    debugE("Residual buffer overflow, flushing %d bytes", residual_len);
    residual_len = 0;
  }

  yield();
}

// Identifie et traite un burst selon son type.
void processBurst(const uint8_t *burst, int len)
{
  // Heartbeat télécommande → PAC : XX [04|0C] FF ... (12B)
  // byte[1]: 0x04=normal, 0x0C=commande power en cours
  // byte[4] = consigne demandée: (0xFF - x) >> 1 = °C
  // byte[3] = consigne confirmée PAC: même formule
  if (len == 12 && (burst[1] == 0x04 || burst[1] == 0x0C) && burst[2] == 0xFF)
  {
    int setpoint_requested  = ((0xFF - burst[4]) >> 1) & 0x3F;
    int setpoint_confirmed  = ((0xFF - burst[3]) >> 1) & 0x3F;
    bool power_cmd = (burst[1] == 0x0C);
    debugD("Remote HB  : setpoint=%d°C (confirmed=%d°C) power_cmd=%d",
           setpoint_requested, setpoint_confirmed, power_cmd);
    debugV("Remote HB raw: %s", hexDump(burst, len).c_str());
    pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT, setpoint_requested);
    pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED, setpoint_confirmed);
    return;
  }

  // ACK PAC → télécommande : [FD|FF] F1 FF D5 ... (13B)
  // byte[0] varie (FD ou FF selon timing), on identifie par len + bytes[1,3]
  if (len == 13 && burst[1] == 0xF1 && burst[3] == 0xD5)
  {
    debugV("PAC ACK    : %s", hexDump(burst, len).c_str());
    return;
  }

  // Trames PAC data : commencent par FF, 0x00 ou 0x80 (flag byte)
  // Les 9 DERNIERS octets = capteurs (si le burst se termine par ... FF [status] FF FF)
  // tail[5]=FF est à burst[len-4], tail[7]=FF à burst[len-2], tail[8]=FF à burst[len-1]
  if (burst[0] == FRAME_SYNC_BYTE || burst[0] == 0x00 || burst[0] == 0x80)
  {
    bool has_sensor_tail = (len >= SENSOR_TAIL_LEN) &&
                           (burst[len - 1] == 0xFF) &&
                           (burst[len - 2] == 0xFF) &&
                           (burst[len - 4] == 0xFF);  // tail[5]=FF constant

    if (has_sensor_tail)
    {
      int sensor_start = len - SENSOR_TAIL_LEN;

      // Décoder les capteurs (9B en fin)
      if (!decodeSensorTail(burst + sensor_start))
        debugV("Sensor INVALID: %s", hexDump(burst + sensor_start, SENSOR_TAIL_LEN).c_str());

      // La partie avant = flags + état/config machine (pour analyse future)
      if (sensor_start > 0)
        debugV("State      (%2dB): %s", sensor_start, hexDump(burst, sensor_start).c_str());
    }
    else
    {
      // Burst sans terminaison FF FF — log pour analyse
      debugV("PAC-noterm (%2dB): %s", len, hexDump(burst, len).c_str());
    }
    return;
  }

  // Trame courte (flags isolés)
  if (len <= 4)
  {
    debugV("Short(%dB)  : %s", len, hexDump(burst, len).c_str());
    return;
  }

  // Sensor tail isolé (9B, terminé FF FF, sans flag en tête)
  if (len == SENSOR_TAIL_LEN && burst[len - 1] == 0xFF && burst[len - 2] == 0xFF && burst[len - 4] == 0xFF)
  {
    if (!decodeSensorTail(burst))
      debugV("Unknown    (%2dB): %s", len, hexDump(burst, len).c_str());
    return;
  }

  debugV("Unknown    (%2dB): %s", len, hexDump(burst, len).c_str());
}

// Décode les 9 octets capteurs (fin de burst, terminés FF FF).
// Formules confirmées avec mesures réelles (eau=17°C, air=15°C, mai 2026).
bool decodeSensorTail(const uint8_t *tail)
{
  int water_in    = ((0xFF - tail[0]) >> 3) & 0x3F;
  int air_ambient = ((0xFF - tail[1]) >> 1) & 0x3F;
  int coil_temp   = ((0xFF - tail[2]) >> 1) & 0x3F;
  int gas_temp    = ((0xFF - tail[3]) >> 1) & 0x3F;
  int water_out   = ((0xFF - tail[4]) >> 1) & 0x3F;
  // tail[5] = 0xFF constant
  // tail[6] status bits: inv=0x16(on+heating) vs inv=0x10(on+standby)
  uint8_t status_inv = 0xFF - tail[6];
  int power_on    = (status_inv >> 4) & 0x01;  // bit4 = power
  int heating     = (status_inv >> 1) & 0x01;  // bit1 = compressor/heating
  // tail[7-8] = FF FF terminaison

  if (water_in  < SENSOR_TEMP_MIN || water_in  > SENSOR_TEMP_MAX) return false;
  if (water_out < SENSOR_TEMP_MIN || water_out > SENSOR_TEMP_MAX) return false;

  debugD("Sensor     : water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C power=%d heating=%d",
         water_in, water_out, air_ambient, coil_temp, gas_temp, power_on, heating);

  pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP,    water_in);
  pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP,   water_out);
  pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP,        coil_temp);
  pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP,         gas_temp);
  pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient);
  pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS,    power_on);
  return true;
}

// --- TX Probe ---
// Envoie un HB de "2ème télécommande" pour tester si la PAC répond.
// Format identique au HB observé, mais avec un ID différent (0x6_ au lieu de 0x5_).

void sendProbeHB()
{
  // HB template basé sur l'observation: XX 04 FF C7 C7 C9 FB FF AF DD 0B FD
  // On utilise 0x6_ comme ID de "2ème télécommande"
  uint8_t hb[12] = {
    (uint8_t)(0x60 | (probe_counter & 0x0F)),  // ID=0x6_ + counter
    0x04,  // keepalive normal
    0xFF,
    0xC7,  // consigne confirmée (28°C par défaut)
    0xC7,  // consigne demandée (28°C par défaut)
    0xC9,
    0xFB,
    0xFF,
    0xAF,
    0xDD,
    0x0B,
    0xFD
  };

  probe_counter += 2;  // incrémente comme la vraie télécommande

  debugI("TX PROBE: %s", hexDump(hb, 12).c_str());

  // Basculer MAX485 en TX
  digitalWrite(UART_RTS, HIGH);
  delayMicroseconds(100);

  // Envoyer le HB
  PS.write(hb, 12);
  PS.flush();

  // Attendre fin de transmission: 12 bytes * 10 bits / 9600 baud ≈ 12.5ms
  delay(15);

  // Repasser en RX
  digitalWrite(UART_RTS, LOW);

  debugI("TX PROBE sent, listening for PAC response...");
  probe_pending = false;
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
      pubsubClient.subscribe(MQTT_TOPIC_COMMAND_PROBE);
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

  if(strcmp(topic, MQTT_TOPIC_COMMAND_FRAME_TIMEOUT) == 0)
  {
    char buff_p[length + 1];
    for (unsigned int i = 0; i < length; i++)
      buff_p[i] = (char)payload[i];
    buff_p[length] = '\0';
    int timeout = String(buff_p).toInt();
    debugI("Receiving new frame timeout : %d", timeout);
    PS.setTimeout(timeout);
  }
  else if(strcmp(topic, MQTT_TOPIC_COMMAND_PROBE) == 0)
  {
    debugI("PROBE command received — will send test HB on next loop");
    probe_pending = true;
  }
}