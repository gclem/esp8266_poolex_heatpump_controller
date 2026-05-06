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

// TX PROBE STATE
volatile bool probe_pending = false;
static uint8_t probe_counter = 0x01;

// MQTT rate limiting — ne publie que si la valeur a changé OU toutes les 60s
static int last_water_in = -1, last_water_out = -1, last_air = -1;
static int last_coil = -1, last_gas = -1, last_power = -1, last_heating = -1;
static int last_setpoint = -1, last_setpoint_confirmed = -1;
static int last_mode = -1;
static String last_error = "";
static unsigned long last_error_seen_ms = 0;  // dernier 0x40 reçu
const unsigned long ERROR_CLEAR_DELAY_MS = 10000;  // 10s sans 0x40 pour clear l'erreur
static unsigned long last_full_publish_ms = 0;
const unsigned long MQTT_FULL_PUBLISH_INTERVAL_MS = 60000; // republish toutes les 60s

// MQTT reconnect throttle
static unsigned long mqtt_last_attempt = 0;
const unsigned long MQTT_RECONNECT_INTERVAL_MS = 5000;
// MQTT TOPICS
const char *MQTT_TOPIC_STATUS = "poolheater/status";
// MQTT VALUES
const char *MQTT_TOPIC_VALUES_WATER_IN_TEMP = "poolheater/values/water_in_temp";
const char *MQTT_TOPIC_VALUES_WATER_OUT_TEMP = "poolheater/values/water_out_temp";
const char *MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP = "poolheater/values/air_ambient_temp";
const char *MQTT_TOPIC_VALUES_COIL_TEMP = "poolheater/values/coil_temp";
const char *MQTT_TOPIC_VALUES_GAZ_TEMP = "poolheater/values/gaz_temp";
const char *MQTT_TOPIC_VALUES_ACTIVE_STATUS = "poolheater/values/active_status";
const char *MQTT_TOPIC_VALUES_HEATING = "poolheater/values/heating";
const char *MQTT_TOPIC_VALUES_SETPOINT = "poolheater/values/setpoint";
const char *MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED = "poolheater/values/setpoint_confirmed";
const char *MQTT_TOPIC_VALUES_MODE = "poolheater/values/mode";
const char *MQTT_TOPIC_VALUES_ERROR = "poolheater/values/error";
// Partie état/config (burst long sans les 9B capteurs) — pour analyse future
const char *MQTT_TOPIC_VALUES_STATE_RAW = "poolheater/values/state_raw";
// MQTT COMMANDS
const char *MQTT_TOPIC_COMMAND_PROBE = "poolheater/command/probe";


// FUNC DECLARATIONS
void sniffing();
void processBurst(const uint8_t *burst, int len);
bool decodeSensorTail(const uint8_t *tail);
void sendProbeHB();
const char* hexDumpFast(const uint8_t *buf, int len);
void connectToMQTTBroker();
void pushMQTTMessage(const char *topic, const char *message);
void pushMQTTValue(const char *topic, int value);
void mqttReceiveCallback(char *topic, byte *payload, unsigned int length);

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
  PS.begin(PORT_SPEED, SWSERIAL_8N1, UART_RX, UART_TX, false);
  Serial.println("UART Protocol is listening...");
}

// Flag indiquant que le bus est en idle (dernier traitement a fini sur des 0x01)
static bool bus_is_idle = false;

void loop()
{
  // Donner du temps au WiFi quand le bus est en idle (entre les trames).
  // On ne coupe JAMAIS le RX en plein milieu d'une trame.
  static unsigned long last_wifi_window = 0;
  unsigned long now = millis();
  if (bus_is_idle && (now - last_wifi_window >= 50))
  {
    last_wifi_window = now;
    PS.enableRx(false);
    delay(3); // 3ms WiFi window pendant l'idle bus
    PS.enableRx(true);
  }

  Debug.handle();
  ArduinoOTA.handle();

  if (WiFi.status() != WL_CONNECTED)
  {
    Serial.println("WIFI disconnected. Restarting ESP");
    ESP.restart();
  }

  if (!pubsubClient.connected())
  {
    if (millis() - mqtt_last_attempt > MQTT_RECONNECT_INTERVAL_MS)
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

// Retourne un dump hexadécimal lisible d'un buffer (buffer statique, pas d'allocation heap).
static char hex_buf[768]; // max 250 bytes * 3 chars + null
const char* hexDumpFast(const uint8_t *buf, int len)
{
  int pos = 0;
  for (int i = 0; i < len && pos < (int)sizeof(hex_buf) - 4; i++)
  {
    pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", buf[i]);
  }
  if (pos > 0) hex_buf[pos - 1] = '\0'; // enlève le dernier espace
  else hex_buf[0] = '\0';
  return hex_buf;
}

// Buffer résiduel — conserve les bytes non terminés entre les appels readBytes.
static uint8_t residual[300];
static int residual_len = 0;
static unsigned long residual_last_activity = 0;
const unsigned long RESIDUAL_TIMEOUT_MS = 100; // flush si pas de nouvelles données pendant 100ms

// Lit le bus RS485 et découpe le flux en bursts (séquences non-0x01).
// Accumule les bytes dans un buffer résiduel pour gérer les trames coupées.
// Limite la fréquence de traitement pour laisser le WiFi stack respirer.
void sniffing()
{
  const int READ_SIZE = 250;
  uint8_t buf[READ_SIZE];

  // Mini-throttle : lire au minimum toutes les 5ms pour accumuler des bytes
  // (~5 bytes à 9600 baud) au lieu de lire byte par byte
  static unsigned long last_read_ms = 0;
  unsigned long now = millis();

  int avail = PS.available();
  if (avail <= 0)
  {
    // Si le residual stagne sans terminateur, forcer le traitement
    if (residual_len > 0 && (now - residual_last_activity) > RESIDUAL_TIMEOUT_MS)
    {
      if (residual_len >= 3)
        processBurst(residual, residual_len);
      residual_len = 0;
    }
    bus_is_idle = (residual_len == 0);
    return;
  }

  // Ne lire que si au moins 5ms écoulées OU buffer > 20 bytes
  if (avail < 20 && (now - last_read_ms) < 5) return;

  last_read_ms = now;
  bus_is_idle = false;
  residual_last_activity = now;

  // Lire uniquement ce qui est disponible — ne jamais bloquer
  int to_read = min(avail, READ_SIZE);
  int size = PS.readBytes((char *)buf, to_read);
  if (size <= 0) return;

  debugV("raw(%d): %s", size, hexDumpFast(buf, size));

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
    yield(); // laisser le WiFi stack traiter entre chaque burst
  }

  // Compacte le résiduel : garde uniquement les bytes non traités
  if (last_processed > 0)
  {
    residual_len -= last_processed;
    if (residual_len > 0)
      memmove(residual, residual + last_processed, residual_len);
  }

  // Le bus est idle si on a fini de traiter tout le residual (que des 0x01 restants ou vide)
  bus_is_idle = (residual_len == 0);

  // Sécurité : si le buffer déborde sans aucun idle byte, on le vide sans traiter
  if (residual_len >= (int)sizeof(residual) - READ_SIZE)
  {
    debugV("Residual overflow, dropping %d bytes", residual_len);
    residual_len = 0;
  }

  yield();
}

// Identifie et traite un burst selon son type.
// Rate-limit les trames non reconnues pour éviter de saturer le debug/WiFi.
static unsigned long last_unknown_log_ms = 0;
static int unknown_count_since_log = 0;

void processBurst(const uint8_t *burst, int len)
{
  // Heartbeat télécommande → PAC : XX [04|0C] FF ... (12B)
  // Parfois le premier byte (counter) est perdu → 11B avec burst[0]=[04|0C]
  // byte[1]: 0x04=normal, 0x0C=commande power en cours
  // byte[4] = consigne demandée: (0xFF - x) >> 1 = °C
  // byte[3] = consigne confirmée PAC: même formule
  bool is_hb_12 = (len == 12 && (burst[1] == 0x04 || burst[1] == 0x0C) && burst[2] == 0xFF);
  bool is_hb_11 = (len == 11 && (burst[0] == 0x04 || burst[0] == 0x0C) && burst[1] == 0xFF);

  if (is_hb_12 || is_hb_11)
  {
    int offset = is_hb_11 ? -1 : 0; // décalage si counter manquant
    int setpoint_requested  = ((0xFF - burst[4 + offset]) >> 1) & 0x3F;
    int setpoint_confirmed  = ((0xFF - burst[3 + offset]) >> 1) & 0x3F;
    bool power_cmd = (burst[1 + offset] == 0x0C);
    debugD("Remote HB  : setpoint=%d°C (confirmed=%d°C) power_cmd=%d",
           setpoint_requested, setpoint_confirmed, power_cmd);
    debugV("Remote HB raw: %s", hexDumpFast(burst, len));
    bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
    if (force || setpoint_requested != last_setpoint)           { pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT, setpoint_requested);           last_setpoint = setpoint_requested; }
    if (force || setpoint_confirmed != last_setpoint_confirmed) { pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED, setpoint_confirmed); last_setpoint_confirmed = setpoint_confirmed; }
    return;
  }

  // ACK PAC → télécommande : contient F1 xx D5 et tout le reste est FF
  // Taille variable (8-13B) car parfois tronqué par le timing de lecture
  if (len >= 7 && len <= 13 && burst[1] == 0xF1 && burst[3] == 0xD5)
  {
    debugV("PAC ACK    : %s", hexDumpFast(burst, len));
    return;
  }
  // Variante tronquée : commence directement par F1
  if (len >= 6 && len <= 13 && burst[0] == 0xF1 && burst[2] == 0xD5)
  {
    debugV("PAC ACK    : %s", hexDumpFast(burst, len));
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
        debugV("Sensor INVALID: %s", hexDumpFast(burst + sensor_start, SENSOR_TAIL_LEN));

      // La partie avant = flags + état/config machine
      if (sensor_start > 0)
      {
        debugV("State      (%2dB): %s", sensor_start, hexDumpFast(burst, sensor_start));
        pushMQTTMessage(MQTT_TOPIC_VALUES_STATE_RAW, hexDumpFast(burst, sensor_start));

        // Décodage du mode PAC (dernier byte de la partie état, trames longues ≥15B)
        if (sensor_start >= 15)
        {
          uint8_t mode_byte = burst[sensor_start - 1];
          int mode = -1;
          switch (mode_byte) {
            case 0x6C: mode = 2; break; // Chauffage seul
            case 0x64: mode = 2; break; // Chauffage (variante transition)
            case 0x54: mode = 1; break; // Auto (chaud + froid)
            case 0x5C: mode = 0; break; // Refroidissement seul
          }
          if (mode >= 0)
          {
            bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
            if (force || mode != last_mode)
            {
              const char *mode_str = (mode == 2) ? "heat" : (mode == 1) ? "auto" : "cool";
              pushMQTTMessage(MQTT_TOPIC_VALUES_MODE, mode_str);
              last_mode = mode;
              debugD("Mode PAC: %s (byte=0x%02X)", mode_str, mode_byte);
            }
          }

          // Consigne confirmée par la PAC : byte[2] du state (quand burst commence par 00/80)
          // Formule identique à water_in : ((0xFF - x) >> 3) & 0x3F
          int sp_offset = (burst[0] == FRAME_SYNC_BYTE) ? 3 : 2;
          if (sp_offset < sensor_start)
          {
            int sp_confirmed = ((0xFF - burst[sp_offset]) >> 3) & 0x3F;
            if (sp_confirmed >= 15 && sp_confirmed <= 40)
            {
              bool force2 = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
              if (force2 || sp_confirmed != last_setpoint_confirmed)
              {
                pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED, sp_confirmed);
                last_setpoint_confirmed = sp_confirmed;
                debugD("Setpoint confirmed (PAC): %d°C", sp_confirmed);
              }
            }
          }
        }
      }
    }
    else
    {
      // Mode erreur PAC (PL/no flow): trame courte avec capteurs en format réduit
      // Pattern: [00|80] [flag] [error_code] [5 temps] FF FF  (10B)
      // Aussi : les 7 derniers octets [5 temps] FF FF sont le "sensor short tail"
      bool has_short_tail = (len >= 7) &&
                            (burst[len - 1] == 0xFF) &&
                            (burst[len - 2] == 0xFF);

      if (has_short_tail && len <= 12)
      {
        // Tenter le décodage en mode court (5 temp bytes + FF + FF)
        int tail_start = len - 7;
        const uint8_t *st = burst + tail_start;
        int water_in  = ((0xFF - st[0]) >> 3) & 0x3F;
        int water_out = ((0xFF - st[4]) >> 1) & 0x3F;
        if (water_in >= SENSOR_TEMP_MIN && water_in <= SENSOR_TEMP_MAX &&
            water_out >= SENSOR_TEMP_MIN && water_out <= SENSOR_TEMP_MAX)
        {
          int air_ambient = ((0xFF - st[1]) >> 1) & 0x3F;
          int coil_temp   = ((0xFF - st[2]) >> 1) & 0x3F;
          int gas_temp    = ((0xFF - st[3]) >> 1) & 0x3F;
          // En mode erreur, status = OFF (st[5]=FF, st[6]=FF → power=0)
          int power_on = 0;
          int heating  = 0;

          debugD("Sensor(err): water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C power=%d heating=%d",
                 water_in, water_out, air_ambient, coil_temp, gas_temp, power_on, heating);

          // Publier les valeurs
          bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
          if (force) last_full_publish_ms = millis();
          if (force || water_in != last_water_in)   { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP, water_in);     last_water_in = water_in; }
          if (force || water_out != last_water_out) { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP, water_out);   last_water_out = water_out; }
          if (force || air_ambient != last_air)     { pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient); last_air = air_ambient; }
          if (force || coil_temp != last_coil)      { pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP, coil_temp);         last_coil = coil_temp; }
          if (force || gas_temp != last_gas)        { pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP, gas_temp);           last_gas = gas_temp; }
          if (force || power_on != last_power)      { pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS, power_on);      last_power = power_on; }
          if (force || heating != last_heating)     { pushMQTTValue(MQTT_TOPIC_VALUES_HEATING, heating);             last_heating = heating; }
        }
        else
        {
          debugV("PAC-err    (%2dB): %s", len, hexDumpFast(burst, len));
        }
      }
      else
      {
        // Burst sans terminaison FF FF — état machine (mode erreur ou config)
        debugV("PAC-state  (%2dB): %s", len, hexDumpFast(burst, len));
        if (len >= 10) // assez de données pour être intéressant
        {
          pushMQTTMessage(MQTT_TOPIC_VALUES_STATE_RAW, hexDumpFast(burst, len));

          // Décodage du mode PAC sur trames d'état longues (≥15B)
          if (len >= 15)
          {
            uint8_t mode_byte = burst[len - 1];
            int mode = -1;
            switch (mode_byte) {
              case 0x6C: mode = 2; break; // Chauffage seul
              case 0x64: mode = 2; break; // Chauffage (variante transition)
              case 0x54: mode = 1; break; // Auto (chaud + froid)
              case 0x5C: mode = 0; break; // Refroidissement seul
            }
            if (mode >= 0)
            {
              bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
              if (force || mode != last_mode)
              {
                const char *mode_str = (mode == 2) ? "heat" : (mode == 1) ? "auto" : "cool";
                pushMQTTMessage(MQTT_TOPIC_VALUES_MODE, mode_str);
                last_mode = mode;
                debugD("Mode PAC: %s (byte=0x%02X)", mode_str, mode_byte);
              }
            }
          }
        }
      }
    }
    return;
  }

  // Trame courte (flags isolés, 0x40 flooding en mode erreur)
  if (len <= 4)
  {
    debugV("Short(%dB)  : %s", len, hexDumpFast(burst, len));
    return;
  }

  // Burst de 0x40 (signal erreur PAC, mode PL) — ignorer silencieusement
  if (burst[0] == 0x40)
  {
    debugV("PAC-alert  (%2dB): 0x40 flood", len);
    last_error_seen_ms = millis();
    // Publier l'état d'erreur (on ne connaît pas encore le code exact, mais 0x40 = erreur active)
    if (last_error != "PL")
    {
      pushMQTTMessage(MQTT_TOPIC_VALUES_ERROR, "PL");
      last_error = "PL";
      debugD("Error PAC: PL (0x40 flood detected)");
    }
    return;
  }

  // Sensor tail isolé (7B en mode erreur: 5 temps + FF FF, sans header)
  if (len == 7 && burst[len - 1] == 0xFF && burst[len - 2] == 0xFF)
  {
    int water_in  = ((0xFF - burst[0]) >> 3) & 0x3F;
    int water_out = ((0xFF - burst[4]) >> 1) & 0x3F;
    if (water_in >= SENSOR_TEMP_MIN && water_in <= SENSOR_TEMP_MAX &&
        water_out >= SENSOR_TEMP_MIN && water_out <= SENSOR_TEMP_MAX)
    {
      int air_ambient = ((0xFF - burst[1]) >> 1) & 0x3F;
      int coil_temp   = ((0xFF - burst[2]) >> 1) & 0x3F;
      int gas_temp    = ((0xFF - burst[3]) >> 1) & 0x3F;
      debugD("Sensor(iso): water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C",
             water_in, water_out, air_ambient, coil_temp, gas_temp);
      // Publier (power=off en mode erreur)
      bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
      if (force) last_full_publish_ms = millis();
      if (force || water_in != last_water_in)   { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP, water_in);     last_water_in = water_in; }
      if (force || water_out != last_water_out) { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP, water_out);   last_water_out = water_out; }
      if (force || air_ambient != last_air)     { pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient); last_air = air_ambient; }
      if (force || coil_temp != last_coil)      { pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP, coil_temp);         last_coil = coil_temp; }
      if (force || gas_temp != last_gas)        { pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP, gas_temp);           last_gas = gas_temp; }
      if (force || 0 != last_power)             { pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS, 0);             last_power = 0; }
      if (force || 0 != last_heating)           { pushMQTTValue(MQTT_TOPIC_VALUES_HEATING, 0);                   last_heating = 0; }
      return;
    }
  }

  // Sensor tail isolé (9B, terminé FF FF, sans flag en tête)
  if (len == SENSOR_TAIL_LEN && burst[len - 1] == 0xFF && burst[len - 2] == 0xFF && burst[len - 4] == 0xFF)
  {
    if (!decodeSensorTail(burst))
      debugV("SensorIso INVALID: %s", hexDumpFast(burst, len));
    return;
  }

  // Trame non reconnue — rate-limited pour ne pas saturer
  unknown_count_since_log++;
  unsigned long now = millis();
  if (now - last_unknown_log_ms > 2000)
  {
    if (unknown_count_since_log > 1) {
      debugD("Unknown    : %d trames non reconnues en 2s (dernière %dB: %s)",
             unknown_count_since_log, len, hexDumpFast(burst, min(len, 16)));
    } else {
      debugV("Unknown    (%2dB): %s", len, hexDumpFast(burst, len));
    }
    last_unknown_log_ms = now;
    unknown_count_since_log = 0;
  }
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

  // Force republish toutes les 60s même si les valeurs n'ont pas changé
  bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
  if (force) last_full_publish_ms = millis();

  if (force || water_in != last_water_in)    { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP,    water_in);    last_water_in = water_in; }
  if (force || water_out != last_water_out)  { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP,   water_out);   last_water_out = water_out; }
  if (force || air_ambient != last_air)      { pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient); last_air = air_ambient; }
  if (force || coil_temp != last_coil)       { pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP,        coil_temp);   last_coil = coil_temp; }
  if (force || gas_temp != last_gas)         { pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP,         gas_temp);    last_gas = gas_temp; }
  if (force || power_on != last_power)       { pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS,    power_on);    last_power = power_on; }
  if (force || heating != last_heating)      { pushMQTTValue(MQTT_TOPIC_VALUES_HEATING,          heating);     last_heating = heating; }

  // Clear error seulement si power=1 ET aucun 0x40 depuis 10 secondes
  if (power_on == 1 && last_error != "none" &&
      (millis() - last_error_seen_ms) > ERROR_CLEAR_DELAY_MS)
  {
    pushMQTTMessage(MQTT_TOPIC_VALUES_ERROR, "none");
    last_error = "none";
    debugD("Error PAC: cleared (no 0x40 for 10s + power ON)");
  }
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

  debugI("TX PROBE: %s", hexDumpFast(hb, 12));

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
  mqtt_last_attempt = millis();

  if (!pubsubClient.connected())
  {
    Debug.printf("Connecting to MQTT Broker as %s.....\n", HOST_NAME);
    if (pubsubClient.connect(HOST_NAME, HOST_NAME, HOST_NAME))
    {
      Debug.println("Connected to MQTT broker.");
      pubsubClient.publish(MQTT_TOPIC_STATUS, "ON", true);
      pubsubClient.subscribe(MQTT_TOPIC_COMMAND_PROBE);
    }
    else
    {
      Debug.print("Failed to connect to MQTT broker, rc=");
      Debug.print(pubsubClient.state());
      Debug.println(" — retrying in 5s");
    }
  }
}

void pushMQTTMessage(const char *topic, const char *msg)
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

void pushMQTTValue(const char *topic, int value)
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

  if(strcmp(topic, MQTT_TOPIC_COMMAND_PROBE) == 0)
  {
    debugI("PROBE command received — will send test HB on next loop");
    probe_pending = true;
  }
}