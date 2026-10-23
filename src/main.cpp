#include <Arduino.h>
#include <secrets.h>

#define HOST_NAME "esp8266_poolheater_controller"

// UART PROTOCOL — GPIO (Wemos D1 Mini)
// Current wiring: RO→D2, DI→D1, DE/RE→D5
#define UART_RTS 14  // GPIO14 = D5 : MAX485 DE/RE
#define UART_TX   5  // GPIO5  = D1 : MAX485 DI
#define UART_RX   4  // GPIO4  = D2 : MAX485 RO
#define PORT_SPEED 9600

// Bus protocol — frames identified by live analysis (May 2026)
//
//  Bus cycle (~190ms) — alternating pattern:
//   [PAC→remote] [flags] [state ~14B][sensors 9B] : full frame
//   [remote→PAC] XX 04 FF ... (12B) : remote control heartbeat, constant
//   [PAC→remote] FD XX ... (13B) : PAC ACK
//   ... (the cycle repeats)
//
//  Inter-frame separator: 0x01 bytes (idle bus)
//
//  PAC frame structure: [flag1: 00|80] [optional flag2: FC|06|08|88|5C] <state> <sensors 9B>
//  The LAST 9 bytes = sensors (terminated by FF FF):
//    [0] water_in_temp    = ((0xFF - x) >> 3) & 0x3F
//    [1] air_ambient_temp = ((0xFF - x) >> 1) & 0x3F
//    [2] coil_temp        = ((0xFF - x) >> 1) & 0x3F
//    [3] gas_temp         = ((0xFF - x) >> 1) & 0x3F
//    [4] water_out_temp   = ((0xFF - x) >> 1) & 0x3F
//    [5] constant 0xFF
//    [6] active_status    = ((0xFF - x) >> 4) & 0x01
//    [7-8] constant FF FF (termination)
//
//  The part BEFORE the 9B = machine state/config (setpoint, mode — to be decoded)
#define FRAME_IDLE_BYTE       0x01  // inter-frame separator on the bus
#define FRAME_SYNC_BYTE       0xFF  // sometimes captured on its own at the start of a burst
#define SENSOR_TAIL_LEN        9    // sensors = last 9 bytes of the burst (terminated by FF FF)
#define SENSOR_TEMP_MIN        5    // °C — plausible lower bound for validation
#define SENSOR_TEMP_MAX       45    // °C — plausible upper bound for validation

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

// MQTT rate limiting — only publish if the value changed OR every 60s
static int last_water_in = -1, last_water_out = -1, last_air = -1;
static int last_coil = -1, last_gas = -1, last_power = -1, last_heating = -1;
static int last_setpoint = -1, last_setpoint_confirmed = -1;
static int last_mode = -1;
static String last_error = "";
static uint8_t error_debounce_val = 0;    // last status_byte value seen
static uint8_t error_debounce_count = 0;  // number of consecutive times
static unsigned long last_full_publish_ms = 0;
const unsigned long MQTT_FULL_PUBLISH_INTERVAL_MS = 60000; // republish every 60s

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
// State/config part (long burst without the 9B sensors) — for future analysis
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
  // MAX485 in receive mode IMMEDIATELY — before anything else
  // so the RS485 bus is never corrupted, even if WiFi fails.
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

// Flag indicating the bus is idle (last processing ended on 0x01 bytes)
static bool bus_is_idle = false;

void loop()
{
  // Give the WiFi stack time when the bus is idle (between frames).
  // NEVER cut RX in the middle of a frame.
  static unsigned long last_wifi_window = 0;
  unsigned long now = millis();
  if (bus_is_idle && (now - last_wifi_window >= 50))
  {
    last_wifi_window = now;
    PS.enableRx(false);
    delay(3); // 3ms WiFi window during bus idle
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

  // If a probe is pending, send it now
  if (probe_pending)
  {
    sendProbeHB();
  }

  // sniffing UART
  sniffing();
}

// Returns a readable hex dump of a buffer (static buffer, no heap allocation).
static char hex_buf[768]; // max 250 bytes * 3 chars + null
const char* hexDumpFast(const uint8_t *buf, int len)
{
  int pos = 0;
  for (int i = 0; i < len && pos < (int)sizeof(hex_buf) - 4; i++)
  {
    pos += snprintf(hex_buf + pos, sizeof(hex_buf) - pos, "%02X ", buf[i]);
  }
  if (pos > 0) hex_buf[pos - 1] = '\0'; // removes the trailing space
  else hex_buf[0] = '\0';
  return hex_buf;
}

// Residual buffer — keeps unterminated bytes between readBytes calls.
static uint8_t residual[300];
static int residual_len = 0;
static unsigned long residual_last_activity = 0;
const unsigned long RESIDUAL_TIMEOUT_MS = 100; // flush if no new data for 100ms

// Reads the RS485 bus and splits the stream into bursts (non-0x01 sequences).
// Accumulates bytes in a residual buffer to handle split frames.
// Limits the processing rate to let the WiFi stack breathe.
void sniffing()
{
  const int READ_SIZE = 250;
  uint8_t buf[READ_SIZE];

  // Mini-throttle: read at most every 5ms to accumulate bytes
  // (~5 bytes at 9600 baud) instead of reading byte by byte
  static unsigned long last_read_ms = 0;
  unsigned long now = millis();

  int avail = PS.available();
  if (avail <= 0)
  {
    // If the residual stalls without a terminator, force processing
    if (residual_len > 0 && (now - residual_last_activity) > RESIDUAL_TIMEOUT_MS)
    {
      if (residual_len >= 3)
        processBurst(residual, residual_len);
      residual_len = 0;
    }
    bus_is_idle = (residual_len == 0);
    return;
  }

  // Only read if at least 5ms elapsed OR buffer > 20 bytes
  if (avail < 20 && (now - last_read_ms) < 5) return;

  last_read_ms = now;
  bus_is_idle = false;
  residual_last_activity = now;

  // Only read what is available — never block
  int to_read = min(avail, READ_SIZE);
  int size = PS.readBytes((char *)buf, to_read);
  if (size <= 0) return;

  // debugV("raw(%d): %s", size, hexDumpFast(buf, size));

  // Append the new data to the residual buffer
  int copy_len = min(size, (int)(sizeof(residual) - residual_len));
  memcpy(residual + residual_len, buf, copy_len);
  residual_len += copy_len;

  // Process complete bursts (terminated by at least one 0x01)
  int i = 0;
  int last_processed = 0;

  while (i < residual_len)
  {
    // Skip idle bytes
    while (i < residual_len && residual[i] == FRAME_IDLE_BYTE) i++;
    if (i >= residual_len) { last_processed = i; break; }

    int burst_start = i;
    while (i < residual_len && residual[i] != FRAME_IDLE_BYTE) i++;

    if (i >= residual_len)
    {
      // No 0x01 found after this burst → incomplete, keep it for the next read
      last_processed = burst_start;
      break;
    }

    // Complete burst (followed by a 0x01)
    int burst_len = i - burst_start;
    if (burst_len >= 3)
      processBurst(residual + burst_start, burst_len);

    last_processed = i;
    yield(); // let the WiFi stack run between bursts
  }

  // Compact the residual: keep only unprocessed bytes
  if (last_processed > 0)
  {
    residual_len -= last_processed;
    if (residual_len > 0)
      memmove(residual, residual + last_processed, residual_len);
  }

  // The bus is idle if the whole residual was processed (only 0x01 left, or empty)
  bus_is_idle = (residual_len == 0);

  // Safety: if the buffer overflows without any idle byte, drop it unprocessed
  if (residual_len >= (int)sizeof(residual) - READ_SIZE)
  {
    debugV("Residual overflow, dropping %d bytes", residual_len);
    residual_len = 0;
  }

  yield();
}

// Identifies and processes a burst according to its type.
// Rate-limits unrecognised frames to avoid saturating debug/WiFi.
static unsigned long last_unknown_log_ms = 0;
static int unknown_count_since_log = 0;

void processBurst(const uint8_t *burst, int len)
{
  // Remote control → PAC heartbeat: XX [04|0C] FF ... (12B)
  // Sometimes the first byte (counter) is lost → 11B with burst[0]=[04|0C]
  // byte[1]: 0x04=normal, 0x0C=power command in progress
  // byte[4] = requested setpoint: (0xFF - x) >> 1 = °C
  // byte[3] = PAC-confirmed setpoint: same formula
  bool is_hb_12 = (len == 12 && (burst[1] == 0x04 || burst[1] == 0x0C) && burst[2] == 0xFF);
  bool is_hb_11 = (len == 11 && (burst[0] == 0x04 || burst[0] == 0x0C) && burst[1] == 0xFF);

  if (is_hb_12 || is_hb_11)
  {
    int offset = is_hb_11 ? -1 : 0; // offset if counter is missing
    int setpoint_requested  = ((0xFF - burst[4 + offset]) >> 1) & 0x3F;
    int setpoint_confirmed  = ((0xFF - burst[3 + offset]) >> 1) & 0x3F;
    bool power_cmd = (burst[1 + offset] == 0x0C);
    debugD("Remote HB  : setpoint=%d°C (confirmed=%d°C) power_cmd=%d",
           setpoint_requested, setpoint_confirmed, power_cmd);
    // debugV("Remote HB raw: %s", hexDumpFast(burst, len));
    bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
    if (force || setpoint_requested != last_setpoint)           { pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT, setpoint_requested);           last_setpoint = setpoint_requested; }
    if (force || setpoint_confirmed != last_setpoint_confirmed) { pushMQTTValue(MQTT_TOPIC_VALUES_SETPOINT_CONFIRMED, setpoint_confirmed); last_setpoint_confirmed = setpoint_confirmed; }
    return;
  }

  // PAC → remote ACK: contains F1 xx D5 and everything else is FF
  // Variable size (8-13B) as it is sometimes truncated by read timing
  if (len >= 7 && len <= 13 && burst[1] == 0xF1 && burst[3] == 0xD5)
  {
    debugV("PAC ACK    : %s", hexDumpFast(burst, len));
    return;
  }
  // Truncated variant: starts directly with F1
  if (len >= 6 && len <= 13 && burst[0] == 0xF1 && burst[2] == 0xD5)
  {
    debugV("PAC ACK    : %s", hexDumpFast(burst, len));
    return;
  }

  // PAC data frames: start with FF, 0x00 or 0x80 (flag byte)
  // The LAST 9 bytes = sensors (if the burst ends with ... FF [status] FF FF)
  // tail[5]=FF is at burst[len-4], tail[7]=FF at burst[len-2], tail[8]=FF at burst[len-1]
  if (burst[0] == FRAME_SYNC_BYTE || burst[0] == 0x00 || burst[0] == 0x80)
  {
    bool has_sensor_tail = (len >= SENSOR_TAIL_LEN) &&
                           (burst[len - 1] == 0xFF) &&
                           (burst[len - 2] == 0xFF) &&
                           (burst[len - 4] == 0xFF);  // tail[5]=FF constant

    if (has_sensor_tail)
    {
      int sensor_start = len - SENSOR_TAIL_LEN;

      // Decode the sensors (9B at the end)
      if (!decodeSensorTail(burst + sensor_start))
        debugV("Sensor INVALID: %s", hexDumpFast(burst + sensor_start, SENSOR_TAIL_LEN));

      // The part before = flags + machine state/config
      if (sensor_start > 0)
      {
        debugV("State      (%2dB): %s", sensor_start, hexDumpFast(burst, sensor_start));
        pushMQTTMessage(MQTT_TOPIC_VALUES_STATE_RAW, hexDumpFast(burst, sensor_start));

        // Decode the PAC mode (last byte of the state part, long frames ≥15B)
        if (sensor_start >= 15)
        {
          uint8_t mode_byte = burst[sensor_start - 1];
          int mode = -1;
          switch (mode_byte) {
            case 0x6C: mode = 2; break; // Heat only
            case 0x64: mode = 2; break; // Heat (transition variant)
            case 0x54: mode = 1; break; // Auto (heat + cool)
            case 0x5C: mode = 0; break; // Cool only
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

          // Setpoint confirmed by the PAC: state byte[2] (when burst starts with 00/80)
          // Same formula as water_in: ((0xFF - x) >> 3) & 0x3F
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
      // PAC error mode (PL/no flow): short frame with sensors in reduced format
      // Pattern: [00|80] [flag] [error_code] [5 temps] FF FF  (10B)
      // Also: the last 7 bytes [5 temps] FF FF are the "sensor short tail"
      bool has_short_tail = (len >= 7) &&
                            (burst[len - 1] == 0xFF) &&
                            (burst[len - 2] == 0xFF);

      if (has_short_tail && len <= 12)
      {
        // Try decoding in short mode (5 temp bytes + FF + FF)
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
          // In error mode, status = OFF (st[5]=FF, st[6]=FF → power=0)
          int power_on = 0;
          int heating  = 0;

          debugD("Sensor(err): water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C power=%d heating=%d",
                 water_in, water_out, air_ambient, coil_temp, gas_temp, power_on, heating);

          // Publish the values
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
        // Burst without FF FF termination — machine state (error mode or config)
        debugV("PAC-state  (%2dB): %s", len, hexDumpFast(burst, len));

        // TODO: PL detection disabled — the 0x0C/0x14 values do not match
        // field observations. To be recalibrated with new captures.
        // Short 3B frame: [00|80] [flag] [status_byte]
        if (len == 3 && (burst[0] == 0x00 || burst[0] == 0x80))
        {
          debugV("PAC-status 3B: %02X %02X %02X", burst[0], burst[1], burst[2]);
        }

        if (len >= 10) // enough data to be interesting
        {
          pushMQTTMessage(MQTT_TOPIC_VALUES_STATE_RAW, hexDumpFast(burst, len));

          // Decode the PAC mode on long state frames (≥15B)
          if (len >= 15)
          {
            uint8_t mode_byte = burst[len - 1];
            int mode = -1;
            switch (mode_byte) {
              case 0x6C: mode = 2; break; // Heat only
              case 0x64: mode = 2; break; // Heat (transition variant)
              case 0x54: mode = 1; break; // Auto (heat + cool)
              case 0x5C: mode = 0; break; // Cool only
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

  // Short frame (isolated flags, 0x40 flooding in error mode)
  if (len <= 4)
  {
    debugV("Short(%dB)  : %s", len, hexDumpFast(burst, len));
    return;
  }

  // 0x40 burst — normal bus signal (not an error indicator)
  if (burst[0] == 0x40)
  {
    debugV("PAC-signal (%2dB): 0x40 burst", len);
    return;
  }

  // Isolated sensor tail (7B in error mode: 5 temps + FF FF, no header)
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
      // Publish (power=off in error mode)
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

  // Isolated sensor tail (9B in error mode: 5 temps + status + status2 + FF FF)
  // E.g. 54 E9 D1 C3 D9 FD EB FF FF — burst[5] is NOT 0xFF in error mode
  if (len == SENSOR_TAIL_LEN && burst[len - 1] == 0xFF && burst[len - 2] == 0xFF)
  {
    int water_in  = ((0xFF - burst[0]) >> 3) & 0x3F;
    int water_out = ((0xFF - burst[4]) >> 1) & 0x3F;
    if (water_in >= SENSOR_TEMP_MIN && water_in <= SENSOR_TEMP_MAX &&
        water_out >= SENSOR_TEMP_MIN && water_out <= SENSOR_TEMP_MAX)
    {
      int air_ambient = ((0xFF - burst[1]) >> 1) & 0x3F;
      int coil_temp   = ((0xFF - burst[2]) >> 1) & 0x3F;
      int gas_temp    = ((0xFF - burst[3]) >> 1) & 0x3F;
      // bytes 5-6 = status (FD=off? EB=?), in error mode power=0
      int power_on = (burst[5] == 0xFF) ? (((0xFF - burst[6]) >> 4) & 0x01) : 0;
      int heating  = 0;

      debugD("Sensor(9B) : water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C power=%d",
             water_in, water_out, air_ambient, coil_temp, gas_temp, power_on);

      bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
      if (force) last_full_publish_ms = millis();
      if (force || water_in != last_water_in)    { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP, water_in);     last_water_in = water_in; }
      if (force || water_out != last_water_out)  { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP, water_out);   last_water_out = water_out; }
      if (force || air_ambient != last_air)      { pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient); last_air = air_ambient; }
      if (force || coil_temp != last_coil)       { pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP, coil_temp);         last_coil = coil_temp; }
      if (force || gas_temp != last_gas)         { pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP, gas_temp);           last_gas = gas_temp; }
      if (force || power_on != last_power)       { pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS, power_on);      last_power = power_on; }
      if (force || heating != last_heating)      { pushMQTTValue(MQTT_TOPIC_VALUES_HEATING, heating);             last_heating = heating; }
      return;
    }
    // If validation fails, try normal decoding (burst[5]==FF)
    if (burst[len - 4] == 0xFF)
    {
      if (!decodeSensorTail(burst))
        debugV("SensorIso INVALID: %s", hexDumpFast(burst, len));
      return;
    }
  }

  // Unrecognised frame — rate-limited to avoid flooding
  unknown_count_since_log++;
  unsigned long now = millis();
  if (now - last_unknown_log_ms > 2000)
  {
    if (unknown_count_since_log > 1) {
      debugD("Unknown    : %d unrecognised frames in 2s (last %dB: %s)",
             unknown_count_since_log, len, hexDumpFast(burst, min(len, 16)));
    } else {
      debugV("Unknown    (%2dB): %s", len, hexDumpFast(burst, len));
    }
    last_unknown_log_ms = now;
    unknown_count_since_log = 0;
  }
}

// Decodes the 9 sensor bytes (end of burst, terminated by FF FF).
// Formulas confirmed with real measurements (water=17°C, air=15°C, May 2026).
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
  // tail[7-8] = FF FF termination

  if (water_in  < SENSOR_TEMP_MIN || water_in  > SENSOR_TEMP_MAX) return false;
  if (water_out < SENSOR_TEMP_MIN || water_out > SENSOR_TEMP_MAX) return false;

  debugD("Sensor     : water_in=%d°C water_out=%d°C air=%d°C coil=%d°C gas=%d°C power=%d heating=%d",
         water_in, water_out, air_ambient, coil_temp, gas_temp, power_on, heating);

  // Force republish every 60s even if values have not changed
  bool force = (millis() - last_full_publish_ms) > MQTT_FULL_PUBLISH_INTERVAL_MS;
  if (force) last_full_publish_ms = millis();

  if (force || water_in != last_water_in)    { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_IN_TEMP,    water_in);    last_water_in = water_in; }
  if (force || water_out != last_water_out)  { pushMQTTValue(MQTT_TOPIC_VALUES_WATER_OUT_TEMP,   water_out);   last_water_out = water_out; }
  if (force || air_ambient != last_air)      { pushMQTTValue(MQTT_TOPIC_VALUES_AIR_AMBIENT_TEMP, air_ambient); last_air = air_ambient; }
  if (force || coil_temp != last_coil)       { pushMQTTValue(MQTT_TOPIC_VALUES_COIL_TEMP,        coil_temp);   last_coil = coil_temp; }
  if (force || gas_temp != last_gas)         { pushMQTTValue(MQTT_TOPIC_VALUES_GAZ_TEMP,         gas_temp);    last_gas = gas_temp; }
  if (force || power_on != last_power)       { pushMQTTValue(MQTT_TOPIC_VALUES_ACTIVE_STATUS,    power_on);    last_power = power_on; }
  if (force || heating != last_heating)      { pushMQTTValue(MQTT_TOPIC_VALUES_HEATING,          heating);     last_heating = heating; }
  return true;
}

// --- TX Probe ---
// Sends a "2nd remote control" HB to test whether the PAC responds.
// Same format as the observed HB, but with a different ID (0x6_ instead of 0x5_).

void sendProbeHB()
{
  // HB template based on observation: XX 04 FF C7 C7 C9 FB FF AF DD 0B FD
  // We use 0x6_ as the ID of the "2nd remote control"
  uint8_t hb[12] = {
    (uint8_t)(0x60 | (probe_counter & 0x0F)),  // ID=0x6_ + counter
    0x04,  // keepalive normal
    0xFF,
    0xC7,  // confirmed setpoint (28°C default)
    0xC7,  // requested setpoint (28°C default)
    0xC9,
    0xFB,
    0xFF,
    0xAF,
    0xDD,
    0x0B,
    0xFD
  };

  probe_counter += 2;  // increments like the real remote control

  debugI("TX PROBE: %s", hexDumpFast(hb, 12));

  // Switch MAX485 to TX
  digitalWrite(UART_RTS, HIGH);
  delayMicroseconds(100);

  // Send the HB
  PS.write(hb, 12);
  PS.flush();

  // Wait for end of transmission: 12 bytes * 10 bits / 9600 baud ≈ 12.5ms
  delay(15);

  // Back to RX
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
      pubsubClient.publish(MQTT_TOPIC_VALUES_ERROR, "none");
      last_error = "none";
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