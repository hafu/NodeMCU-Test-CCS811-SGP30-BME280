#include <Arduino.h>

// WiFiManager
#include <DNSServer.h>
#include <ESP8266WebServer.h>
#include "WiFiManager.h"

// EEPROM
#include <ESP_EEPROM.h>

// MQTT
#include <PubSubClient.h>

// NTP / Timezones
#include <TZ.h>
#include <time.h>

// CCS811 Sensor
#include "Adafruit_CCS811.h"
// SGP30 Sensor
#include "Adafruit_SGP30.h"
// BME280
#include "Adafruit_BME280.h"


// Basic config
char baseSSID[] = "NodeMCU-Sensors";
#ifdef WIFI_PASSWD
char defaultWiFiPassword[] = WIFI_PASSWD;
#else
char defaultWiFiPassword[] = "changeme";
#endif

// Configuration
#include <config.h>


// Global variables
unsigned long t_timout_millis;  // used for messuring timeouts


// Sensors
Adafruit_CCS811 ccs;
Adafruit_SGP30 sgp;
Adafruit_BME280 bme;

// BME280 configuration
// TODO
#define SEALEVELPRESSURE_HPA (1013.25)


// MQTT Client
WiFiClient wifiClient;
PubSubClient mqttClient(wifiClient);

// struct for EEPROM data
typedef struct {
  time_t last_saved_time;
  uint16_t baseline_eco2;
  uint16_t baseline_tvoc;
} eeprom_sgp_baseline_s;

// struct for ccs data
typedef struct {
  bool      available_timeout;
  uint8_t   status;
  uint16_t  tvoc;
  uint16_t  eco2;
} ccsdata_s;

// struct for sgp30 data
typedef struct {
  bool      iaq_measure;
  bool      iaq_measure_raw;
  uint16_t  tvoc;
  uint16_t  eco2;
  uint16_t  raw_ethanol;
  uint16_t  raw_h2;
} sgpdata_s;

// struct for bme data
typedef struct {
  float temperature;
  float pressure;
  float humidity;
} bmedata_s;

// stores baseline data of sgp
eeprom_sgp_baseline_s sgp_baseline_data_rom; 

void setup() {
  Serial.begin(115200);

  // WiFiManager
  WiFiManager wifiManager;
  #ifdef RESET_WIFI_MANAGER
  Serial.println(F("Reset WiFiManager settings"));
  wifiManager.resetSettings();
  #endif
  wifiManager.setTimeout(600);
  // allocate memory for ssid (baseSSID + chipID in hex)
  char *ssid;
  ssid = (char *)malloc(sizeof(baseSSID) + 8);
  if (ssid == NULL) {
    Serial.println(F("failed to allocate memory"));
    ESP.reset();
  }
  sprintf(ssid, "%s-%X", baseSSID, ESP.getChipId());
  wifi_station_set_hostname(ssid);
  if (!wifiManager.autoConnect(ssid, defaultWiFiPassword)) {
    Serial.println(F("Failed to connect to WiFi and hit timeout"));
    ESP.reset();
  }
  Serial.println(F("Connected to WiFi"));
  free(ssid);
  // End: WiFiManager


  // Setup time
  Serial.println(F("Setup time"));
  configTime(TZ_Etc_UCT, "0.de.pool.ntp.org", "1.de.pool.ntp.org", "2.de.pool.ntp.org");
  t_timout_millis = millis();
  // should be greater than some seconds / minutes
  while (!time(nullptr) || time(nullptr) <= 300) {
    if ((millis() - t_timout_millis) > (60 * 1000)) {
      Serial.println(F("Failed to sync time in time"));
      ESP.reset();
    }
    Serial.print(F("."));
    delay(500);
  }
  Serial.println();
  // End: Setup time

  // Setup EEPROM
  Serial.println(F("Setup EEPROM"));
  EEPROM.begin(sizeof(eeprom_sgp_baseline_s));
  EEPROM.get(0, sgp_baseline_data_rom);
  Serial.print("EEPROM: Last save datetime: ");
  Serial.print(String(ctime(&sgp_baseline_data_rom.last_saved_time)));
  Serial.printf("EEPROM: Data: eCO2: 0x%x, TVOC: 0x%x\n", sgp_baseline_data_rom.baseline_eco2, sgp_baseline_data_rom.baseline_tvoc);
  // END: Setup EEPROM

  // Setup CCS811
  Serial.println(F("Setup CCS811"));
  if (!ccs.begin()) {
    Serial.println(F("Failed to start CCS811 sensor"));
    delay(1000);
    ESP.reset();
  }
  t_timout_millis = millis();
  while (!ccs.available()) {
    if ((millis() - t_timout_millis) > (30 * 1000)) {
      Serial.println(F("Sensor took to long"));
      ESP.reset();
    }
    Serial.print(F("."));
    delay(500);
  }
  // read data first time
  ccs.readData();
  Serial.println();
  // End: Setup CCS811


  // Setup SGB30
  Serial.println(F("Setup SGP30"));
  if (!sgp.begin()) {
    Serial.println(F("Failed to start SGP30 sensor"));
    delay(1000);
    ESP.reset();
  }
  // use saved baseline data
  if (!sgp.setIAQBaseline(sgp_baseline_data_rom.baseline_eco2, sgp_baseline_data_rom.baseline_tvoc)) {
    Serial.println(F("Failed to set baseline data for SGP30"));
    delay(1000);
    ESP.reset();
  }
  
  // End: Setup SGB30


  // Setup BME280
  Serial.println(F("Setup BME280"));
  if (!bme.begin(0x76)) {
    Serial.println(F("Failed to start BME280 sensor"));
    delay(1000);
    ESP.reset();
  }
  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::SAMPLING_X16,
                  Adafruit_BME280::FILTER_X8,
                  Adafruit_BME280::STANDBY_MS_1000);
  // End: Setup BME280

  // Setup MQTT client
  mqttClient.setServer(mqttServer, mqttServerPort);

  // End: Setup MQTT client

}

void loop() {
  unsigned long t_start_millis = millis();

  time_t tnow = time(nullptr);
  Serial.print(String(ctime(&tnow)));

  bmedata_s bmedata;
  bmedata.humidity = bme.readHumidity();
  bmedata.pressure = bme.readPressure();
  bmedata.temperature = bme.readTemperature();


  ccsdata_s ccsdata;
  ccsdata.available_timeout = false;
  t_timout_millis = millis();
  while (!ccs.available()) {
    if ((millis() - t_timout_millis) > (3 * 1000)) {
      Serial.println("CCS811 sensor not available in time (3s)");
      ccsdata.available_timeout = true;
    }
    delay(50);
  }
  // use values from bme to "calibrate"
  ccs.setEnvironmentalData(bmedata.humidity, bmedata.temperature);
  ccsdata.status = ccs.readData();
  if (!ccsdata.status) {
    ccsdata.eco2 = ccs.geteCO2();
    ccsdata.tvoc = ccs.getTVOC();
  } else {
    Serial.printf("Failed to read CCS811 sensor, error code: %x\n\r", ccsdata.status);
  }

  sgpdata_s sgpdata;
  if (!sgp.setHumidity(bmedata.humidity)) {
    Serial.println("Failed to set humidity for SGP30");
  }
  sgpdata.iaq_measure = sgp.IAQmeasure();
  sgpdata.iaq_measure_raw = sgp.IAQmeasureRaw();
  if (!sgpdata.iaq_measure) {
    Serial.println("Failed to read SGP30 sensor");
  } else {
    sgpdata.eco2 = sgp.eCO2;
    sgpdata.tvoc = sgp.TVOC;
  }
  if (!sgpdata.iaq_measure_raw) {
    Serial.println("Failed to read SGP30 sensor RAW data");
  } else {
    sgpdata.raw_ethanol = sgp.rawEthanol;
    sgpdata.raw_h2 = sgp.rawH2;
  }
  // save baseline values
  if (difftime(tnow, sgp_baseline_data_rom.last_saved_time) >= (3 * 24 * 60 * 60)) {
    uint16_t spg_baseline_tvoc, spg_baseline_co2;
    if (sgp.getIAQBaseline(&spg_baseline_co2, &spg_baseline_tvoc)) {
      if (spg_baseline_tvoc != 0x0 && spg_baseline_co2 != 0x0) {
        Serial.printf("SGP30: Baseline values: eCO2: 0x%x, TVOC: 0x%x\n", spg_baseline_co2, spg_baseline_tvoc);
        // only write data when values differ
        if (sgp_baseline_data_rom.baseline_eco2 != spg_baseline_co2 || sgp_baseline_data_rom.baseline_tvoc != spg_baseline_tvoc) {
          eeprom_sgp_baseline_s tmp_basline_data;
          tmp_basline_data.last_saved_time = tnow;
          tmp_basline_data.baseline_eco2 = spg_baseline_co2;
          tmp_basline_data.baseline_tvoc = spg_baseline_tvoc;
          EEPROM.put(0, tmp_basline_data);
          if (EEPROM.commit()) {
            Serial.println("Baseline data commited.");
            EEPROM.get(0, sgp_baseline_data_rom);
          } else {
            Serial.println("Failed to write baseline data, go on ...");
          }
        }
      }
    }
  }


  Serial.printf("Measurement took %lu ms\n\r", (millis() - t_start_millis));

  // test output
  if (!ccsdata.available_timeout && !ccsdata.status) {
    Serial.printf("CCS811: eCO2: %d ppm, TVOC: %d ppb\n\r", ccsdata.eco2, ccsdata.tvoc);
  }
  if (sgpdata.iaq_measure) {
    Serial.printf("SGP30:  eCO2: %d ppm, TVOC: %d ppb\n\r", sgpdata.eco2, sgpdata.tvoc);
  }
  if (sgpdata.iaq_measure_raw) {
    Serial.printf("SGP30:  raw ethanol: %d, raw h2: %d\n\r", sgpdata.raw_ethanol, sgpdata.raw_h2);
  }
  Serial.printf("BME280: %.2f °C, %.2f %%\n\r", bmedata.temperature, bmedata.humidity);

  tnow = time(nullptr);
  Serial.print(String(ctime(&tnow)));

  // test mqtt client
  if (!mqttClient.connected()) {
    t_timout_millis = millis();
    if ((millis() - t_timout_millis) > (30 * 1000)) {
      // TODO add data to buffer, no reset
      Serial.println("timeout connecting MQTT server");
      ESP.reset();
    }
    char *clientId;
    clientId = (char *)malloc(sizeof(baseSSID) + 8);
    if (clientId == NULL) {
      Serial.println(F("failed to allocate memory"));
      ESP.reset();
    }
    sprintf(clientId, "%s-%X", baseSSID, ESP.getChipId());
    while (!mqttClient.connected()) {
      Serial.print("Attempting MQTT connection...");
      if (mqttClient.connect(clientId)) {
        Serial.println("connected");
      } else {
        Serial.printf("failed to connect, rc=%d\n\r", mqttClient.state());
        delay(5000);
      }
    }
    free(clientId);
  }
  mqttClient.loop();
  // assemble payload
  String payload = "";
  if (!ccsdata.status) {
    payload += String("field1=") + String(ccsdata.eco2, DEC) + String("&field2=") + String(ccsdata.tvoc, DEC) + String("&");
  }
  if (sgpdata.iaq_measure) {
    payload += String("field3=") + String(sgpdata.eco2, DEC) + String("&field4=") + String(sgpdata.tvoc, DEC) + String("&");
  }
  if (sgpdata.iaq_measure_raw) {
    payload += String("field5=") + String(sgpdata.raw_ethanol, DEC) + String("&field6=") + String(sgpdata.raw_h2) + String("&");
  }
  payload += String("field7=") + String(bmedata.temperature, 2) + String("&field8=") + String(bmedata.humidity, 2) + String("&");
  char t_iso[30];
  struct tm *timeinfo;
  timeinfo = localtime(&tnow);
  //strftime(t_iso, 30, "%FT%T%z", timeinfo);
  // Using UTC, because fuck you
  strftime(t_iso, 30, "%FT%TZ", timeinfo);
  payload += String("created_at=") + String(t_iso);
  Serial.print("Payload: ");
  Serial.println(payload);
  Serial.print("ISO Datetime: ");
  Serial.println(t_iso);
  String topicString = "channels/" + String(channelId) + "/publish/" + String(writeAPIKey);
  char *topicBuffer;
  topicBuffer = (char *)malloc(topicString.length());
  if (topicBuffer == NULL) {
    Serial.println(F("failed to allocate memory"));
    ESP.reset();
  }
  sprintf(topicBuffer, "%s", topicString.c_str());
  char *payloadBuffer;
  payloadBuffer = (char *)malloc(payload.length());
  if (payloadBuffer == NULL) {
    Serial.println(F("failed to allocate memory"));
    ESP.reset();
  }
  sprintf(payloadBuffer, "%s", payload.c_str());
  boolean status = mqttClient.publish(topicBuffer, payloadBuffer);
  free(topicBuffer);
  free(payloadBuffer);
  Serial.print("Status: ");
  Serial.println(status);

  // wait 60 s
  delay((60 * 1000) - (t_start_millis - millis()));

/*
  // test read out ccs811
  t_timout_millis = millis();
  while (!ccs.available()) {
    if ((millis() - t_timout_millis) > (5 * 1000)) {
      Serial.println("CCS811 sensor not available in time (5s)");
      ESP.reset();
    }
    delay(50);
  }
  uint8_t ccs_status = ccs.readData();
  if (!ccs_status) {
    Serial.printf("CCS811: CO2: %d ppm, TVOC: %d ppb\n\r", ccs.geteCO2(), ccs.getTVOC());
  } else {
    Serial.printf("Failed to read CCS811 sensor, error code: %x\n\r", ccs_status);
  }
  // end: test read out ccs811


  // test read out sgp30
  if (!sgp.IAQmeasure()) {
    Serial.println("Failed to read SGP30 sensor");
    ESP.reset();
  }
  Serial.printf("SGP30: eCO2: %d ppm, TVOC: %d ppb\n\r", sgp.eCO2, sgp.TVOC);
  // NOTE: there are also RAW values
  // NOTE: there are also baseline readings
  // end: test read out sgb30
*/

/*
  // test read out bme280
  Serial.printf("BME280: %.2f °C, %.2f %%\n\r", bme.readTemperature(), bme.readHumidity());
  // NOTE: altitude in hPa available
  // end: test read out bme280
*/

  // delay(10000);

}
