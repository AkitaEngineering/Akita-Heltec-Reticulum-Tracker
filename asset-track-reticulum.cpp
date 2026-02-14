#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

#if defined(ESP32)
  #include <esp_sleep.h>
  #include <Preferences.h>
#else
  // Minimal Preferences stub for non-ESP builds (non-persistent)
  class Preferences {
  public:
    void begin(const char*, bool) {}
    void end() {}
    unsigned int getUInt(const char*, unsigned int def) { return def; }
    unsigned int getULong(const char*, unsigned int def) { return def; }
    void getString(const char*, char* out, size_t len) { if(len>0) out[0]='\0'; }
  };
#endif

// Akita Engineering Asset Tracker with LoRa

#if defined(BOARD_RAK4631) || defined(BOARD_NRF52840)
  #define LORA_SCK 13
  #define LORA_MISO 12
  #define LORA_MOSI 11
  #define LORA_CS 10
  #define LORA_RST 9
  #define LORA_IRQ 2
  #define GPS_RX 8
  #define GPS_TX 9
  #define STATUS_LED LED_BUILTIN
#else
  #define LORA_SCK 5
  #define LORA_MISO 19
  #define LORA_MOSI 27
  #define LORA_CS 18
  #define LORA_RST 14
  #define LORA_IRQ 26
  #define GPS_RX 16
  #define GPS_TX 17
  #define STATUS_LED 2
#endif
#define uS_TO_S_FACTOR 1000000ULL

Preferences preferences;
uint32_t sleepTimeSeconds = 60;
uint32_t loraFrequency = 915E6;
char reticulumDestAddr[] = "asset_tracker";

HardwareSerial gpsSerial(1);
TinyGPSPlus gps;

struct AssetData {
  double latitude;
  double longitude;
  float altitude;
  uint32_t timestamp;
  uint8_t fixQuality;
};

void configureFromNVS() {
  preferences.begin("config");
  sleepTimeSeconds = preferences.getUInt("sleep", sleepTimeSeconds);
  loraFrequency = preferences.getUInt("loraFreq", loraFrequency);
  preferences.getString("dest", reticulumDestAddr, sizeof(reticulumDestAddr));
  preferences.end();
}

void setup() {
  Serial.begin(115200);
  pinMode(STATUS_LED, OUTPUT);
  configureFromNVS();

  gpsSerial.begin(9600, SERIAL_8N1, GPS_RX, GPS_TX);

  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(loraFrequency)) {
    Serial.println("LoRa init failed. Check your connections.");
    while (true);
  }

  Serial.println("LoRa initialized.");

  // Perform tracking
  digitalWrite(STATUS_LED, HIGH);

  while (gpsSerial.available() > 0) {
    if (gps.encode(gpsSerial.read())) {
      if (gps.location.isValid()) {
        AssetData assetData;
        assetData.latitude = gps.location.lat();
        assetData.longitude = gps.location.lng();
        assetData.altitude = gps.altitude.meters();
        assetData.timestamp = gps.time.value();
        assetData.fixQuality = gps.satellites.value();

        size_t dataSize = sizeof(AssetData);
        uint8_t serializedData[dataSize];
        memcpy(serializedData, &assetData, dataSize);

        LoRa.beginPacket();
        LoRa.write(serializedData, dataSize);
        LoRa.endPacket();

        Serial.println("Data sent via LoRa.");
        Serial.print("Latitude: ");
        Serial.println(assetData.latitude, 6);
        Serial.print("Longitude: ");
        Serial.println(assetData.longitude, 6);
        Serial.print("Altitude: ");
        Serial.println(assetData.altitude);
        Serial.print("Timestamp: ");
        Serial.println(assetData.timestamp);
        Serial.print("Fix Quality: ");
        Serial.println(assetData.fixQuality);

      } else {
        Serial.println("GPS location invalid.");
      }
      break;
    }
  }

  digitalWrite(STATUS_LED, LOW);

  esp_sleep_enable_timer_wakeup(sleepTimeSeconds * uS_TO_S_FACTOR);
  Serial.print("Going to sleep for ");
  Serial.print(sleepTimeSeconds);
  Serial.println(" seconds.");
  esp_deep_sleep_start();
}

void loop() {
  // Should not reach here
}
