#include <Arduino.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <SPI.h>
#include <LoRa.h>

#include "types.h"

// Optional: enable Reticulum integration by setting USE_RETICULUM to 1.
// When enabled the build requires an ESP32/Arduino-compatible Reticulum port
// installed and the code will attempt to register a LoRa interface with it.
#ifndef USE_RETICULUM
#define USE_RETICULUM 0
#endif

#if USE_RETICULUM
#include <Reticulum.h> // Requires a Reticulum ESP32/Arduino port
#endif

// Platform-specific includes and lightweight stubs for non-ESP32 targets
#if defined(ESP32)
  #include "esp_sleep.h"
  #include <Preferences.h>
  #include "esp_wifi.h" // For disabling Wi-Fi
  #include <WiFi.h>
  #include <esp_bt.h>   // For disabling Bluetooth
  #include <driver/adc.h> // For battery voltage
  #include <esp_task_wdt.h> // For Watchdog Timer
#else
  // Non-ESP32 platforms (nRF52840 / RAK4631): provide simple fallbacks so the
  // sketch can compile. These are intentionally conservative stubs; adapt
  // them further for production on those platforms if you need full feature parity.
  // Preferences stub (in-memory, non-persistent)
  class Preferences {
  public:
    void begin(const char*, bool) {}
    void end() {}
    unsigned int getUInt(const char*, unsigned int def) { return def; }
    unsigned long long getULong64(const char*, unsigned long long def) { return def; }
    void getString(const char*, char* out, size_t len) { if(len>0) out[0]='\0'; }
    bool getBool(const char*, bool def) { return def; }
    void putUInt(const char*, unsigned int) {}
    void putULong64(const char*, unsigned long long) {}
    void putString(const char*, const char*) {}
    void putBool(const char*, bool) {}
  };

  // No-op sleep / WDT functions
  static inline void esp_sleep_enable_timer_wakeup(unsigned long long) {}
  static inline void esp_deep_sleep_start() { while(true) { delay(1000); } }
  static inline void esp_task_wdt_init(int, bool) {}
  static inline void esp_task_wdt_add(void*) {}
  static inline void esp_task_wdt_reset() {}
#endif

// Firmware Version
#define FIRMWARE_VERSION_MAJOR 1
#define FIRMWARE_VERSION_MINOR 1

// Default pin definitions. Allow overriding via build flags or board-specific
// defines. Add `-DBOARD_HELTEC`, `-DBOARD_RAK4631`, or `-DBOARD_NRF52840` to
// your build to pick a known profile; otherwise Heltec defaults are used.

// Heltec (ESP32) defaults
// Treat Arduino nRF52 core as the RAK/nRF profile so builds don't require
// extra -D flags when compiling for those cores.
#if defined(BOARD_RAK4631) || defined(BOARD_NRF52840) || defined(ARDUINO_ARCH_NRF52)
  // Generic defaults for nRF/RAK boards; users should override these in their
  // board config if the board's LoRa module uses different pins.
  #define LORA_SCK 13
  #define LORA_MISO 12
  #define LORA_MOSI 11
  #define LORA_CS 10
  #define LORA_RST 9
  #define LORA_IRQ 2

  #define STATUS_LED LED_BUILTIN
  // Many nRF-based boards use a simple analogRead() pin number for battery
  // measurement; set a reasonable default but encourage users to verify.
  #define BATT_ADC_PIN A0
  #define BATT_VOLTAGE_MULTIPLIER 2.0f
  // No Vext control by default on these boards
#else
  // Heltec Wireless Tracker V1.1 (ESP32) defaults
  #define LORA_SCK 5
  #define LORA_MISO 19
  #define LORA_MOSI 27
  #define LORA_CS 18
  #define LORA_RST 14
  #define LORA_IRQ 26 // DIO0

  // GPS & Status LED
  #define STATUS_LED 25 // Heltec Wireless Tracker V1.1 blue LED
  #define VEXT_CTRL 21  // GPIO to control power for external peripherals (like GPS)
                        // Verify if your GPS is on this rail for your board version!
                        // Heltec Vext is often LOW to enable power, HIGH to disable.

  // Battery ADC Pin (Heltec Wireless Tracker typically uses ADC1_CH5 (GPIO13) with a voltage divider)
  #define BATT_ADC_PIN ADC1_CHANNEL_5 // GPIO13. Check your board's schematic.
  #define BATT_VOLTAGE_MULTIPLIER 2.0f // Adjust based on your voltage divider and calibration
#endif
// Voltage divider parameters - calibrate these for your board.
// Vout = Vbat * (R2 / (R1 + R2)) => Vbat = Vout * ((R1+R2)/R2)
// Example: R1=100k, R2=100k => Multiplier = 2.0
// ADC reading to Voltage: (ADC_VALUE / 4095.0) * ADC_REFERENCE_VOLTAGE * MULTIPLIER
#define ADC_REFERENCE_VOLTAGE 3.3f   // ADC reference voltage (can be calibrated if needed)


#define uS_TO_S_FACTOR 1000000ULL
#define SERIAL_BAUD 115200
#define GPS_BAUD 9600
#define CONFIG_TIMEOUT_MS 10000
#define GPS_FIX_TIMEOUT_MS 120000 // 2 minutes for GPS fix
#define WDT_TIMEOUT_S 180 // Watchdog timeout in seconds (longer than GPS + LoRa send)


Preferences preferences;

// Default Configuration Values
uint32_t sleepTimeSeconds = 300;
uint64_t loraFrequency = 915000000ULL;
char reticulumDestName[64] = "asset_tracker";
bool sendLKLonFail = true;

// GPS serial: ESP32 uses `HardwareSerial(1)` (UART1). On non-ESP cores
// fall back to the primary `Serial` instance to ensure compilation even
// when `Serial1` is not provided by the core/variant.
#if defined(ESP32)
HardwareSerial gpsSerial(1); // UART1 for GPS
#else
// Some nRF variants expose Serial1; to keep builds broadly compatible we
// default to Serial (USB) here. Users can change to Serial1 in a board
// specific build flag or by editing this line if their variant provides it.
#define gpsSerial Serial
#endif
TinyGPSPlus gps;

// Forward declaration for AssetData (definition follows below)
struct AssetData;
bool hasValidLKL = false;

#if USE_RETICULUM
Reticulum reticulum; // Global Reticulum object or get instance from library
Reticulum::Identity *identity = nullptr;
Reticulum::Destination *destination = nullptr;
#endif

struct AssetData {
  double latitude;
  double longitude;
  float altitude;
  uint32_t gps_time;
  uint32_t gps_date;
  uint8_t satellites;
  float hdop;
  float battery_voltage;
  uint8_t fix_status; // Use AkitaFixStatus_t enum values (AKITA_FIX_*)
  uint8_t firmware_version_major;
  uint8_t firmware_version_minor;
};

// Store LKL after the full struct definition
AssetData lastKnownLocation;

// LED State Machine
AkitaLedIndicatorState_t currentLedState = AKITA_LED_OFF;
unsigned long lastBlinkTime = 0;
bool ledPhysicalState = LOW;

// Function Prototypes
void controlPeripherals(bool enable);
float readBatteryVoltage();
void configureFromNVS();
void saveConfiguration();
void handleSerialConfiguration();
bool initializeLoRa();
bool initializeReticulum();
void performTrackingAction();
void goToDeepSleep();
void setupLed();
void setLed(AkitaLedIndicatorState_t state);
void updateLed();

// Rename entry points to avoid Arduino prototype generation conflicts.
void app_setup();
void app_loop();


void controlPeripherals(bool enable) {
  #ifdef VEXT_CTRL
    pinMode(VEXT_CTRL, OUTPUT);
    // Heltec Vext is often LOW to enable power, HIGH to disable. Verify for your board!
    digitalWrite(VEXT_CTRL, enable ? LOW : HIGH);
    Serial.print("Vext Peripherals Power (GPIO21 set to "); Serial.print(enable ? "LOW" : "HIGH"); Serial.println(": " + String(enable ? "ON" : "OFF"));
    if(enable) delay(200); // Give power rail time to stabilize if turning on (especially for GPS)
  #else
    Serial.println("VEXT_CTRL not defined, peripheral power not software controlled.");
  #endif
}

float readBatteryVoltage() {
    #ifdef BATT_ADC_PIN
      #if defined(ESP32)
        // Configure ADC - ESP32 ADC can be tricky, this is a basic implementation
        adc1_config_width(ADC_WIDTH_BIT_12); // 0-4095
        adc1_config_channel_atten(BATT_ADC_PIN, ADC_ATTEN_DB_11);

        // For better accuracy, take multiple readings and average
        int num_readings = 10;
        uint32_t adc_raw_sum = 0;
        for (int i = 0; i < num_readings; i++) {
            adc_raw_sum += adc1_get_raw(BATT_ADC_PIN);
            delay(1); // Small delay between readings
        }
        int adc_raw = adc_raw_sum / num_readings;
        float voltage_at_adc = (float)adc_raw / 4095.0f * ADC_REFERENCE_VOLTAGE;
        float battery_volts = voltage_at_adc * BATT_VOLTAGE_MULTIPLIER;
        Serial.print("Avg Raw ADC: "); Serial.print(adc_raw);
        Serial.print(", ADC Voltage: "); Serial.print(voltage_at_adc, 3);
        Serial.print("V, Est. Battery Voltage: "); Serial.print(battery_volts, 2); Serial.println("V");
        return battery_volts;
      #else
        // Generic analogRead-based measurement for non-ESP platforms
        // analogRead resolution varies by core; read once or average a few samples
        int num_readings = 5;
        long sum = 0;
        for (int i=0;i<num_readings;i++) { sum += analogRead(BATT_ADC_PIN); delay(2); }
        float raw = (float)sum / (float)num_readings;
        // Assume 10-bit ADC (0-1023) as a conservative default; users can adjust if needed
        float adc_max = 1023.0f;
        float voltage_at_adc = (raw / adc_max) * ADC_REFERENCE_VOLTAGE;
        float battery_volts = voltage_at_adc * BATT_VOLTAGE_MULTIPLIER;
        Serial.print("Avg Raw ADC: "); Serial.print((int)raw);
        Serial.print(", ADC Voltage: "); Serial.print(voltage_at_adc, 3);
        Serial.print("V, Est. Battery Voltage: "); Serial.print(battery_volts, 2); Serial.println("V");
        return battery_volts;
      #endif
    #else
      return 0.0f; // No battery pin defined
    #endif
}


void configureFromNVS() {
  preferences.begin("tracker_cfg", true); // Read-only mode
  sleepTimeSeconds = preferences.getUInt("sleep", sleepTimeSeconds);
  loraFrequency = preferences.getULong64("loraFreq", loraFrequency);
  preferences.getString("destName", reticulumDestName, sizeof(reticulumDestName));
  sendLKLonFail = preferences.getBool("sendLKL", sendLKLonFail);
  preferences.end();

  Serial.println("Configuration loaded from NVS:");
  Serial.print("  Sleep Interval: "); Serial.print(sleepTimeSeconds); Serial.println(" s");
  Serial.print("  LoRa Frequency: "); Serial.print((unsigned long)loraFrequency); Serial.println(" Hz");
  Serial.print("  Reticulum Dest: "); Serial.println(reticulumDestName);
  Serial.print("  Send LKL on Fail: "); Serial.println(sendLKLonFail ? "Yes" : "No");
}

void saveConfiguration() {
  preferences.begin("tracker_cfg", false); // Read-write mode
  preferences.putUInt("sleep", sleepTimeSeconds);
  preferences.putULong64("loraFreq", loraFrequency);
  preferences.putString("destName", reticulumDestName);
  preferences.putBool("sendLKL", sendLKLonFail);
  preferences.end();
  Serial.println("Configuration saved to NVS.");
}

void handleSerialConfiguration() {
  setLed(AKITA_LED_BLINK_SLOW);
  Serial.println("\n--- Configuration Mode ---");
  Serial.println("Commands: sleep <sec>, loraFreq <hz>, dest <name>, sendLKL <0|1>, show, save, reboot");
  String cmd;
  while (true) {
    updateLed();
    if (Serial.available() > 0) {
      cmd = Serial.readStringUntil('\n');
      cmd.trim();
      Serial.print("CMD> "); Serial.println(cmd);

      if (cmd.startsWith("sleep ")) {
        uint32_t val = cmd.substring(6).toInt();
        if (val >= 10) { // Minimum sleep of 10s
          sleepTimeSeconds = val;
          Serial.print("Sleep interval set to: "); Serial.println(sleepTimeSeconds);
        } else {
          Serial.println("Error: Sleep interval must be >= 10 seconds.");
        }
      } else if (cmd.startsWith("loraFreq ")) {
        unsigned long long freq_val_ll; // Use long long for sscanf
        if (sscanf(cmd.substring(9).c_str(), "%llu", &freq_val_ll) == 1) {
            uint64_t freq_val = freq_val_ll;
            // Basic sanity check for common LoRa bands (e.g., 150MHz to 960MHz)
            if (freq_val > 100000000ULL && freq_val < 1000000000ULL) {
                loraFrequency = freq_val;
                Serial.print("LoRa frequency set to: "); Serial.println((unsigned long)loraFrequency);
            } else {
                Serial.println("Error: Frequency out of typical LoRa range (e.g., 150MHz-960MHz).");
            }
        } else {
            Serial.println("Error: Invalid frequency format.");
        }
      } else if (cmd.startsWith("dest ")) {
        String name = cmd.substring(5);
        if (name.length() > 0 && name.length() < sizeof(reticulumDestName)) {
          strncpy(reticulumDestName, name.c_str(), sizeof(reticulumDestName) - 1);
          reticulumDestName[sizeof(reticulumDestName) - 1] = '\0'; // Ensure null termination
          Serial.print("Reticulum destination name set to: "); Serial.println(reticulumDestName);
        } else {
          Serial.println("Error: Invalid destination name (too long or empty).");
        }
      } else if (cmd.startsWith("sendLKL ")) {
        int val = cmd.substring(8).toInt();
        sendLKLonFail = (val == 1);
        Serial.print("Send LKL on Fail set to: "); Serial.println(sendLKLonFail ? "Yes" : "No");
      } else if (cmd.equalsIgnoreCase("show")) {
        Serial.println("\nCurrent settings (unsaved changes shown):");
        Serial.print("  Sleep Interval: "); Serial.print(sleepTimeSeconds); Serial.println(" s");
        Serial.print("  LoRa Frequency: "); Serial.print((unsigned long)loraFrequency); Serial.println(" Hz");
        Serial.print("  Reticulum Dest: "); Serial.println(reticulumDestName);
        Serial.print("  Send LKL on Fail: "); Serial.println(sendLKLonFail ? "Yes" : "No");
        Serial.print("  Firmware Version: "); Serial.print(FIRMWARE_VERSION_MAJOR); Serial.print("."); Serial.println(FIRMWARE_VERSION_MINOR);
        float current_batt = readBatteryVoltage(); // Read current voltage for display
        Serial.print("  Current Battery: "); Serial.print(current_batt, 2); Serial.println("V (approx, ensure Vext is ON if GPS shares power)");

      } else if (cmd.equalsIgnoreCase("save")) {
        saveConfiguration();
      } else if (cmd.equalsIgnoreCase("reboot")) {
        Serial.println("Rebooting...");
          setLed(AKITA_LED_OFF);
          #if defined(ESP32)
          ESP.restart();
          #elif defined(ARDUINO_ARCH_NRF52)
          NVIC_SystemReset();
          #else
          while(true) { delay(1000); }
          #endif
      } else {
        Serial.println("Unknown command.");
      }
    }
    esp_task_wdt_reset(); // Pet the dog during config loop
    delay(10); // Yield for other tasks, reduce busy wait
  }
}

bool initializeLoRa() {
  Serial.println("Initializing LoRa...");
  setLed(AKITA_LED_ON);
#if defined(ESP32)
  SPI.begin(LORA_SCK, LORA_MISO, LORA_MOSI, LORA_CS);
#else
  SPI.begin();
#endif
  LoRa.setPins(LORA_CS, LORA_RST, LORA_IRQ);

  if (!LoRa.begin(loraFrequency)) {
    Serial.println("LoRa init failed. Check connections and frequency.");
    setLed(AKITA_LED_BLINK_ERROR_LORA);
    return false;
  }
  // Optional: Set LoRa parameters if defaults are not desired
  // LoRa.setSpreadingFactor(7);
  // LoRa.setSignalBandwidth(125E3);
  // LoRa.setCodingRate4(5); // (5 -> 4/5)
  // LoRa.setTxPower(17); // dBm, ensure legal limits & sufficient power supply
  // LoRa.enableCrc();
  Serial.print("LoRa initialized at "); Serial.print(loraFrequency / 1E6); Serial.println(" MHz.");
  setLed(AKITA_LED_OFF);
  return true;
}

// Reticulum integration support. If USE_RETICULUM is 1 the block below
// will attempt to initialize Reticulum, load/create identity and destination.
// If disabled (default) a lightweight stub is provided so the rest of the
// firmware can run using direct LoRa.
#if USE_RETICULUM
bool initializeReticulum() {
  Serial.println("Initializing Reticulum...");
  setLed(AKITA_LED_ON);

  // NOTE: This section is highly dependent on the Reticulum ESP32/Arduino
  // port you install. The code below is a best-effort template and may
  // require adapting to the library you add to your Arduino libraries.

  // Example of what many ports expect (pseudo-code):
  //  - create/load identity
  //  - register a LoRa interface (if required by port)
  //  - call reticulum.init() or equivalent

  // Create or load identity (API may vary)
  identity = new Reticulum::Identity(true);
  if (identity == nullptr || !identity->isValid()) {
    Serial.println("Error: Could not create or load Reticulum Identity.");
    if(identity) { delete identity; identity = nullptr; }
    setLed(AKITA_LED_BLINK_ERROR_GPS);
    return false;
  }

  // Create a destination for announcements (API may vary)
  destination = new Reticulum::Destination(
    *identity,
    Reticulum::Destination::ANNOUNCE,
    reticulumDestName,
    "location",
    "asset_data"
  );

  Serial.println("Reticulum initialized (template). Verify API compatibility.");
  setLed(AKITA_LED_OFF);
  return true;
}
#else
// Stub when Reticulum is not enabled; return true so caller proceeds normally.
bool initializeReticulum() {
  // No-op when Reticulum disabled; direct LoRa path is used instead.
  return true;
}
#endif


void performTrackingAction() {
  esp_task_wdt_reset(); // Pet WDT at start of action

  controlPeripherals(true); // Power ON GPS and other Vext-controlled peripherals
  // Note: readBatteryVoltage() is called after Vext ON, assuming Vext doesn't affect MCU's VDD significantly
  // or that the battery measurement circuit is independent or also Vext controlled.

  float current_battery_voltage = readBatteryVoltage();
  esp_task_wdt_reset();

  // 1. Initialize LoRa
  if (!initializeLoRa()) {
    setLed(AKITA_LED_BLINK_ERROR_LORA);
    Serial.println("Skipping tracking due to LoRa failure.");
    controlPeripherals(false); // Power OFF Vext peripherals
    return; // Don't proceed to sleep setup; WDT will eventually reboot if stuck.
  }
  esp_task_wdt_reset();

  // 2. Optionally initialize Reticulum (if compiled with USE_RETICULUM=1)
#if USE_RETICULUM
  if (!initializeReticulum()) {
    setLed(AKITA_LED_BLINK_ERROR_GPS);
    Serial.println("Skipping tracking due to Reticulum failure.");
    LoRa.sleep(); // Sleep LoRa module if RNS failed but LoRa init was ok
    controlPeripherals(false);
    return;
  }
#endif
  // esp_task_wdt_reset();

  // 3. Acquire GPS Data
  Serial.println("Acquiring GPS fix (timeout: " + String(GPS_FIX_TIMEOUT_MS / 1000) + "s)...");
  // GPS is assumed to be powered on by controlPeripherals(true)
  gpsSerial.begin(GPS_BAUD); // Open GPS serial port (UART1)
  setLed(AKITA_LED_BLINK_GPS);
  unsigned long gpsStartTime = millis();
    bool newGpsFixObtained = false;
  AssetData currentReading; // Temp struct for new reading

  while (millis() - gpsStartTime < GPS_FIX_TIMEOUT_MS) {
    updateLed(); // Keep LED blinking
    esp_task_wdt_reset(); // Pet WDT during potentially long GPS loop

    while (gpsSerial.available() > 0) {
      if (gps.encode(gpsSerial.read())) { // Process one character
        // Check for a valid fix criteria once a sentence is complete
        if (gps.location.isUpdated() && gps.location.isValid() &&
            gps.satellites.isUpdated() && gps.satellites.isValid() && gps.satellites.value() >= 3 && // Min 3 sats for 2D, 4 for 3D
            gps.hdop.isUpdated() && gps.hdop.isValid() && gps.hdop.value() < 500 && gps.hdop.value() > 0 ) { // HDOP < 5.0 reasonable, TinyGPS stores it * 100

          newGpsFixObtained = true;
          currentReading.latitude = gps.location.lat();
          currentReading.longitude = gps.location.lng();
          currentReading.altitude = gps.altitude.isValid() ? gps.altitude.meters() : 0.0f;
          currentReading.gps_time = gps.time.isValid() ? gps.time.value() : 0; // HHMMSSCC
          currentReading.gps_date = gps.date.isValid() ? gps.date.value() : 0; // DDMMYY
          currentReading.satellites = gps.satellites.value();
          currentReading.hdop = (float)gps.hdop.value() / 100.0f; // Convert from centi-HDOP
          currentReading.fix_status = (uint8_t)AKITA_FIX_NEW_FIX;

          // Update Last Known Location (LKL)
          lastKnownLocation = currentReading; // Copy the new valid reading
          hasValidLKL = true;
          break; // Exit inner while (serial available)
        }
      }
    }
    if (newGpsFixObtained) break; // Exit outer while (timeout loop)
    delay(5); // Small delay to yield and prevent busy loop if no serial data
  }
  gpsSerial.end(); // Close GPS serial port

  // Power off Vext peripherals (like GPS) after attempting fix, regardless of success, to save power.
  // If other Vext peripherals need to stay on for LoRa/RNS, this needs adjustment.
  // Assuming LoRa/RNS don't depend on Vext.
  controlPeripherals(false);
  esp_task_wdt_reset();

  // 4. Prepare Data Payload
  AssetData dataToSend;
  bool shouldSendData = false;

  if (newGpsFixObtained) {
    dataToSend = currentReading; // Use the fresh GPS fix
    shouldSendData = true;
    Serial.println("New GPS Fix Obtained.");
  } else {
    Serial.println("Failed to obtain new GPS fix within timeout.");
    setLed(AKITA_LED_BLINK_ERROR_GPS);
    if (sendLKLonFail && hasValidLKL) {
      dataToSend = lastKnownLocation; // Use the stored LKL
      dataToSend.fix_status = (uint8_t)AKITA_FIX_LAST_KNOWN_FIX; // Mark it as LKL
      shouldSendData = true;
      Serial.println("Using Last Known Location for transmission.");
    } else {
      // Option: Send a "no fix" message with minimal data (e.g., just battery and FW version)
      // For now, if no new fix and LKL not allowed/available, we don't send location data.
      // However, we might still want to send a "heartbeat" or error packet.
      // This example will only send if newGpsFixObtained or (sendLKLonFail && hasValidLKL)
      Serial.println("No valid location data to send.");
      // If you want to send a "no fix" packet:
      // dataToSend.fix_status = (uint8_t)AKITA_FIX_NO_FIX;
      // shouldSendData = true; // Then fill other essential fields like battery
    }
  }

  if (shouldSendData) {
    // Populate non-GPS fields for the packet
    dataToSend.battery_voltage = current_battery_voltage;
    dataToSend.firmware_version_major = FIRMWARE_VERSION_MAJOR;
    dataToSend.firmware_version_minor = FIRMWARE_VERSION_MINOR;
    // Note: If it's a NO_FIX packet, GPS fields in dataToSend might be uninitialized or from LKL.
    // Ensure receiver handles this based on fix_status.
    // If it was LKL, GPS fields are already set. If NO_FIX and you send, zero them out or use specific values.
    if (dataToSend.fix_status == (uint8_t)AKITA_FIX_NO_FIX) {
        dataToSend.latitude = 0.0; dataToSend.longitude = 0.0; dataToSend.altitude = 0.0;
        dataToSend.gps_time = 0; dataToSend.gps_date = 0; dataToSend.satellites = 0; dataToSend.hdop = 99.99f;
    }


    Serial.println("--- Payload Data ---");
    Serial.print("  Lat: "); Serial.println(dataToSend.latitude, 6);
    Serial.print("  Lon: "); Serial.println(dataToSend.longitude, 6);
    Serial.print("  Alt: "); Serial.print(dataToSend.altitude); Serial.println(" m");
    Serial.print("  Sats: "); Serial.println(dataToSend.satellites);
    Serial.print("  HDOP: "); Serial.println(dataToSend.hdop, 2);
    Serial.print("  Batt: "); Serial.print(dataToSend.battery_voltage, 2); Serial.println(" V");
    Serial.print("  FixStatus: "); Serial.println(dataToSend.fix_status);
    Serial.print("  FW Ver: "); Serial.print(dataToSend.firmware_version_major); Serial.print("."); Serial.println(dataToSend.firmware_version_minor);

    setLed(AKITA_LED_ON); // Solid LED during LoRa transmission
    // Send the data via LoRa
    LoRa.beginPacket();
    LoRa.write((uint8_t*)&dataToSend, sizeof(AssetData));
    LoRa.endPacket();

    Serial.println("Data sent successfully via LoRa.");
    setLed(AKITA_LED_BLINK_SUCCESS);
  }

  // 5. Cleanup Reticulum objects (if dynamically allocated and need explicit release before sleep)
  // Depending on the Reticulum library, these might be managed internally or require deletion.
  // Deleting and re-creating them on each wake cycle (as done by calling initializeReticulum again)
  // is a safe approach if their state doesn't persist well across deep sleep.
  // if (destination) { delete destination; destination = nullptr; }
  // if (identity) { delete identity; identity = nullptr; }
  // The global 'reticulum' object's state across deep sleep is library-dependent.

  // 6. Sleep LoRa module
  Serial.println("Putting LoRa module to sleep.");
  LoRa.sleep(); // Use LoRa.idle() if LoRa.sleep() is not fully supported or too slow to wake.
                // LoRa.end() is more aggressive, might require re-init of SPI on wake. LoRa.sleep() is usually sufficient.
  setLed(AKITA_LED_OFF); // Ensure LED is off before sleep
}

void goToDeepSleep() {
  Serial.print("Disabling Wi-Fi and Bluetooth (if active). Entering deep sleep for ");
  Serial.print(sleepTimeSeconds);
  Serial.println(" seconds.");
  Serial.println("----------------------------------------");
  Serial.flush(); // Ensure all serial messages are sent

  // Platform-specific power-down: on ESP32 try to disable WiFi/BT first.
#if defined(ESP32)
  // Disable Wi-Fi
  if (WiFi.getMode() != WIFI_OFF) {
    WiFi.disconnect(true); // true to erase WiFi config from NVS
    WiFi.mode(WIFI_OFF);
    Serial.println("Wi-Fi turned OFF.");
  }

  // Disable Bluetooth (ESP32 has Classic BT and BLE)
  if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
    esp_bt_controller_disable();
    Serial.println("Bluetooth Controller Disabled.");
  }

  esp_sleep_enable_timer_wakeup(sleepTimeSeconds * uS_TO_S_FACTOR);
  esp_deep_sleep_start();
#else
  // Non-ESP platforms: no standard WiFi/BT APIs here. Perform a simple delay
  // to emulate sleep and halt (user may replace with an MCU-specific low-power call).
  Serial.println("Non-ESP platform: entering simulated sleep (blocking delay)");
  delay(sleepTimeSeconds * 1000UL);
  Serial.println("Waking from simulated sleep (restarting sketch)");
  // Optionally restart; call MCU reset if available
#if defined(NVIC_SystemReset)
  NVIC_SystemReset();
#else
  while(true) { delay(1000); }
#endif
#endif
  // Execution stops here until wake-up (reset)
}

void app_setup() {
  Serial.begin(SERIAL_BAUD);
  setupLed(); // Initialize LED pin
  setLed(AKITA_LED_ON); // Solid LED during initial setup phase
  Serial.println("\n\n--- Akita Heltec Reticulum Tracker ---");
  Serial.print("Firmware Version: "); Serial.print(FIRMWARE_VERSION_MAJOR); Serial.print("."); Serial.println(FIRMWARE_VERSION_MINOR);
  Serial.println("Booting up...");

  controlPeripherals(false); // Ensure Vext peripherals (like GPS) are initially off

  // Initialize Watchdog Timer
  Serial.println("Initializing Watchdog Timer (" + String(WDT_TIMEOUT_S) + "s)...");
  // Newer ESP32 cores expect esp_task_wdt_init(const esp_task_wdt_config_t*).
  // Only call WDT init/add on ESP32 targets; non-ESP builds have no-op stubs.
#if defined(ESP32)
  esp_task_wdt_init(NULL);
  esp_task_wdt_add(NULL); // Register current task (setup/loop) with WDT
  esp_task_wdt_reset();   // Pet the dog immediately
#endif

  // Check for configuration command via Serial
  Serial.println("Enter 'config' within " + String(CONFIG_TIMEOUT_MS / 1000) + "s for setup mode...");
  unsigned long configEntryStart = millis();
  String command = "";
  bool enteredConfig = false;
  // Non-blocking check for "config" command
  while (millis() - configEntryStart < CONFIG_TIMEOUT_MS) {
    // No LED update here to keep initial ON state, or use a specific "waiting for config" blink
    esp_task_wdt_reset(); // Pet WDT while waiting
    if (Serial.available() > 0) {
      command = Serial.readStringUntil('\n');
      command.trim();
      if (command.equalsIgnoreCase("config")) {
        enteredConfig = true;
        break;
      }
    }
    delay(50); // Small delay to yield CPU
  }

  if (enteredConfig) {
    // Vext might need to be ON for 'show' command to read battery accurately if GPS shares power
    // and measurement happens after GPS power up.
    // For simplicity, turn it on for config mode if battery reading is desired.
    controlPeripherals(true); // Enable Vext for config mode if needed for 'show' battery
    handleSerialConfiguration(); // This function loops until a reboot command is issued
  } else {
    Serial.println("No 'config' command received, proceeding with normal operation.");
    setLed(AKITA_LED_OFF); // Turn off initial ON LED
    configureFromNVS();    // Load saved settings
    performTrackingAction(); // Main logic: GPS, Send Data
    goToDeepSleep();         // Enter deep sleep
  }
}

void app_loop() {
  // This loop should ideally not be reached if deep sleep starts correctly in setup().
  // It acts as a fallback or an error state indicator.
  Serial.println("CRITICAL ERROR: Reached empty loop(). System should have deep slept.");
  Serial.println("This indicates a failure in the main operational flow before deep sleep.");
  Serial.println("Rebooting via WDT or manual reset might be necessary.");
  setLed(AKITA_LED_BLINK_ERROR_GPS); // Use a generic critical error blink
  unsigned long errorLoopStart = millis();
  while(true) { // Loop indefinitely blinking error, WDT should eventually reboot
      updateLed();
      esp_task_wdt_reset(); // Keep petting WDT to allow error blink, or remove to force WDT reboot sooner
      delay(100);
      if(millis() - errorLoopStart > 30000) { // After 30s of error blinking, force a restart
            Serial.println("Forcing restart after prolonged error state in loop().");
            #if defined(ESP32)
            ESP.restart();
            #elif defined(ARDUINO_ARCH_NRF52)
            NVIC_SystemReset();
            #else
            while(true) { delay(1000); }
            #endif
      }
  }
}

// Arduino runtime entry points are provided by the minimal .ino wrapper
// to ensure `types.h` is included before Arduino's prototype generation.

// --- LED Control Functions ---
void setupLed() {
  pinMode(STATUS_LED, OUTPUT);
  digitalWrite(STATUS_LED, LOW); // Start with LED off
}

void setLed(AkitaLedIndicatorState_t state) {
  currentLedState = state;
  lastBlinkTime = millis(); // Reset blink timer on state change for consistent blinking

  switch (currentLedState) {
    case AKITA_LED_ON:
      digitalWrite(STATUS_LED, HIGH);
      ledPhysicalState = HIGH;
      break;
    case AKITA_LED_OFF:
    default:
      digitalWrite(STATUS_LED, LOW);
      ledPhysicalState = LOW;
      break;
  }
}

void updateLed() {
  // This function should be called frequently in any loops or delays
  // where blinking behavior is desired.
  unsigned long currentTime = millis();
  long intervalOn = 0;
  long intervalOff = 0; // Not used in current simple blink, but could be for complex patterns

  switch (currentLedState) {
    case AKITA_LED_BLINK_SLOW: // Config mode
      intervalOn = 1000; break;
    case AKITA_LED_BLINK_GPS: // GPS Acquisition
      intervalOn = 250; break;
    case AKITA_LED_BLINK_SUCCESS: // Message Sent Successfully (3 short blinks then OFF)
      // This state is managed by setLed turning off after a few blinks, or a more complex blinker
      // For simplicity, we'll make it blink a few times then an external call to setLed(OFF) is better.
      // Or, a counter within this state.
      // Simple version: blink fast for a short duration set by an external timer.
      // Here, let's make it a continuous fast blink until changed.
      intervalOn = 100; break; // For a continuous blink during "success phase"
    case AKITA_LED_BLINK_ERROR_LORA:
    case AKITA_LED_BLINK_ERROR_GPS:
    case AKITA_LED_BLINK_ERROR_RNS:
      intervalOn = 150; break; // Fast error blink
    case AKITA_LED_ON:
    case AKITA_LED_OFF:
    default:
      return; // Solid ON or OFF, no periodic update needed from here
  }

  if (intervalOn > 0 && (currentTime - lastBlinkTime >= intervalOn)) {
    ledPhysicalState = !ledPhysicalState; // Toggle state
    digitalWrite(STATUS_LED, ledPhysicalState);
    lastBlinkTime = currentTime;
  }
}
