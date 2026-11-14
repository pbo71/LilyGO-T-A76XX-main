/**
 * @file      HttpsBuiltlnGet.ino
 * @author    Lewis He (lewishe@outlook.com)
 * @license   MIT
 * @copyright Copyright (c) 2023  Shenzhen Xin Yuan Electronic Technology Co., Ltd
 * @date      2023-11-29
 * @note
 * * Example is suitable for A7670X/A7608X/SIM7672 series
 * * Connect https://httpbin.org test get request
 * * Example uses a forked TinyGSM <https://github.com/lewisxhe/TinyGSM>, which will not compile successfully using the mainline TinyGSM.
 */
#define TINY_GSM_RX_BUFFER 1024 // Set RX buffer to 1Kb

// See all AT commands, if wanted
// #define DUMP_AT_COMMANDS

#include "utilities.h"
#ifdef PROVISION_SECRETS
#include "secrets.h"  // Only in the T-A7670X-provision build, never in OTA firmware
#endif
#include <TinyGsmClient.h>
#include <esp32-hal-adc.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <esp_task_wdt.h>    // <-- added for watchdog support
#include <esp_arduino_version.h>
#include "esp_sleep.h"
#include <time.h>
#include <sys/time.h>
#include "lwip/apps/sntp.h"
#include <Preferences.h>
#include <Update.h>
#include <MD5Builder.h>
#include "esp_ota_ops.h"
#include <OneWire.h>
#include <DallasTemperature.h>
#include <NimBLEDevice.h>

// debug: set 0 to remove Serial prints to save power
#define DEBUG 0
#if DEBUG
  #define LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(fmt, ...)
#endif

// Firmware version: raise it for every OTA release (tools/ota_publish.py reads it from here)
#define FIRMWARE_VERSION 15

// OTA settings. The firmware holds no secrets (they live in NVS), so it can be hosted publicly.
// The folder comes from NVS ("ota_dir"), so one binary can follow a test or a stable folder.
// A board without the setting uses OTA_DEFAULT_FOLDER.
#define OTA_BASE_URL "https://raw.githubusercontent.com/pbo71/esp-firmware/main/"
#define OTA_DEFAULT_FOLDER "a7670e"
#define OTA_DRY_RUN 0                    // 1: download and verify only, never flash. Set 0 for real updates
#define OTA_CHECK_INTERVAL_SEC (24 * 3600)  // Check at most once a day (and on cold start)
#define OTA_MIN_VOLTAGE 4000             // Only update with a healthy battery (or on USB)
#define OTA_CHUNK_SIZE 32768             // Bytes per HTTP Range request
#define OTA_MAX_UNCONFIRMED_RUNS 3       // Roll back a new image that has not uploaded data after this many runs

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */

// Reporting schedule. The ESP32 wakes every SAMPLE_INTERVAL_SEC and reads the temperature without
// the modem (cheap). The modem only starts when a report is due:
//   - at least every REPORT_INTERVAL_SEC between ACTIVE_START_HOUR and ACTIVE_END_HOUR (Danish time)
//   - at once when it has risen HEATING_RISE_C since the last report (someone lit the sauna).
//     This fires once; while it keeps rising or is at or above HOT_THRESHOLD_C, reports come
//     every REPORT_INTERVAL_HOT_SEC instead of every 5 minutes
// Outside the active hours nothing is sent and the board sleeps until ACTIVE_START_HOUR.
#define SAMPLE_INTERVAL_SEC (5 * 60)
#define REPORT_INTERVAL_SEC (60 * 60)
#define REPORT_INTERVAL_HOT_SEC (30 * 60)
#define REPORT_SLACK_SEC 60              // A report due within this is sent now instead of a sample later
#define HOT_THRESHOLD_C 50.0
#define HEATING_RISE_C 5.0
#define ACTIVE_START_HOUR 6
#define ACTIVE_END_HOUR 23
#define LOCAL_TIMEZONE "CET-1CEST,M3.5.0,M10.5.0/3"  // Danish time with daylight saving

// Define constants instead of magic numbers
#define RETRY_COUNT 3
#define MAX_URL_LENGTH 256
#define VOLTAGE_MULTIPLIER 2
#define SIM_READY_TIMEOUT_MS 60000UL      // Give up waiting for the SIM after 1 minute
#define NETWORK_REG_TIMEOUT_MS 180000UL   // Give up network registration after 3 minutes
#define MODEM_INIT_RETRY 10
#define CHECK_BATTERY_THRESHOLD 3600  // 3.6V in millivolts
#define DEEP_SLEEP_THRESHOLD 3500     // 3.5V in millivolts - critical battery protection
#define MINIMUM_MODEM_VOLTAGE 3900    // 3.9V minimum for reliable modem startup
#define NO_BATTERY_VOLTAGE 1000       // Below this = no battery, running on USB only

// Low battery alert settings
// Must stay above MINIMUM_MODEM_VOLTAGE: below that the modem is never started, so no SMS can be sent
#define LOW_BATTERY_THRESHOLD 4000        // 4.0V in millivolts - send the SMS below this
#define LOW_BATTERY_RESET_THRESHOLD 4100  // 4.1V in millivolts - allow a new SMS once recovered

// Daily battery voltage drop, sent to ThingSpeak field5 (vDrop). Positive = discharged, negative = charged
#define VDROP_PERIOD_SEC (24 * 3600)
#define VDROP_MAX_AGE_SEC (36 * 3600)  // An older reference (device was off) starts a new period

// ThingSpeak fields (same layout on every board):
//   field1 firmware version, field2 outdoor temp (DS18B20), field3 indoor temp (MAX6675),
//   field4 battery mV, field5 daily vDrop mV, field6 Bluetooth sensor temp, field7 Bluetooth sensor
//   battery mV, field8 Bluetooth signal strength (RSSI). A missing or faulty sensor leaves its field out.

// Bluetooth temperature sensor (XIAO nRF52840, xiao/XiaoBleSender). It only advertises, so the board
// scans for BT_SCAN_SEC before each report and uses the advertisement whose name matches.
// The name comes from NVS ("bt_name"); a board without the setting uses BT_DEFAULT_NAME.
// The company ID 0xFFFF is the shared test ID, so the name and format version are both checked.
#define BT_DEFAULT_NAME "TestTempBTModule"
#define BT_SCAN_SEC 5
#define BT_COMPANY_ID 0xFFFF
#define BT_FORMAT_VERSION 1
#define BT_TEMP_INVALID 0x7FFF

// DS18B20 outdoor sensor on GPIO0 (needs a 4.7k pull-up to 3.3V). Optional: if none answers, field2 is left out
#define DS18B20_PIN 0

// MAX6675 thermocouple for the indoor (sauna / hut) temperature
#define MAX6675_SCK_PIN GPIO_NUM_18
#define MAX6675_CS_PIN BOARD_SD_CS_PIN  // GPIO13
#define MAX6675_SO_PIN GPIO_NUM_19
#define MAX6675_DEBUG 0  // Set 1 to print the raw MAX6675 word every cycle (wiring debug)
#define MAX6675_MAX_PLAUSIBLE_C 150.0  // Readings above this are treated as wiring noise
#define MAX6675_MAX_DIFF_C 2.0         // Two reads 250 ms apart must agree within this

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

// It depends on the operator whether to set up an APN. If some operators do not set up an APN,
// they will be rejected when registering for the network. You need to ask the local operator for the specific APN.
// APNs from other operators are welcome to submit PRs for filling.
#define NETWORK_APN "internet" // CHN-CT: China Telecom

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR bool modem_initialized = false;
RTC_DATA_ATTR bool low_battery_alert_sent = false;
RTC_DATA_ATTR time_t last_ota_check = 0;
RTC_DATA_ATTR time_t last_report_time = 0;   // Start of the last report attempt (UTC)
RTC_DATA_ATTR float last_report_temp = NAN;  // Indoor temperature sent with it
RTC_DATA_ATTR bool last_report_heating = false;  // The last report saw the temperature rising
// RTC memory survives deep sleep, also across an OTA update to a build with a different layout.
// The magic tells whether the RTC data above was written by this exact build.
#define RTC_MAGIC (0xA7670000UL | FIRMWARE_VERSION)
RTC_DATA_ATTR uint32_t rtc_magic = 0;

// Secrets and OTA state, stored in NVS (written once by the T-A7670X-provision build)
Preferences prefs;
String alertPhone;
String thingspeakKey;
String thingspeakUrl;
String otaBaseUrl;
String btSensorName;

// Latest reading from the Bluetooth sensor (valid only when found is true)
struct BtReading {
    bool found = false;
    bool tempValid = false;
    float tempC = NAN;
    uint16_t batteryMv = 0;
    uint16_t counter = 0;
    int rssi = 0;
};


// Time & quiet-hours helpers
bool haveValidTime()
{
    time_t now = time(NULL);
    return (now > 1600000000); // after 2020
}

// Seconds since 1970 for a UTC date (days-from-civil algorithm, no timezone involved)
time_t utcFromCivil(int y, int m, int d, int hh, int mm, int ss)
{
    y -= m <= 2;
    int era = (y >= 0 ? y : y - 399) / 400;
    int yoe = y - era * 400;
    int doy = (153 * (m + (m > 2 ? -3 : 9)) + 2) / 5 + d - 1;
    int doe = yoe * 365 + yoe / 4 - yoe / 100 + doy;
    int64_t days = (int64_t)era * 146097 + doe - 719468;
    return (time_t)(days * 86400 + hh * 3600 + mm * 60 + ss);
}

time_t parseHTTPDate(String dateStr)
{
    // Parse "Date: 16 Feb 2026 19:01:43 GMT" or "Date: Sun, 16 Feb 2026 19:01:43 GMT"
    const char* months[] = {"Jan", "Feb", "Mar", "Apr", "May", "Jun",
                            "Jul", "Aug", "Sep", "Oct", "Nov", "Dec"};
    
    // Find "Date: " in the header
    int idx = dateStr.indexOf("Date: ");
    if (idx == -1) {
        Serial.println("Date header not found");
        return 0;
    }
    
    // Extract the date line
    String dateLine = dateStr.substring(idx + 6);
    int eol = dateLine.indexOf('\r');
    if (eol > 0) dateLine = dateLine.substring(0, eol);
    eol = dateLine.indexOf('\n');
    if (eol > 0) dateLine = dateLine.substring(0, eol);
    
    Serial.printf("Parsing date: '%s'\n", dateLine.c_str());
    
    // Extract components - handle optional day name
    int day, year, hour, min, sec;
    char month_str[4];
    char dayname[10];
    
    // Try with day name first: "Sun, 16 Feb 2026 19:01:43 GMT"
    int matched = sscanf(dateLine.c_str(), "%9[^,], %d %3s %d %d:%d:%d", 
                         dayname, &day, month_str, &year, &hour, &min, &sec);
    
    // If that fails, try without day name: "16 Feb 2026 19:01:43 GMT"
    if (matched != 7) {
        matched = sscanf(dateLine.c_str(), "%d %3s %d %d:%d:%d", 
                        &day, month_str, &year, &hour, &min, &sec);
        if (matched != 6) {
            Serial.printf("Date parsing failed, matched %d fields\n", matched);
            return 0;
        }
    }
    
    // Find month number
    int month = -1;
    for (int i = 0; i < 12; i++) {
        if (strcmp(month_str, months[i]) == 0) {
            month = i;
            break;
        }
    }
    if (month == -1) {
        Serial.printf("Unknown month: %s\n", month_str);
        return 0;
    }
    
    // Convert to a UTC timestamp (HTTP Date is always in GMT). Not mktime(): that applies the local timezone
    return utcFromCivil(year, month + 1, day, hour, min, sec);
}

void syncTimeFromHTTP(String httpHeader)
{
    Serial.println("Extracting time from HTTP header...");
    time_t http_time = parseHTTPDate(httpHeader);
    
    if (http_time > 1600000000) {
        // The clock runs in UTC; LOCAL_TIMEZONE turns it into Danish time where needed
        struct timeval tv = { .tv_sec = http_time, .tv_usec = 0 };
        settimeofday(&tv, NULL);

        struct tm timeinfo;
        localtime_r(&http_time, &timeinfo);
        Serial.printf("Time synced from HTTP: %04d-%02d-%02d %02d:%02d:%02d Danish time\n",
                      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println("HTTP time parsing failed");
    }
}

// Seconds until the next targetHour:00 Danish time (daylight saving handled by mktime)
uint32_t secondsUntilLocalHour(int targetHour)
{
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    t.tm_hour = targetHour;
    t.tm_min = 0;
    t.tm_sec = 0;
    t.tm_isdst = -1;
    time_t target = mktime(&t);
    if (target <= now) {
        t.tm_mday += 1;
        t.tm_hour = targetHour;
        t.tm_isdst = -1;
        target = mktime(&t);
    }
    return (uint32_t)(target - now);
}

// True between ACTIVE_START_HOUR and ACTIVE_END_HOUR Danish time
bool inActiveHours()
{
    time_t now = time(NULL);
    struct tm t;
    localtime_r(&now, &t);
    return t.tm_hour >= ACTIVE_START_HOUR && t.tm_hour < ACTIVE_END_HOUR;
}

// Deep sleep without touching the modem (it is already off)
void sleepWithoutModem(uint32_t sec)
{
#ifdef MODEM_RESET_PIN
    gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
    gpio_deep_sleep_hold_en();
#endif
    Serial.printf("Sleeping %u s without the modem\n", sec);
    Serial.flush();
    esp_sleep_enable_timer_wakeup((uint64_t)sec * uS_TO_S_FACTOR);
    esp_deep_sleep_start();
}

void initThermocouplePins()
{
    // CS high starts a conversion; it takes ~220 ms, so do this early in setup()
    pinMode(MAX6675_CS_PIN, OUTPUT);
    digitalWrite(MAX6675_CS_PIN, HIGH);
    pinMode(MAX6675_SCK_PIN, OUTPUT);
    // Pull-up: a loose SO wire then reads 0xFFFF instead of floating. The MAX6675 drives SO while CS is low.
    pinMode(MAX6675_SO_PIN, INPUT_PULLUP);
}

// One raw read of the MAX6675, so wiring faults can be told apart from real readings.
// Returns NAN on any fault.
float readThermocoupleOnce()
{
    uint16_t raw = 0;
    digitalWrite(MAX6675_CS_PIN, LOW);
    delayMicroseconds(10);
    for (int i = 15; i >= 0; i--) {
        digitalWrite(MAX6675_SCK_PIN, LOW);
        delayMicroseconds(10);
        if (digitalRead(MAX6675_SO_PIN)) {
            raw |= (1 << i);
        }
        digitalWrite(MAX6675_SCK_PIN, HIGH);
        delayMicroseconds(10);
    }
    digitalWrite(MAX6675_CS_PIN, HIGH);

#if MAX6675_DEBUG
    Serial.printf("MAX6675 raw: 0x%04X\n", raw);
#endif

    const char *fault = NULL;
    if (raw == 0xFFFF) {
        fault = "SO is always HIGH - SO wire (pin 19) loose, or no VCC/GND to the module";
    } else if (raw == 0x0000) {
        // Also what a powerless module returns; a real 0.00-0.24 C reading is dropped too
        fault = "SO is always LOW - check the CS (13) / SCK (18) wires and GND to the module";
    } else if (raw & 0x8002) {
        // Bit 15 and bit 1 are always 0 on a working MAX6675
        fault = "invalid bits set - loose wire or noise on SO/SCK";
    } else if (raw & 0x0004) {
        fault = "thermocouple open - check the probe wires at the screw terminals";
    }

    if (fault) {
        Serial.printf("MAX6675 fault: %s\n", fault);
        return NAN;
    }
    return (raw >> 3) * 0.25;
}

// Two reads that must agree and be plausible: a loose wire gives random values
// that can pass the bit checks, but rarely the same value twice.
float readThermocoupleCelsius()
{
    float first = readThermocoupleOnce();
    if (isnan(first)) {
        return NAN;
    }
    delay(250);  // MAX6675 needs up to 220 ms for a new conversion
    float second = readThermocoupleOnce();
    if (isnan(second)) {
        return NAN;
    }

    if (fabsf(first - second) > MAX6675_MAX_DIFF_C) {
        Serial.printf("MAX6675 fault: unstable reading (%.2f C vs %.2f C) - loose wire?\n", first, second);
        return NAN;
    }
    float temp = (first + second) / 2;
    if (temp > MAX6675_MAX_PLAUSIBLE_C) {
        Serial.printf("MAX6675 fault: %.2f C is above %.0f C - loose wire?\n", temp, MAX6675_MAX_PLAUSIBLE_C);
        return NAN;
    }
    return temp;
}

// Outdoor temperature from a DS18B20, or NAN when no sensor answers or the value is invalid
float readOutdoorCelsius()
{
    OneWire oneWire(DS18B20_PIN);
    DallasTemperature sensors(&oneWire);
    sensors.begin();
    if (sensors.getDeviceCount() == 0) {
        Serial.println("DS18B20: no sensor found, outdoor temperature left out");
        return NAN;
    }
    sensors.requestTemperatures();  // Blocks up to 750 ms at 12-bit resolution
    float t = sensors.getTempCByIndex(0);
    // -127 = lost contact, 85 = power-on value (no conversion done)
    if (t == DEVICE_DISCONNECTED_C || t == 85.0f || t < -55.0f || t > 85.0f) {
        Serial.printf("DS18B20 fault: invalid reading %.2f C\n", t);
        return NAN;
    }
    return t;
}

// Print a URL without the ThingSpeak write key
void printMaskedUrl(const char *label, const char *url)
{
    String masked(url);
    if (thingspeakKey.length() > 0) {
        masked.replace(thingspeakKey, "****");
    }
    Serial.print(label);
    Serial.println(masked);
}

// ---------------------------------------------------------------- settings in NVS
void loadSettings()
{
    prefs.begin("a7670", false);
#ifdef PROVISION_SECRETS
    prefs.putString("phone", ALERT_PHONE_NUMBER);
    prefs.putString("tskey", THINGSPEAK_API_KEY);
#ifdef OTA_FOLDER
    prefs.putString("ota_dir", OTA_FOLDER);
#else
    prefs.remove("ota_dir");
#endif
#ifdef BT_SENSOR_NAME
    prefs.putString("bt_name", BT_SENSOR_NAME);
#else
    prefs.remove("bt_name");
#endif
    Serial.println("PROVISION: secrets written to NVS");
#endif
    alertPhone = prefs.getString("phone", "");
    thingspeakKey = prefs.getString("tskey", "");
    thingspeakUrl = String("https://api.thingspeak.com/update?api_key=") + thingspeakKey;
    String folder = prefs.getString("ota_dir", OTA_DEFAULT_FOLDER);
    // Only plain folder names, so a bad value cannot break the URL
    bool folderOk = folder.length() > 0;
    for (size_t i = 0; i < folder.length(); i++) {
        char c = folder[i];
        if (!isalnum((unsigned char)c) && c != '-' && c != '_') folderOk = false;
    }
    if (!folderOk) {
        Serial.println("WARNING: invalid ota_dir in NVS, using " OTA_DEFAULT_FOLDER);
        folder = OTA_DEFAULT_FOLDER;
    }
    otaBaseUrl = String(OTA_BASE_URL) + folder + "/";
    Serial.printf("OTA folder: %s\n", folder.c_str());
    btSensorName = prefs.getString("bt_name", BT_DEFAULT_NAME);
    Serial.printf("Bluetooth sensor: %s\n", btSensorName.c_str());
    if (alertPhone.length() == 0 || thingspeakKey.length() == 0) {
        Serial.println("WARNING: secrets missing in NVS - flash the T-A7670X-provision build once");
    }
}

// ---------------------------------------------------------------- OTA rollback
// Tell the Arduino core not to accept a new OTA image at boot. We confirm it ourselves
// once the network works; if the new firmware resets before that, the bootloader rolls back.
extern "C" bool verifyRollbackLater()
{
    return true;
}

bool otaPendingVerify()
{
    esp_ota_img_states_t state;
    const esp_partition_t *running = esp_ota_get_running_partition();
    return esp_ota_get_state_partition(running, &state) == ESP_OK && state == ESP_OTA_IMG_PENDING_VERIFY;
}

// Called early in setup(), before the modem is powered.
// The bootloader is built with CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP: a deep-sleep wake
// boots the firmware that ran before the sleep, so a freshly installed image only starts after
// a normal reset. Restart into it if it is still waiting, and notice real rollbacks.
void otaCheckRollback()
{
    const esp_partition_t *running = esp_ota_get_running_partition();
    const esp_partition_t *boot = esp_ota_get_boot_partition();
    esp_ota_img_states_t state;
    if (boot && boot != running && esp_ota_get_state_partition(boot, &state) == ESP_OK &&
        state == ESP_OTA_IMG_NEW) {
        Serial.println("OTA: new firmware is installed but not started, restarting into it");
        Serial.flush();
        esp_restart();
    }

    uint32_t tried = prefs.getUInt("ota_try", 0);
    if (tried > FIRMWARE_VERSION && esp_ota_get_last_invalid_partition() != NULL) {
        // The bootloader marked the new image invalid and went back to this one
        Serial.printf("OTA: version %u was rolled back, it will not be installed again\n", tried);
        prefs.putUInt("ota_bad", tried);
        prefs.remove("ota_try");
    }
}

// Called once the network is registered: the new firmware works well enough to keep
void otaConfirmIfPending()
{
    if (otaPendingVerify()) {
        esp_ota_mark_app_valid_cancel_rollback();
        prefs.remove("ota_try");
        prefs.remove("ota_runs");
        Serial.printf("OTA: firmware version %d confirmed\n", FIRMWARE_VERSION);
    }
}

// ---------------------------------------------------------------- OTA download
// Read exactly len bytes of the current HTTP response body with a single AT+HTTPREAD.
// One read per response works whatever the modem's READMODE is.
bool otaReadBody(uint8_t *buf, size_t len)
{
    modem.sendAT("+HTTPREAD=0,", (uint32_t)len);
    if (modem.waitResponse(5000UL) != 1) {
        return false;
    }
    size_t got = 0;
    while (got < len) {
        if (modem.waitResponse(30000UL, "+HTTPREAD: ") != 1) {
            return false;
        }
        int n = SerialAT.readStringUntil('\n').toInt();
        if (n <= 0 || got + n > len) {
            return false;
        }
        if (SerialAT.readBytes(buf + got, n) != (size_t)n) {
            return false;
        }
        got += n;
    }
    modem.waitResponse(5000UL, "+HTTPREAD: 0");
    return true;
}

// Download the image in Range chunks, check the MD5 and (unless dry run) write it to the OTA partition
bool otaDownload(uint32_t size, const char *md5hex, bool dryRun)
{
    uint8_t *buf = (uint8_t *)malloc(OTA_CHUNK_SIZE);
    if (!buf) {
        Serial.println("OTA: out of memory");
        return false;
    }
    if (!dryRun) {
        if (!Update.begin(size)) {
            Serial.printf("OTA: Update.begin failed: %s\n", Update.errorString());
            free(buf);
            return false;
        }
        Update.setMD5(md5hex);
    }

    MD5Builder md5;
    md5.begin();
    uint32_t offset = 0;
    int lastPercent = -10;
    bool ok = true;
    while (offset < size) {
        uint32_t len = min((uint32_t)OTA_CHUNK_SIZE, size - offset);
        char range[40];
        snprintf(range, sizeof(range), "bytes=%lu-%lu", (unsigned long)offset, (unsigned long)(offset + len - 1));

        ok = false;
        for (int attempt = 0; attempt < 3 && !ok; attempt++) {
            esp_task_wdt_reset();
            // Fresh HTTP session per chunk: the modem does not replace USERDATA (the Range header)
            // within one session, and a second Range header makes the server answer 501
            if (!modem.https_begin() || !modem.https_set_ssl_index(0) ||
                !modem.https_set_url((otaBaseUrl + "firmware.bin").c_str()) || !modem.https_add_header("Range", range)) {
                delay(1000);
                continue;
            }
            size_t bodyLen = 0;
            int status = modem.https_get(&bodyLen);
            if (status != 206 || bodyLen != len) {
                Serial.printf("OTA: chunk at %lu: HTTP %d, %u bytes (expected 206, %lu)\n",
                              (unsigned long)offset, status, (unsigned)bodyLen, (unsigned long)len);
                if (bodyLen > 0 && bodyLen < 1024) {
                    Serial.println(modem.https_body().substring(0, 300));  // Server error text
                }
                delay(1000);
                continue;
            }
            ok = otaReadBody(buf, len);
            if (!ok) {
                Serial.printf("OTA: reading chunk at %lu failed\n", (unsigned long)offset);
            }
        }
        if (!ok) {
            break;
        }

        md5.add(buf, len);
        if (!dryRun && Update.write(buf, len) != len) {
            Serial.printf("OTA: flash write failed: %s\n", Update.errorString());
            ok = false;
            break;
        }
        offset += len;

        int percent = (int)((uint64_t)offset * 100 / size);
        if (percent - lastPercent >= 10 || offset == size) {
            Serial.printf("OTA: %d%% (%lu/%lu)\n", percent, (unsigned long)offset, (unsigned long)size);
            lastPercent = percent;
        }
    }
    free(buf);

    if (!ok) {
        if (!dryRun) {
            Update.abort();
        }
        return false;
    }
    md5.calculate();
    if (!md5.toString().equalsIgnoreCase(md5hex)) {
        Serial.printf("OTA: MD5 mismatch (got %s, expected %s)\n", md5.toString().c_str(), md5hex);
        if (!dryRun) {
            Update.abort();
        }
        return false;
    }
    if (dryRun) {
        Serial.println("OTA: DRY RUN - download and MD5 OK, nothing written");
        return true;
    }
    if (!Update.end()) {
        Serial.printf("OTA: Update.end failed: %s\n", Update.errorString());
        return false;
    }
    return true;
}

// Check the manifest and install a newer version. Runs after the ThingSpeak upload.
void checkForOtaUpdate(uint32_t battery_voltage)
{
    time_t now = time(NULL);
    if (haveValidTime() && last_ota_check != 0 && now >= last_ota_check &&
        now - last_ota_check < OTA_CHECK_INTERVAL_SEC) {
        return;  // Checked recently
    }
    if (battery_voltage < OTA_MIN_VOLTAGE) {
        Serial.printf("OTA: battery %umV below %umV, check postponed\n", battery_voltage, (uint32_t)OTA_MIN_VOLTAGE);
        return;
    }
    Serial.printf("OTA: checking for updates (running version %d)\n", FIRMWARE_VERSION);
    esp_task_wdt_reset();
    bool http_ready = modem.https_begin();
    if (!http_ready) {
        delay(3000);
        http_ready = modem.https_begin();
    }
    if (!http_ready) {
        Serial.println("OTA: HTTPINIT failed, check postponed to the next cycle");
        return;
    }
    configureSSLForOTA();
    if (!modem.https_set_url((otaBaseUrl + "version.txt").c_str())) {
        modem.https_end();
        return;
    }
    int status = modem.https_get();
    if (status != 200) {
        Serial.printf("OTA: version.txt HTTP %d\n", status);
        modem.https_end();
        return;
    }
    String manifest = modem.https_body();
    if (haveValidTime()) {
        last_ota_check = now;  // Only a completed check counts, failures are retried next cycle
    }

    // version.txt: "<version> <size> <md5>"
    unsigned long version = 0, size = 0;
    char md5hex[33] = {0};
    if (sscanf(manifest.c_str(), "%lu %lu %32s", &version, &size, md5hex) != 3 || strlen(md5hex) != 32) {
        Serial.println("OTA: version.txt has an unexpected format");
        modem.https_end();
        return;
    }
    if (version <= FIRMWARE_VERSION) {
        Serial.printf("OTA: up to date (latest %lu)\n", version);
        modem.https_end();
        return;
    }
    if (version <= prefs.getUInt("ota_bad", 0)) {
        Serial.printf("OTA: version %lu was rolled back before, skipped\n", version);
        modem.https_end();
        return;
    }
    const esp_partition_t *next = esp_ota_get_next_update_partition(NULL);
    if (!next || size == 0 || size > next->size) {
        Serial.printf("OTA: image size %lu does not fit the OTA partition\n", size);
        modem.https_end();
        return;
    }

    Serial.printf("OTA: downloading version %lu (%lu bytes)%s\n", version, size, OTA_DRY_RUN ? " - DRY RUN" : "");
    bool ok = otaDownload(size, md5hex, OTA_DRY_RUN);
    modem.https_end();
    if (ok && !OTA_DRY_RUN) {
        prefs.putUInt("ota_try", version);
        prefs.remove("ota_runs");
        Serial.printf("OTA: version %lu installed, restarting into it\n", version);
        // Restart instead of deep sleep: a deep-sleep wake would boot the old firmware again
        modem.poweroff();
        delay(3000);
        Serial.flush();
        esp_restart();
    } else if (!ok) {
        Serial.println("OTA: update failed, the current firmware keeps running");
    }
}

// ---------------------------------------------------------------- data connection
// Send an AT command and print the full response (diagnostics only)
void logAT(const char *cmd)
{
    String response;
    modem.sendAT(cmd);
    modem.waitResponse(5000UL, response);
    response.replace("\r\n", " ");
    response.trim();
    Serial.printf("AT%s -> %s\n", cmd, response.c_str());
}

// IP address of PDP context 1, the context AT+HTTPINIT uses. Empty when it is not active.
String pdpAddress()
{
    String response;
    modem.sendAT("+CGPADDR=1");
    if (modem.waitResponse(5000UL, response) != 1) {
        return "";
    }
    int start = response.indexOf("+CGPADDR: 1,");
    if (start < 0) {
        return "";
    }
    String ip = response.substring(start + 12);
    int end = ip.indexOf('\r');
    if (end >= 0) {
        ip = ip.substring(0, end);
    }
    ip.replace("\"", "");
    ip.trim();
    return (ip == "0.0.0.0") ? "" : ip;
}

void sendLowBatterySMS(uint32_t voltage)
{
    char message[100];
    // ASCII only: the SMS is sent in the GSM character set
    snprintf(message, sizeof(message), "Lavt batteri: %umV. Enheden stopper under %umV. Oplad snart.",
             voltage, (uint32_t)MINIMUM_MODEM_VOLTAGE);

    Serial.println("Sending low battery SMS...");
    if (alertPhone.length() == 0) {
        Serial.println("No alert phone number in NVS - SMS skipped");
        return;
    }
    if (modem.sendSMS(alertPhone, message)) {
        Serial.println("Low battery SMS sent");
        low_battery_alert_sent = true;  // Only one SMS until the battery has recovered
    } else {
        Serial.println("Low battery SMS failed, will retry next cycle");
    }
}

void powerModemDown()
{
    // Read battery voltage to determine sleep duration
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN) * 2;
    
    // Pull up DTR to put the modem into sleep
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, HIGH);
    delay(5000);

    // modem shutdown sequence...
    LOG("Check modem online .\n");
    unsigned long start = millis();
    while (!modem.testAT())
    {
        esp_task_wdt_reset();
        if (millis() - start > 15000) break; // timeout
        Serial.print(".");
        delay(500);
    }
    Serial.println("Modem is online !");

    delay(5000);  // Keep this longer

    Serial.println("Enter modem power off!");

    if (modem.poweroff())
    {
        Serial.println("Modem enter power off modem!");
    }
    else
    {
        Serial.println("modem power off failed!");
    }

    delay(5000);  // Keep this longer

    Serial.println("Check modem response .");
    start = millis();
    while (modem.testAT())
    {
        esp_task_wdt_reset();
        if (millis() - start > 15000) break;
        Serial.print(".");
        delay(500);
    }
    Serial.println("Modem is not respond, modem has power off !");

    delay(2000);

#ifdef BOARD_POWERON_PIN
    // Turn off DC boost to power off the modem
    digitalWrite(BOARD_POWERON_PIN, LOW);
#endif

#ifdef MODEM_RESET_PIN
    // Keep it low during the sleep period
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
    gpio_deep_sleep_hold_en();
#endif

    Serial.println("Enter esp32 goto deepsleep!");
    Serial.flush();

    // remove current task from WDT and deinit the WDT to avoid reset during sleep entry
    esp_task_wdt_delete(NULL);
    esp_task_wdt_deinit();
    delay(50);

    // The next wake is a cheap temperature sample; the schedule decides when the modem runs again
    Serial.printf("Battery %umV, next sample in %d s\n", battery_voltage, SAMPLE_INTERVAL_SEC);

    // Enable only timer wakeup and enter deep sleep.
    esp_sleep_enable_timer_wakeup((uint64_t)SAMPLE_INTERVAL_SEC * uS_TO_S_FACTOR);
    delay(200);
    esp_deep_sleep_start();
}

bool setupNetwork(uint8_t maxAttempts = 3) {
    Serial.println("Setting up network connection...");
    
    // Check modem registration status
    RegStatus status;
    uint8_t attempts = 0;
    
    while (attempts < maxAttempts) {
        status = modem.getRegistrationStatus();
        
        switch (status) {
            case REG_OK_HOME:
                Serial.println("Registered to home network");
                break;
            case REG_OK_ROAMING:
                Serial.println("Registered to roaming network");
                break;
            case REG_SEARCHING:
                Serial.println("Searching for network...");
                delay(2000);
                attempts++;
                continue;
            case REG_DENIED:
                Serial.println("Network registration denied!");
                return false;
            case REG_UNREGISTERED:
            case REG_UNKNOWN:
                Serial.println("Not registered to network");
                delay(2000);
                attempts++;
                continue;
            default:
                Serial.printf("Unknown registration status: %d\n", status);
                delay(2000);
                attempts++;
                continue;
        }
        
        // If we're here, we're registered (either home or roaming)
        if (status == REG_OK_HOME || status == REG_OK_ROAMING) {
            // Check signal quality
            int16_t signalQuality = modem.getSignalQuality();
            Serial.printf("Signal Quality: %d\n", signalQuality);
            
            // Enable network connection
            if (modem.enableNetwork()) {
                // Wait for network to stabilize
                delay(5000);
                
                // Verify we have an IP address
                String ip = modem.getLocalIP();
                if (ip != "0.0.0.0" && ip.length() > 0) {
                    Serial.printf("Network connected with IP: %s\n", ip.c_str());
                    return true;
                }
            }
            
            Serial.println("Failed to enable network");
        }
        
        attempts++;
        delay(1000);
    }
    
    Serial.println("Network setup failed after maximum attempts");
    return false;
}

static const char* resetReasonToStr(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_UNKNOWN: return "UNKNOWN";
        case ESP_RST_POWERON: return "POWERON";
        case ESP_RST_EXT: return "EXTERNAL";
        case ESP_RST_SW: return "SW_CPU_RESET";
        case ESP_RST_PANIC: return "PANIC";
        case ESP_RST_INT_WDT: return "INT_WDT";
        case ESP_RST_TASK_WDT: return "TASK_WDT";
        case ESP_RST_WDT: return "WDT";
        case ESP_RST_DEEPSLEEP: return "DEEPSLEEP";
        case ESP_RST_BROWNOUT: return "BROWNOUT";
        case ESP_RST_SDIO: return "SDIO";
        default: return "OTHER";
    }
}

static const char* wakeReasonToStr(esp_sleep_wakeup_cause_t w) {
    switch (w) {
        case ESP_SLEEP_WAKEUP_EXT0: return "EXT0";
        case ESP_SLEEP_WAKEUP_EXT1: return "EXT1";
        case ESP_SLEEP_WAKEUP_TIMER: return "TIMER";
        case ESP_SLEEP_WAKEUP_TOUCHPAD: return "TOUCHPAD";
        case ESP_SLEEP_WAKEUP_ULP: return "ULP";
        case ESP_SLEEP_WAKEUP_GPIO: return "GPIO";
        default: return "UNDEFINED";
    }
}

static void printResetAndWakeReason() {
    esp_reset_reason_t rr = esp_reset_reason();
    Serial.printf("ESP reset reason: %d (%s)\n", (int)rr, resetReasonToStr(rr));
    esp_sleep_wakeup_cause_t wc = esp_sleep_get_wakeup_cause();
    Serial.printf("ESP wakeup cause: %d (%s)\n", (int)wc, wakeReasonToStr(wc));
}

// Check if this is a cold start (powered via USB without battery)
bool isColdStart() {
    esp_reset_reason_t reason = esp_reset_reason();
    // Cold start: power on reset or external reset
    return (reason == ESP_RST_POWERON || reason == ESP_RST_EXT);
}

// Battery voltage in mV, averaged over 8 samples. Read with the modem off, so cycles compare.
uint32_t readBatteryMilliVolts()
{
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogReadMilliVolts(BOARD_BAT_ADC_PIN);
        delay(2);
    }
    return sum / 8 * 2;  // The board divides the battery voltage by 2
}

void optimizeModemPower() {
    // Try to reduce modem activity when not needed
    modem.sendAT("+CSCLK=2"); // enable modem sleep
    delay(200);
}

bool configureSSLForOTA() {
    Serial.println("Configuring SSL for HTTPS...");
    
    // Configure SSL context 0 for HTTPS
    // Set SSL/TLS version to TLS 1.2 (value 4)
    modem.sendAT("+CSSLCFG=\"sslversion\",0,4");
    if (modem.waitResponse(3000) != 1) {
        Serial.println("Failed to set SSL version");
        return false;
    }
    
    // Set authentication mode to 0 (no certificate validation)
    // For production, consider using mode 1 with proper CA certificates
    modem.sendAT("+CSSLCFG=\"authmode\",0,0");
    if (modem.waitResponse(3000) != 1) {
        Serial.println("Failed to set auth mode");
        return false;
    }
    
    // Enable SNI (Server Name Indication) for GitHub
    modem.sendAT("+CSSLCFG=\"enableSNI\",0,1");
    if (modem.waitResponse(3000) != 1) {
        Serial.println("Failed to enable SNI");
        return false;
    }
    
    // Set SSL context for HTTP client
    if (!modem.https_set_ssl_index(0)) {
        Serial.println("Failed to set SSL index");
        return false;
    }
    
    Serial.println("SSL configuration complete");
    return true;
}

// Scan for the Bluetooth sensor. WiFi is never started and Bluetooth only runs during this scan,
// so no radio needs switching off elsewhere (deep sleep powers both down anyway).
BtReading scanBtSensor()
{
    BtReading r;
    Serial.printf("Bluetooth: scanning %d s for %s\n", BT_SCAN_SEC, btSensorName.c_str());
    NimBLEDevice::init("");
    NimBLEScan *scan = NimBLEDevice::getScan();
    scan->setActiveScan(true);  // The name is in the scan response
    scan->setInterval(100);
    scan->setWindow(99);
    NimBLEScanResults results = scan->getResults(BT_SCAN_SEC * 1000, false);
    for (int i = 0; i < results.getCount(); i++) {
        const NimBLEAdvertisedDevice *dev = results.getDevice(i);
        if (!dev->haveName() || btSensorName != dev->getName().c_str()) continue;
        std::string md = dev->getManufacturerData();
        const uint8_t *d = (const uint8_t *)md.data();
        if (md.size() < 9 || (d[0] | (d[1] << 8)) != BT_COMPANY_ID || d[2] != BT_FORMAT_VERSION) {
            Serial.printf("Bluetooth: %s found, but with unknown data (%u bytes)\n", btSensorName.c_str(), (unsigned)md.size());
            continue;
        }
        r.found = true;
        r.counter = d[3] | (d[4] << 8);
        int16_t centi = (int16_t)(d[5] | (d[6] << 8));
        r.tempValid = centi != BT_TEMP_INVALID;
        r.tempC = r.tempValid ? centi / 100.0f : NAN;
        r.batteryMv = d[7] | (d[8] << 8);
        r.rssi = dev->getRSSI();
        break;
    }
    scan->clearResults();
    NimBLEDevice::deinit(true);

    if (!r.found) {
        Serial.println("Bluetooth: sensor not heard, fields 6-8 left out");
    } else if (r.tempValid) {
        Serial.printf("Bluetooth: %.2f C, battery %u mV, RSSI %d dBm, counter %u\n", r.tempC, r.batteryMv, r.rssi, r.counter);
    } else {
        Serial.printf("Bluetooth: no temperature (sensor fault), battery %u mV, RSSI %d dBm, counter %u\n", r.batteryMv,
                      r.rssi, r.counter);
    }
    return r;
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    setenv("TZ", LOCAL_TIMEZONE, 1);
    tzset();
    printResetAndWakeReason();
    initThermocouplePins();
    Serial.printf("Firmware version: %d%s\n", FIRMWARE_VERSION, otaPendingVerify() ? " (new, waiting for confirmation)" : "");

    // RTC data from another build (power-on, or the first wake after an OTA update) is not valid
    if (rtc_magic != RTC_MAGIC) {
        modem_initialized = false;
        low_battery_alert_sent = false;
        last_ota_check = 0;
        last_report_time = 0;
        last_report_temp = NAN;
        last_report_heating = false;
        rtc_magic = RTC_MAGIC;
    }

    loadSettings();
    otaCheckRollback();

    // Read battery voltage FIRST, before the modem draws current
    uint32_t battery_voltage = readBatteryMilliVolts();

    Serial.printf("Battery voltage: %umV\n", battery_voltage);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("\n*** Wakeup from deep sleep ***");
    }

    // Indoor temperature first: it decides whether the modem is needed at all
    delay(250);  // The MAX6675 needs a full conversion after CS went high in initThermocouplePins()
    float temp = readThermocoupleCelsius();
    if (isnan(temp)) {
        Serial.println("Thermocouple read failed, indoor temperature left out");
    } else {
        Serial.printf("Indoor temp (MAX6675): %.1f C\n", temp);
    }

    // Rising since the last report (someone lit the sauna, or it is still heating up)
    bool heating = !isnan(temp) && !isnan(last_report_temp) && temp >= last_report_temp + HEATING_RISE_C;

    // Schedule. A new OTA image and a board without a clock always report, to confirm or to get the time
    if (haveValidTime() && !otaPendingVerify()) {
        if (!inActiveHours()) {
            uint32_t sec = secondsUntilLocalHour(ACTIVE_START_HOUR);
            Serial.printf("Outside active hours - sleeping until %02d:00\n", ACTIVE_START_HOUR);
            sleepWithoutModem(sec);
        }
        time_t now = time(NULL);
        float ref = isnan(temp) ? last_report_temp : temp;
        bool hot = !isnan(ref) && ref >= HOT_THRESHOLD_C;
        uint32_t interval = (hot || last_report_heating) ? REPORT_INTERVAL_HOT_SEC : REPORT_INTERVAL_SEC;
        bool due = last_report_time == 0 || now < last_report_time ||
                   now - last_report_time + REPORT_SLACK_SEC >= (time_t)interval;
        if (heating && !last_report_heating) {
            Serial.printf("Temperature rose %.1f C since the last report - reporting now\n", temp - last_report_temp);
        } else if (!due) {
            Serial.printf("No report due (last %ld s ago, interval %u s)\n", (long)(now - last_report_time), interval);
            sleepWithoutModem(SAMPLE_INTERVAL_SEC);
        }
    }
    // From here the modem runs. Remember this attempt, so a failing network does not start it every 5 minutes
    last_report_time = time(NULL);
    last_report_temp = temp;
    last_report_heating = heating;

    // Count only runs that use the modem, so a new image gets its full chances to upload
    if (otaPendingVerify()) {
        uint32_t runs = prefs.getUInt("ota_runs", 0) + 1;
        prefs.putUInt("ota_runs", runs);
        if (runs > OTA_MAX_UNCONFIRMED_RUNS) {
            // Deep-sleep wakes skip the bootloader's rollback check, so do it ourselves
            Serial.printf("OTA: not confirmed after %d runs, rolling back\n", OTA_MAX_UNCONFIRMED_RUNS);
            prefs.remove("ota_runs");
            Serial.flush();
            esp_ota_mark_app_invalid_rollback_and_reboot();
        }
        Serial.printf("OTA: unconfirmed run %u of %d\n", runs, OTA_MAX_UNCONFIRMED_RUNS);
    }

    // Check if battery is too low for modem startup
    // Skip this check if no battery is detected (USB-only power)
    bool usb_only = (battery_voltage < NO_BATTERY_VOLTAGE);
    if (usb_only) {
        Serial.println("No battery detected - running on USB power only");
        battery_voltage = 4200;  // Use nominal value so battery checks pass
    } else if (battery_voltage < MINIMUM_MODEM_VOLTAGE) {
        Serial.printf("Battery too low (%umV) for modem startup. Entering long sleep...\n", battery_voltage);
        // Sleep for 2 hours to allow charging/recovery
        esp_sleep_enable_timer_wakeup(2 * 60 * 60 * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }

    // Bluetooth before the modem starts, so the two never draw current at the same time
    BtReading bt = scanBtSensor();

    // Check if this is a cold start from USB power
    if (isColdStart()) {
        Serial.println("Cold start detected - powering on modem automatically");
        modem_initialized = false;  // Force full initialization on cold start
#ifdef BOARD_POWERON_PIN
        pinMode(BOARD_POWERON_PIN, OUTPUT);
        digitalWrite(BOARD_POWERON_PIN, HIGH);
        delay(500);
#endif
    }

    SerialAT.setRxBufferSize(4096);  // Large HTTPREAD bursts during OTA
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    // Initialize watchdog timer to prevent hangs during modem startup
#if ESP_ARDUINO_VERSION_MAJOR >= 3
    esp_task_wdt_config_t wdt_config = {};
    wdt_config.timeout_ms = 120000;
    wdt_config.idle_core_mask = 0;
    wdt_config.trigger_panic = true;
    esp_err_t wdt_result = esp_task_wdt_reconfigure(&wdt_config);
    if (wdt_result == ESP_ERR_INVALID_STATE) {
        wdt_result = esp_task_wdt_init(&wdt_config);
    }
#else
    esp_err_t wdt_result = esp_task_wdt_init(120, true);
#endif
    if (wdt_result != ESP_OK) {
        Serial.printf("Watchdog configuration failed: %d\n", wdt_result);
    }

    wdt_result = esp_task_wdt_add(NULL);
    if (wdt_result != ESP_OK && wdt_result != ESP_ERR_INVALID_STATE) {
        Serial.printf("Watchdog task registration failed: %d\n", wdt_result);
    }
    Serial.println("Watchdog configured (120s timeout)");

    // Configure ADC for power saving
    analogSetPinAttenuation(BOARD_BAT_ADC_PIN, ADC_11db);
#if ESP_ARDUINO_VERSION_MAJOR < 3
    analogSetClockDiv(255);
#endif
    
    // Optimize modem power
    optimizeModemPower();

    analogSetAttenuation(ADC_11db);
    analogReadResolution(12);

#if CONFIG_IDF_TARGET_ESP32
    analogSetWidth(12);
#endif

#ifdef BOARD_POWERON_PIN
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
    delay(1000);
#endif

    // Always a full start: powerModemDown() powers the modem off completely, so there is
    // nothing to wake via DTR. DTR low keeps the modem awake while we use it.
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, LOW);
    {
        Serial.println("Full modem initialization...");
        // Release the reset pin hold set before deep sleep, otherwise the pulse below has no effect
        gpio_hold_dis((gpio_num_t)MODEM_RESET_PIN);
        gpio_deep_sleep_hold_dis();

        // Set modem reset pin with longer stabilization
        pinMode(MODEM_RESET_PIN, OUTPUT);
        digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
        delay(300);
        digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
        delay(5000);  // Critical for low voltage stability
        digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
        delay(1000);

        pinMode(BOARD_PWRKEY_PIN, OUTPUT);
        digitalWrite(BOARD_PWRKEY_PIN, LOW);
        delay(200);
        digitalWrite(BOARD_PWRKEY_PIN, HIGH);
        delay(200);
        digitalWrite(BOARD_PWRKEY_PIN, LOW);

        delay(4000);  // Allow full modem boot
        modem_initialized = true;  // Mark as initialized for next wake
    }

    Serial.println("Verifying modem is online...");

    int retry = 0;
    int max_retries = 20;
    
    while (!modem.testAT(1000))
    {
        Serial.println(".");
        esp_task_wdt_reset();  // Reset watchdog during modem checks
        if (retry++ > max_retries)
        {
            Serial.printf("Modem failed after %d retries at %umV. Entering recovery sleep...\n", max_retries, battery_voltage);
            modem_initialized = false;  // Reset flag so next wake does full init
            // Deinit watchdog before sleep
            esp_task_wdt_delete(NULL);
            esp_task_wdt_deinit();
            // Sleep 30 minutes before retry
            esp_sleep_enable_timer_wakeup(30 * 60 * uS_TO_S_FACTOR);
            esp_deep_sleep_start();
        }
    }
    Serial.println("Modem online!");

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    unsigned long sim_wait_start = millis();
    while (sim != SIM_READY)
    {
        esp_task_wdt_reset();  // Reset watchdog during SIM checks
        if (millis() - sim_wait_start > SIM_READY_TIMEOUT_MS)
        {
            Serial.println("SIM card not ready before timeout. Powering down until next cycle...");
            modem_initialized = false;  // Do a full init on next wake
            powerModemDown();
        }
        sim = modem.getSimStatus();
        switch (sim)
        {
        case SIM_READY:
            Serial.println("SIM card online");
            break;
        case SIM_LOCKED:
            Serial.println("The SIM card is locked. Please unlock the SIM card first.");
            break;
        default:
            break;
        }
        delay(1000);
    }

#ifndef TINY_GSM_MODEM_SIM7672
    if (!modem.setNetworkMode(MODEM_NETWORK_AUTO))
    {
        Serial.println("Set network mode failed!");
    }
    String mode = modem.getNetworkModes();
    Serial.print("Current network mode : ");
    Serial.println(mode);
#endif

#ifdef NETWORK_APN
    Serial.printf("Set network apn : %s\n", NETWORK_APN);
    modem.sendAT(GF("+CGDCONT=1,\"IP\",\""), NETWORK_APN, "\"");
    if (modem.waitResponse() != 1)
    {
        Serial.println("Set network apn error !");
    }
#endif

    // Check network registration status
    int16_t sq;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    unsigned long reg_wait_start = millis();
    // REG_EMERGENCY (11) can show up briefly after start; it gives no data service, so keep waiting
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED ||
           status == REG_EMERGENCY)
    {
        esp_task_wdt_reset();  // Reset watchdog during network registration
        if (millis() - reg_wait_start > NETWORK_REG_TIMEOUT_MS)
        {
            Serial.println("Network registration timed out. Powering down until next cycle...");
            modem_initialized = false;  // Do a full init on next wake
            powerModemDown();
        }
        status = modem.getRegistrationStatus();
        switch (status)
        {
        case REG_UNREGISTERED:
        case REG_SEARCHING:
            sq = modem.getSignalQuality();
            Serial.printf("[%lu] Signal Quality:%d\n", millis() / 1000, sq);
            delay(1000);
            break;
        case REG_DENIED:
            Serial.println("Network registration was rejected, please check if the APN is correct");
            modem_initialized = false;  // Do a full init on next wake
            powerModemDown();  // Does not return: enters deep sleep
            break;
        case REG_OK_HOME:
            Serial.println("Online registration successful");
            break;
        case REG_OK_ROAMING:
            Serial.println("Network registration successful, currently in roaming mode");
            break;
        default:
            Serial.printf("Registration Status:%d\n", status);
            delay(1000);
            break;
        }
    }
    Serial.println();

    Serial.printf("Registration Status:%d\n", status);
    delay(1000);


    String ueInfo;
    if (modem.getSystemInformation(ueInfo))
    {
        Serial.print("Inquiring UE system information:");
        Serial.println(ueInfo);
    }

    // Data connection. HTTP (AT+HTTPINIT) uses PDP context 1; it is checked and activated below
    // with a realistic timeout, and details are logged when it fails.
    // Socket service: HTTP does not need it, but while it is open the data connection stays up
    // between HTTP sessions (the upload, the OTA check and one session per OTA chunk). Without it
    // a new AT+HTTPINIT failed right after the upload. NETOPEN can take up to 120 s to answer;
    // the library waits only 1 s, so wait here instead.
    modem.sendAT("+NETOPEN");
    int netopen = modem.waitResponse(30000UL, "+NETOPEN: 0", "+IP ERROR: Network is already opened");
    if (netopen != 1 && netopen != 2) {
        Serial.println("NETOPEN not confirmed within 30 s");
    }

    String ipAddress = pdpAddress();
    for (int attempt = 1; attempt <= 3 && ipAddress.length() == 0; attempt++) {
        esp_task_wdt_reset();
        Serial.printf("No IP on PDP context 1 (attempt %d/3), activating it\n", attempt);
        modem.sendAT("+CGACT=1,1");
        modem.waitResponse(30000UL);
        delay(2000);
        ipAddress = pdpAddress();
    }
    Serial.print("Network IP:");
    Serial.println(ipAddress);
    if (ipAddress.length() == 0) {
        Serial.println("No data connection yet - network details:");
        logAT("+CEREG?");
        logAT("+CGATT?");
        logAT("+CGACT?");
        logAT("+CSQ");
        Serial.println("Trying the upload anyway, AT+HTTPINIT may still bring the connection up");
    }

    Serial.printf("Battery:%umV\n", battery_voltage);

    // Network is registered, so the SMS can go out here
    if (battery_voltage < LOW_BATTERY_THRESHOLD && !low_battery_alert_sent) {
        esp_task_wdt_reset();  // Sending an SMS can take a while
        sendLowBatterySMS(battery_voltage);
    } else if (battery_voltage >= LOW_BATTERY_RESET_THRESHOLD) {
        low_battery_alert_sent = false;
    }

    // The indoor temperature was read at wake-up; the outdoor sensor only matters for an upload
    float outdoor = readOutdoorCelsius();
    if (!isnan(outdoor)) {
        Serial.printf("Outdoor temp (DS18B20): %.1f C\n", outdoor);
    }

    // Daily voltage drop for field5: compare with a reference stored in NVS a day ago
    bool vdrop_ready = false;
    int32_t vdrop_mv = 0;
    time_t now_time = time(NULL);
    if (!usb_only && haveValidTime()) {
        uint32_t ref_mv = prefs.getUInt("vref_mv", 0);
        int64_t ref_time = prefs.getLong64("vref_t", 0);
        int64_t elapsed = (int64_t)now_time - ref_time;
        if (ref_mv == 0 || ref_time == 0 || elapsed < 0 || elapsed > VDROP_MAX_AGE_SEC) {
            prefs.putUInt("vref_mv", battery_voltage);
            prefs.putLong64("vref_t", now_time);
            Serial.printf("vDrop: new reference %umV, first value in 24 h\n", battery_voltage);
        } else if (elapsed >= VDROP_PERIOD_SEC) {
            // Periods run 24-34 h (hourly reports, no reports at night), so scale the drop to exactly 24 h
            vdrop_mv = (int32_t)(((int64_t)ref_mv - (int64_t)battery_voltage) * VDROP_PERIOD_SEC / elapsed);
            vdrop_ready = true;
            Serial.printf("vDrop: %ldmV per 24 h, measured over %.1f h (%umV -> %umV)\n", (long)vdrop_mv, elapsed / 3600.0,
                          ref_mv, battery_voltage);
        }
    }

    // Initialize HTTPS
    modem.https_begin();

    // Send data only ONCE per cycle
    int httpRetry = RETRY_COUNT;
    bool upload_skipped = false;
    if (thingspeakKey.length() == 0) {
        Serial.println("No ThingSpeak key in NVS - upload skipped");
        upload_skipped = true;
    }
    if (upload_skipped) {
        httpRetry = 0;
    }
    bool sent_successfully = false;

    while (httpRetry-- && !sent_successfully)
    {
        esp_task_wdt_reset();  // Reset watchdog during HTTPS operations
        
        printMaskedUrl("Request URL : ", thingspeakUrl.c_str());

        // Field layout: see "ThingSpeak fields" at the top. Temperature fields are left out
        // on a sensor fault, so ThingSpeak shows a gap instead of 0
        char str[MAX_URL_LENGTH];
        size_t used = snprintf(str, MAX_URL_LENGTH, "%s&field1=%d", thingspeakUrl.c_str(), FIRMWARE_VERSION);
        if (!isnan(outdoor) && used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field2=%.1f", outdoor);
        }
        if (!isnan(temp) && used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field3=%.0f", temp);
        }
        if (used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field4=%u", battery_voltage);
        }
        if (vdrop_ready && used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field5=%ld", (long)vdrop_mv);
        }
        if (bt.found && bt.tempValid && used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field6=%.1f", bt.tempC);
        }
        if (bt.found && used < MAX_URL_LENGTH) {
            used += snprintf(str + used, MAX_URL_LENGTH - used, "&field7=%u&field8=%d", bt.batteryMv, bt.rssi);
        }

        printMaskedUrl("Request URL with data: ", str);

        if (!modem.https_set_url(str))
        {
            Serial.println("Failed to set URL, retrying...");
            delay(2000);
            continue;
        }

        esp_task_wdt_reset();  // Reset before potentially long HTTPS operation
        
        int httpCode = modem.https_get();
        if (httpCode != 200)
        {
            Serial.print("HTTP get failed ! error code = ");
            Serial.println(httpCode);
            delay(2000);
            continue;
        }

        String header = modem.https_header();
        Serial.print("HTTP Header : ");
        Serial.println(header);
        
        // Extract time from HTTP Date header for next cycle
        syncTimeFromHTTP(header);
        
        delay(500);

        String body = modem.https_body();
        Serial.print("HTTP body : ");
        Serial.println(body);

        sent_successfully = true;
        delay(1000);
    }

    if (!sent_successfully && !upload_skipped) {
        Serial.println("Failed to send data after retries");
    }

    // Data reached ThingSpeak: the new firmware works well enough to keep
    if (sent_successfully) {
        otaConfirmIfPending();
    }

    // The day's value is on ThingSpeak: start the next 24 h period from here
    if (vdrop_ready && sent_successfully) {
        prefs.putUInt("vref_mv", battery_voltage);
        prefs.putLong64("vref_t", now_time);
    }
    
    // Close HTTPS session after all network operations
    modem.https_end();

    // Only when the upload worked: a poor connection cannot download 1 MB anyway
    if (sent_successfully) {
        checkForOtaUpdate(battery_voltage);
    }

    Serial.println("-------------------------------------");

    Serial.println("Preparing for deep sleep...");
    Serial.flush();

    powerModemDown();
}

void loop()
{
    // Debug AT passthrough
    if (SerialAT.available())
    {
        Serial.write(SerialAT.read());
    }
    if (Serial.available())
    {
        SerialAT.write(Serial.read());
    }
    delay(1);
}
