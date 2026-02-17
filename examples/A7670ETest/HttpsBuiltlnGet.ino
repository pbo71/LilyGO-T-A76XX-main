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
#include <TinyGsmClient.h>
#include <WiFi.h>
#include <max6675.h>
#include <esp32-hal-adc.h>
#include <Update.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <esp_task_wdt.h>    // <-- added for watchdog support
#include "esp_sleep.h"
#include "esp_system.h"
#include <time.h>
#include <sys/time.h>
#include "lwip/apps/sntp.h"

// debug: set 0 to remove Serial prints to save power
#define DEBUG 0
#if DEBUG
  #define LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(fmt, ...)
#endif

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 * 20     /* Time ESP32 will go to sleep (in seconds) */

// Define constants instead of magic numbers
#define RETRY_COUNT 3
#define MAX_URL_LENGTH 192
#define VOLTAGE_MULTIPLIER 2
#define MODEM_INIT_RETRY 10
#define SLEEP_DURATION_MINUTES 20
#define CHECK_BATTERY_THRESHOLD 3600  // 3.6V in millivolts
#define DEEP_SLEEP_THRESHOLD 3500     // 3.5V in millivolts - critical battery protection
#define MINIMUM_MODEM_VOLTAGE 4000    // 4.0V minimum for reliable modem startup

// OTA Update settings
#define OTA_MINIMUM_VOLTAGE 3800      // 3.8V minimum for safe OTA update
#define CURRENT_FIRMWARE_VERSION 1    // Increment this when you release new firmware
// GitHub "latest" URLs automatically point to newest release
#define OTA_FIRMWARE_URL "https://github.com/pbo71/LilyGO-T-A76XX-main/releases/latest/download/firmware.bin"
#define OTA_VERSION_URL "https://github.com/pbo71/LilyGO-T-A76XX-main/releases/latest/download/version.txt"

// Low battery alert settings
#define LOW_BATTERY_THRESHOLD 3600  // 3.6V in millivolts - alert earlier for recharge time
#define ALERT_PHONE_NUMBER "+4540959135"

// MAX6675 thermocouple (using only MAX sensor, Dallas/OneWire removed)
MAX6675 thermocouple(GPIO_NUM_18, BOARD_SD_CS_PIN, GPIO_NUM_19);

#ifdef DUMP_AT_COMMANDS // if enabled it requires the streamDebugger lib
#include <StreamDebugger.h>
StreamDebugger debugger(SerialAT, Serial);
TinyGsm modem(debugger);
#else
TinyGsm modem(SerialAT);
#endif

#define BOARD_SOLAR_ADC_PIN_NOT

// It depends on the operator whether to set up an APN. If some operators do not set up an APN,
// they will be rejected when registering for the network. You need to ask the local operator for the specific APN.
// APNs from other operators are welcome to submit PRs for filling.
#define NETWORK_APN "internet" // CHN-CT: China Telecom

// RTC memory to persist data across deep sleep
RTC_DATA_ATTR bool low_battery_alert_sent = false;
RTC_DATA_ATTR uint32_t cycle_count = 0;
RTC_DATA_ATTR uint32_t total_mah_used = 0;
RTC_DATA_ATTR uint32_t startup_voltage = 0;
RTC_DATA_ATTR time_t last_synced_time = 0;
RTC_DATA_ATTR uint32_t wakes_since_sync = 0;
RTC_DATA_ATTR bool modem_initialized = false;
RTC_DATA_ATTR uint32_t last_ota_check_time = 0;  // Track when we last checked for updates

bool isCharning = false;

const char *request_url[] = {
    "https://api.thingspeak.com/update?api_key=04T1JJYY542SD2GL"};

// Time & quiet-hours helpers
void estimateCurrentTime()
{
    if (last_synced_time > 1600000000) {
        // Each wake cycle is TIME_TO_SLEEP seconds apart
        uint32_t elapsed_sec = wakes_since_sync * TIME_TO_SLEEP;
        time_t estimated = last_synced_time + elapsed_sec;
        struct timeval tv = { .tv_sec = estimated, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        Serial.printf("Estimated time from RTC: %lu (wakes: %u)\n", estimated, wakes_since_sync);
        wakes_since_sync++;  // Increment for next cycle
    }
}

bool haveValidTime()
{
    time_t now = time(NULL);
    return (now > 1600000000); // after 2020
}

time_t parseHTTPDate(String dateStr)
{
    // Parse "Date: 16 Feb 2026 19:01:43 GMT" or "Date: Sun, 16 Feb 2026 19:01:43 GMT"
    struct tm tm_time = {0};
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
    int matched = sscanf(dateLine.c_str(), "%[^,], %d %3s %d %d:%d:%d", 
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
    
    tm_time.tm_mday = day;
    tm_time.tm_mon = month;
    tm_time.tm_year = year - 1900;
    tm_time.tm_hour = hour;
    tm_time.tm_min = min;
    tm_time.tm_sec = sec;
    tm_time.tm_isdst = 0;
    
    // Convert to UTC timestamp (HTTP Date is always in GMT)
    time_t utc_time = mktime(&tm_time);
    return utc_time;
}

void syncTimeFromHTTP(String httpHeader)
{
    Serial.println("Extracting time from HTTP header...");
    time_t http_time = parseHTTPDate(httpHeader);
    
    if (http_time > 1600000000) {
        // Add GMT+1 timezone offset (3600 seconds)
        http_time += 3600;
        
        struct timeval tv = { .tv_sec = http_time, .tv_usec = 0 };
        settimeofday(&tv, NULL);
        
        last_synced_time = http_time;
        wakes_since_sync = 0;  // Reset counter after successful sync
        
        struct tm timeinfo;
        localtime_r(&http_time, &timeinfo);
        Serial.printf("Time synced from HTTP: %04d-%02d-%02d %02d:%02d:%02d GMT+1\n",
                      timeinfo.tm_year + 1900, timeinfo.tm_mon + 1, timeinfo.tm_mday,
                      timeinfo.tm_hour, timeinfo.tm_min, timeinfo.tm_sec);
    } else {
        Serial.println("HTTP time parsing failed");
    }
}

uint32_t secondsUntilHour(int targetHour)
{
    time_t now = time(NULL);
    struct tm tm_now;
    gmtime_r(&now, &tm_now);
    int hour = tm_now.tm_hour;
    int minute = tm_now.tm_min;
    int second = tm_now.tm_sec;

    int delta = targetHour - hour;
    if (delta <= 0) delta += 24;
    uint32_t seconds = delta * 3600 - minute * 60 - second;
    return seconds;
}

bool shouldReportNow()
{
    time_t now = time(NULL);
    struct tm tm_now;
    gmtime_r(&now, &tm_now);
    int hour = tm_now.tm_hour;
    // Don't report between 00:00 (inclusive) and 06:00 (exclusive)
    return !(hour >= 0 && hour < 6);
}

void chargingFunction()
{
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_voltage *= 2; // The hardware voltage divider resistor is half of the actual voltage, multiply it by 2 to get the true voltage

    Serial.print("Check Charging - battery voltage = ");
    Serial.println(battery_voltage);

    if (battery_voltage >= 4200)
    {
        Serial.println("Stop Charging battery ....");
        digitalWrite(RELAY_PIN, HIGH); // Stop charging
        delay(1000);
        digitalWrite(RELAY_PIN, HIGH);
        isCharning = false;
    }
    else
    {
        Serial.println("Start Charging battery ....");
        pinMode(RELAY_PIN, OUTPUT);
        digitalWrite(RELAY_PIN, HIGH);
        delay(500);
        digitalWrite(RELAY_PIN, LOW); // Start charging
        Serial.println("RELAY PIN High");
        delay(500);
        digitalWrite(RELAY_PIN, HIGH);
        isCharning = true;
    }
}

// Improve the existing charging function
void optimizedChargingFunction() {
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN) * 2;
    
    static const uint32_t CHARGING_START_THRESHOLD = 3800;  // 3.8V
    static const uint32_t CHARGING_STOP_THRESHOLD = 4200;   // 4.2V
    
    if (battery_voltage >= CHARGING_STOP_THRESHOLD) {
        digitalWrite(RELAY_PIN, HIGH);  // Stop charging
        isCharning = false;
    } else if (battery_voltage < CHARGING_START_THRESHOLD) {
        digitalWrite(RELAY_PIN, LOW);   // Start charging
        isCharning = true;
    }
    // No change if voltage is between thresholds (hysteresis)
}

void powerModemDown()
{
    // Read battery voltage to determine sleep duration
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN) * 2;
    
    // Disable WiFi and Bluetooth before sleep
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop();
    btStop();

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

    // Adaptive sleep duration based on battery level
    uint32_t sleep_duration;
    if (battery_voltage < 4000) {
        sleep_duration = 60 * 30;  // 30 minutes when battery is low
        Serial.printf("Low battery (%umV), extending sleep to 30 min\n", battery_voltage);
    } else {
        sleep_duration = TIME_TO_SLEEP;  // Normal 20 minutes
        Serial.printf("Normal battery (%umV), sleeping 20 min\n", battery_voltage);
    }
    
    // Enable only timer wakeup and enter deep sleep.
    esp_sleep_enable_timer_wakeup(sleep_duration * uS_TO_S_FACTOR);
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

void sendLowBatterySMS() {
    Serial.println("Sending low battery alert SMS...");
    
    char message[128];
    snprintf(message, sizeof(message), 
             "Low battery alert! Current voltage: %umV. Device will enter deep sleep.", 
             analogReadMilliVolts(BOARD_BAT_ADC_PIN) * 2);
    
    if (modem.sendSMS(ALERT_PHONE_NUMBER, message)) {
        Serial.println("SMS sent successfully!");
        low_battery_alert_sent = true;  // Mark as sent to prevent spam
    } else {
        Serial.println("Failed to send SMS");
    }
    delay(1000);
}

// Check if this is a cold start (powered via USB without battery)
bool isColdStart() {
    esp_reset_reason_t reason = esp_reset_reason();
    // Cold start: power on reset or external reset
    return (reason == ESP_RST_POWERON || reason == ESP_RST_EXT);
}

void logPowerConsumption(uint32_t current_voltage, uint32_t previous_voltage) {
    if (previous_voltage == 0) {
        Serial.println("First cycle - no previous voltage to compare");
        return;
    }

    // Rough estimation: voltage drop correlates to mAh used
    // LiPo discharge curve: ~3.7V nominal, usable 3.0V-4.2V = 1.2V range
    // Simplified: ~4 mAh per 1mV drop (varies by battery chemistry)
    uint32_t voltage_drop = previous_voltage - current_voltage;
    uint32_t estimated_mah = voltage_drop * 4 / 100;  // mAh estimate
    
    if (voltage_drop > 0) {
        total_mah_used += estimated_mah;
        cycle_count++;
        
        Serial.println("\n=== Power Consumption Report ===");
        Serial.printf("Cycle: %d\n", cycle_count);
        Serial.printf("Voltage drop this cycle: %umV\n", voltage_drop);
        Serial.printf("Estimated mAh used this cycle: %umAh\n", estimated_mah);
        Serial.printf("Total mAh used so far: %umAh\n", total_mah_used);
        Serial.printf("Current voltage: %umV\n", current_voltage);
        
        // Estimate remaining runtime
        uint32_t remaining_mah = (current_voltage - 3400) * 4 / 100;  // down to 3.4V cutoff
        uint32_t avg_mah_per_cycle = total_mah_used / cycle_count;
        uint32_t estimated_cycles_remaining = (remaining_mah > 0) ? remaining_mah / avg_mah_per_cycle : 0;
        uint32_t estimated_hours_remaining = (estimated_cycles_remaining * 20) / 60;
        
        Serial.printf("Avg mAh per cycle: %umAh\n", avg_mah_per_cycle);
        Serial.printf("Estimated cycles remaining: %d\n", estimated_cycles_remaining);
        Serial.printf("Estimated runtime remaining: %d hours\n", estimated_hours_remaining);
        Serial.println("================================\n");
    }
}

void optimizeModemPower() {
    // Try to reduce modem activity when not needed
    modem.sendAT("+CSCLK=2"); // enable modem sleep
    delay(200);
}

bool checkForOTAUpdate() {
    Serial.println("Checking for firmware updates...");
    
    // Check version from server
    if (!modem.https_set_url(OTA_VERSION_URL)) {
        Serial.println("Failed to set version check URL");
        return false;
    }
    
    int httpCode = modem.https_get();
    if (httpCode != 200) {
        Serial.printf("Version check failed, HTTP code: %d\n", httpCode);
        return false;
    }
    
    String versionStr = modem.https_body();
    versionStr.trim();
    int serverVersion = versionStr.toInt();
    
    Serial.printf("Current version: %d, Server version: %d\n", CURRENT_FIRMWARE_VERSION, serverVersion);
    
    if (serverVersion > CURRENT_FIRMWARE_VERSION) {
        Serial.println("New firmware available!");
        return true;
    }
    
    Serial.println("Firmware is up to date");
    return false;
}

bool performOTAUpdate() {
    Serial.println("=== Starting OTA Update ===");
    
    // Set firmware URL
    if (!modem.https_set_url(OTA_FIRMWARE_URL)) {
        Serial.println("Failed to set firmware URL");
        return false;
    }
    
    // Download firmware
    Serial.println("Downloading firmware...");
    int httpCode = modem.https_get();
    if (httpCode != 200) {
        Serial.printf("Firmware download failed, HTTP code: %d\n", httpCode);
        return false;
    }
    
    // Get firmware size
    size_t firmware_size = modem.https_get_size();
    Serial.printf("Firmware size: %d bytes\n", firmware_size);
    
    if (firmware_size == 0 || firmware_size > 2000000) {  // Max 2MB sanity check
        Serial.println("Invalid firmware size");
        return false;
    }
    
    // Begin update
    if (!Update.begin(firmware_size)) {
        Serial.println("Not enough space for OTA update");
        return false;
    }
    
    // Download and write firmware
    uint8_t buffer[1024];
    int written = 0;
    int progress = 0;
    int total = 0;
    
    Serial.println("Writing firmware...");
    while (1) {
        esp_task_wdt_reset();  // Keep watchdog happy during long operation
        
        int len = modem.https_body(buffer, 1024);
        if (len == 0) break;
        
        written = Update.write(buffer, len);
        if (written != len) {
            Serial.printf("Write error: %d/%d bytes\n", written, len);
            Update.abort();
            return false;
        }
        
        total += written;
        int newProgress = (total * 100) / firmware_size;
        if (newProgress - progress >= 10 || newProgress == 100) {
            progress = newProgress;
            Serial.printf("Progress: %d%%\n", progress);
        }
    }
    
    // Finalize update
    if (!Update.end()) {
        Serial.printf("Update failed, error #%d\n", Update.getError());
        return false;
    }
    
    if (!Update.isFinished()) {
        Serial.println("Update not finished");
        return false;
    }
    
    Serial.println("=== OTA Update Successful ===");
    Serial.println("Rebooting in 3 seconds...");
    delay(3000);
    
    esp_restart();
    return true;  // Never reached
}

void disableWiFiAndBT() {
    // Disable WiFi
    WiFi.mode(WIFI_OFF);
    
    // Initialize WiFi before trying to stop it
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    esp_err_t result = esp_wifi_init(&cfg);
    if (result == ESP_OK) {
        result = esp_wifi_stop();
        if (result == ESP_OK) {
            result = esp_wifi_deinit();
            if (result != ESP_OK) {
                Serial.printf("WiFi deinit failed with error: %d\n", result);
            }
        } else {
            Serial.printf("WiFi stop failed with error: %d\n", result);
        }
    } else {
        Serial.printf("WiFi init failed with error: %d\n", result);
    }

    // Disable Bluetooth
    btStop();
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_ENABLED) {
        if (esp_bt_controller_disable() != ESP_OK) {
            Serial.println("BT controller disable failed");
        }
    }
    
    if (esp_bt_controller_get_status() == ESP_BT_CONTROLLER_STATUS_INITED) {
        if (esp_bt_controller_mem_release(ESP_BT_MODE_BTDM) != ESP_OK) {
            Serial.println("BT controller memory release failed");
        }
    }
}

void setup()
{
    Serial.begin(115200);
    delay(50);
    printResetAndWakeReason();

    // Read battery voltage FIRST
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_voltage *= 2;
    
    Serial.printf("Battery voltage: %umV\n", battery_voltage);

    // Log power consumption on wakeup (compare to previous cycle)
    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("\n*** Wakeup from deep sleep ***");
        logPowerConsumption(battery_voltage, startup_voltage);
    }

    // Store current voltage for next cycle comparison
    startup_voltage = battery_voltage;

    // Estimate current time from last sync (if available)
    estimateCurrentTime();

    // If we have valid time and it's quiet hours (00:00-06:00), sleep until 06:00
    if (haveValidTime()) {
        if (!shouldReportNow()) {
            uint32_t sec = secondsUntilHour(6);
            Serial.printf("Quiet hours detected - sleeping %u seconds until 06:00\n", sec);
            esp_sleep_enable_timer_wakeup((uint64_t)sec * uS_TO_S_FACTOR);
            esp_deep_sleep_start();
        }
    }

    // Check if battery is too low for modem startup
    if (battery_voltage < MINIMUM_MODEM_VOLTAGE) {
        Serial.printf("Battery too low (%umV) for modem startup. Entering long sleep...\n", battery_voltage);
        // Sleep for 2 hours to allow charging/recovery
        esp_sleep_enable_timer_wakeup(2 * 60 * 60 * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }

    // Check if this is a cold start from USB power
    if (isColdStart()) {
        Serial.println("Cold start detected - powering on modem automatically");
        cycle_count = 0;  // Reset cycle counter on cold start
        total_mah_used = 0;
        startup_voltage = battery_voltage;
        modem_initialized = false;  // Force full initialization on cold start
#ifdef BOARD_POWERON_PIN
        pinMode(BOARD_POWERON_PIN, OUTPUT);
        digitalWrite(BOARD_POWERON_PIN, HIGH);
        delay(500);
#endif
    }

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    // Initialize watchdog timer to prevent hangs during modem startup
    esp_task_wdt_init(120, true);  // 120 second timeout with panic on timeout
    esp_task_wdt_add(NULL);       // Add current task to watchdog
    Serial.println("Watchdog timer initialized (120s timeout)");

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
    {
        Serial.println("Wakeup timer");
        int i = 15;
        while (i > 0)
        {
            Serial.printf("Modem will start in %d seconds\n", i);
            Serial.flush();
            delay(1000);
            i--;
        }
        Serial.println("TurnON Modem!");
    }

    // Configure ADC for power saving
    analogSetPinAttenuation(BOARD_BAT_ADC_PIN, ADC_11db);
    analogSetClockDiv(255);
    
    // Disable WiFi and Bluetooth
    disableWiFiAndBT();
    
    // Use optimized charging
    optimizedChargingFunction();
    
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

    // Try fast wake first if modem was previously initialized
    bool fast_wake_attempted = false;
    if (modem_initialized && esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER) {
        Serial.println("Attempting fast modem wake via DTR...");
        pinMode(MODEM_DTR_PIN, OUTPUT);
        digitalWrite(MODEM_DTR_PIN, LOW);  // Pull DTR low to wake
        delay(2000);
        
        // Quick test - does modem respond?
        fast_wake_attempted = true;
        if (!modem.testAT(1000)) {
            Serial.println("Fast wake failed, will do full reset");
            fast_wake_attempted = false;  // Mark as failed so we do full init below
        } else {
            Serial.println("Fast wake successful!");
        }
    }
    
    // Do full initialization if fast wake wasn't attempted or failed
    if (!fast_wake_attempted) {
        Serial.println("Full modem initialization...");
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
    while (sim != SIM_READY)
    {
        esp_task_wdt_reset();  // Reset watchdog during SIM checks
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
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED)
    {
        esp_task_wdt_reset();  // Reset watchdog during network registration
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
            return;
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

    if (!modem.enableNetwork())
    {
        Serial.println("Enable network failed!");
    }

    delay(5000);

    String ipAddress = modem.getLocalIP();
    Serial.print("Network IP:");
    Serial.println(ipAddress);

    char buf[256];
#ifdef BOARD_SOLAR_ADC_PIN_NOT
    uint32_t solar_voltage = analogReadMilliVolts(BOARD_SOLAR_ADC_PIN);
    solar_voltage *= 2;
    snprintf(buf, 256, "Battery:%umV \tSolar:%umV", battery_voltage, solar_voltage);
#else
    snprintf(buf, 256, "Battery:%umV ", battery_voltage);
#endif
    Serial.println(buf);

    // CHECK LOW BATTERY AND SEND SMS ONLY AFTER NETWORK IS READY
    if (battery_voltage < LOW_BATTERY_THRESHOLD && !low_battery_alert_sent) {
        Serial.printf("Low battery detected: %umV\n", battery_voltage);
        // Network is now ready, safe to send SMS
        sendLowBatterySMS();
        Serial.println("Skipping data upload due to low battery");
    } else if (battery_voltage >= LOW_BATTERY_THRESHOLD) {
        // Reset alert flag if battery recovers
        low_battery_alert_sent = false;

        // Read MAX6675 thermocouple ONLY when we're about to transmit (saves power)
        float tempIn = thermocouple.readCelsius();
        float temp = tempIn;
        if (isnan(temp)) {
            Serial.println("Thermocouple read failed, using fallback");
            temp = 0.0;
        }
        Serial.printf("Thermo Coupler temp: %.1f C\n", temp);

        // Initialize HTTPS
        modem.https_begin();

        // Send data only ONCE per cycle
        int httpRetry = RETRY_COUNT;
        bool sent_successfully = false;

        while (httpRetry-- && !sent_successfully)
        {
            esp_task_wdt_reset();  // Reset watchdog during HTTPS operations
            
            Serial.print("Request URL : ");
            Serial.println(request_url[0]);

            char str[MAX_URL_LENGTH];
#ifdef BOARD_SOLAR_ADC_PIN_NOT
            uint32_t solar_voltage = analogReadMilliVolts(BOARD_SOLAR_ADC_PIN) * VOLTAGE_MULTIPLIER;
            snprintf(str, MAX_URL_LENGTH, "%s&field1=%i&field2=%i&field3=%.0f&field4=%.0f", request_url[0], battery_voltage, solar_voltage, temp, tempIn);
#else
            snprintf(str, MAX_URL_LENGTH, "%s&field1=%i&field2=%.0f&field3=%.0f", request_url[0], battery_voltage, temp, tempIn);
#endif

            Serial.print("Request URL with data: ");
            Serial.println(str);

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

        if (!sent_successfully) {
            Serial.println("Failed to send data after retries");
        }
        
        // Check for OTA updates (only once per day to save data)
        time_t now = time(NULL);
        uint32_t seconds_since_last_check = now - last_ota_check_time;
        bool should_check_ota = (last_ota_check_time == 0) || (seconds_since_last_check > 86400);  // 24 hours
        
        if (should_check_ota && battery_voltage >= OTA_MINIMUM_VOLTAGE) {
            Serial.println("\n=== OTA Update Check ===");
            Serial.printf("Battery: %umV (safe for OTA)\n", battery_voltage);
            
            last_ota_check_time = now;
            
            if (checkForOTAUpdate()) {
                // New firmware available - perform update
                performOTAUpdate();
                // If we return here, update failed - continue normally
                Serial.println("OTA update failed, continuing with normal operation");
            }
        } else if (should_check_ota) {
            Serial.printf("Skipping OTA check - battery too low: %umV (need %umV)\n", 
                         battery_voltage, OTA_MINIMUM_VOLTAGE);
        }
    }

    Serial.println("-------------------------------------");

    Serial.println("Preparing for deep sleep...");
    Serial.flush();

    // Disable watchdog before deep sleep
    esp_task_wdt_deinit();

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

