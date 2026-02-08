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
#include <esp_wifi.h>
#include <esp_bt.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include <esp_task_wdt.h>    // <-- added for watchdog support
#include "esp_sleep.h"
#include "esp_system.h"

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
#define DEEP_SLEEP_THRESHOLD 3400     // 3.4V in millivolts

// Low battery alert settings
#define LOW_BATTERY_THRESHOLD 3400  // 3.4V in millivolts
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

bool isCharning = false;

const char *request_url[] = {
    "https://api.thingspeak.com/update?api_key=04T1JJYY542SD2GL"};

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

    // Enable only timer wakeup and enter deep sleep.
    // Do NOT call esp_sleep_disable_wakeup_source(...) unless you know that source was enabled.
    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
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

void setup()
{
    Serial.begin(115200);
    delay(50);
    printResetAndWakeReason();

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

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

    // Set modem reset pin
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(200);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(3000);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(500);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(150);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(150);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    Serial.println("Start modem...");

    int retry = 0;
    while (!modem.testAT(1000))
    {
        Serial.println(".");
        if (retry++ > MODEM_INIT_RETRY)
        {
            Serial.println("Modem not responding, toggling PWRKEY and retrying...");
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(200);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
            delay(3000);
        }
    }
    Serial.println();

    // Check if SIM card is online
    SimStatus sim = SIM_ERROR;
    while (sim != SIM_READY)
    {
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

    // Check network registration
    int16_t sq;
    Serial.print("Wait for the modem to register with the network.");
    RegStatus status = REG_NO_RESULT;
    while (status == REG_NO_RESULT || status == REG_SEARCHING || status == REG_UNREGISTERED)
    {
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

    // Read MAX6675 thermocouple
    float tempIn = thermocouple.readCelsius();
    float temp = tempIn;
    if (isnan(temp)) {
        Serial.println("Thermocouple read failed, using fallback");
        temp = 0.0;
    }
    Serial.printf("Thermo Coupler temp: %.1f C\n", temp);

    // Read battery voltage
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_voltage *= 2;

    // CHECK LOW BATTERY BEFORE SENDING DATA
    if (battery_voltage < LOW_BATTERY_THRESHOLD && !low_battery_alert_sent) {
        Serial.printf("Low battery detected: %umV\n", battery_voltage);
        sendLowBatterySMS();
        // Skip HTTPS data send when battery is low
        Serial.println("Skipping data upload due to low battery");
    } else if (battery_voltage >= LOW_BATTERY_THRESHOLD) {
        // Reset alert flag if battery recovers above threshold
        low_battery_alert_sent = false;
        
        char buf[256];
#ifdef BOARD_SOLAR_ADC_PIN_NOT
        uint32_t solar_voltage = analogReadMilliVolts(BOARD_SOLAR_ADC_PIN);
        solar_voltage *= 2;
        snprintf(buf, 256, "Battery:%umV \tSolar:%umV", battery_voltage, solar_voltage);
#else
        snprintf(buf, 256, "Battery:%umV ", battery_voltage);
#endif
        Serial.println(buf);

        // Initialize HTTPS
        modem.https_begin();

        // Send data only ONCE per wake cycle
        int httpRetry = RETRY_COUNT;
        bool sent_successfully = false;

        while (httpRetry-- && !sent_successfully)
        {
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
    }

    Serial.println("-------------------------------------");

    // ensure watchdog disabled before entering deep sleep
    Serial.println("Preparing for deep sleep...");
    Serial.flush();

    // Disable watchdog before deep sleep to avoid wake race conditions
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

// Add this function for modem power optimization
void optimizeModemPower() {
    // Try to reduce modem activity when not needed
    modem.sendAT("+CSCLK=2"); // enable modem sleep
    delay(200);
}

// Add this function for handling connection failures
void handleConnectionFailure() {
    static uint8_t failureCount = 0;
    failureCount++;
    
    if (failureCount >= 3) {
        Serial.println("Multiple connection failures, entering long sleep...");
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MINUTES * 3 * 60 * uS_TO_S_FACTOR);
        esp_deep_sleep_start();
    }
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

