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
#include <DallasTemperature.h>
#include <OneWire.h>
#include <esp32-hal-adc.h>
#include <max6675.h>
#include <esp_wifi.h>
#include <esp_bt.h>
#include "driver/adc.h"
#include "esp_adc_cal.h"
#include "WiFi.h"

#define ONE_WIRE_BUS 0

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

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
DallasTemperature Dallas_sensors(&oneWire);

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
    // Pull up DTR to put the modem into sleep
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, HIGH);

    // Delay sometime ...
    delay(5000);  // Keep this longer for modem to respond

    Serial.println("Check modem online .");
    while (!modem.testAT())
    {
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
    while (modem.testAT())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("Modem is not respone ,modem has power off !");

    delay(5000);

#ifdef BOARD_POWERON_PIN
    // Turn on DC boost to power off the modem
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

void setup()
{
    Serial.begin(115200); // Set console baud rate
    Serial.println("Start Sketch");
    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
    {
        Serial.println("Wakeup timer");
        int i = 5;
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
#endif

    // Set modem reset pin ,reset modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(100);
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(2600);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(100);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    Dallas_sensors.begin();

    // Check if the modem is online
    Serial.println("Start modem...");

    int retry = 0;
    while (!modem.testAT(1000))
    {
        Serial.println(".");
        if (retry++ > 10)
        {
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(100);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(1000);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            retry = 0;
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
            // const char *SIMCARD_PIN_CODE = "123456";
            // modem.simUnlock(SIMCARD_PIN_CODE);
            break;
        default:
            break;
        }
        delay(1000);
    }

    // SIM7672G Can't set network mode
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

    // Check network registration status and network signal status
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

    Dallas_sensors.requestTemperatures(); // Send the command to get temperatures
    Serial.print("Temperature for the device 1 (index 0) is: ");
    float temp = 21; // Dallas_sensors.getTempCByIndex(0);
    Serial.println(temp);

    float tempIn = thermocouple.readCelsius();
    temp = tempIn;
    Serial.printf("thermo Coupler temp: %.0f\n", tempIn);

    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_voltage *= 2; // The hardware voltage divider resistor is half of the actual voltage, multiply it by 2 to get the true voltage

    char buf[256];
#ifdef BOARD_SOLAR_ADC_PIN_NOT
    uint32_t solar_voltage = analogReadMilliVolts(BOARD_SOLAR_ADC_PIN);
    solar_voltage *= 2; // The hardware voltage divider resistor is half of the actual voltage, multiply it by 2 to get the true voltage
    snprintf(buf, 256, "Battery:%umV \tSolar:%umV", battery_voltage, solar_voltage);
#else
    snprintf(buf, 256, "Battery:%umV ", battery_voltage);
#endif
    Serial.println(buf);

    // Initialize HTTPS
    modem.https_begin();

    // If the status code 715 is returned, please see here
    // https://github.com/Xinyuan-LilyGO/LilyGO-T-A76XX/issues/117

    for (int i = 0; i < sizeof(request_url) / sizeof(request_url[0]); ++i)
    {

        int retry = 3;

        while (retry--)
        {

            Serial.print("Request URL : ");
            Serial.println(request_url[i]);
            char str[192];
            sprintf(str, "%s&field1=%i&field2=%i&field3=%.0f&field4=%.0f", request_url[i], battery_voltage, solar_voltage, temp, tempIn);

            Serial.print("Request URL with data: ");
            Serial.println(str);

            // Set GET URT
            if (!modem.https_set_url(str /*request_url[i])*/))
            {
                Serial.print("Failed to request : ");
                Serial.println(request_url[i]);
                delay(3000);
                continue;
            }

            // Send GET request
            int httpCode = 0;
            httpCode = modem.https_get();
            if (httpCode != 200)
            {
                Serial.print("HTTP get failed ! error code = ");
                Serial.println(httpCode);
                delay(3000);
                continue;
            }

            // Get HTTPS header information
            String header = modem.https_header();
            Serial.print("HTTP Header : ");
            Serial.println(header);

            delay(1000);

            // Get HTTPS response
            String body = modem.https_body();
            Serial.print("HTTP body : ");
            Serial.println(body);

            delay(3000);

            break;
        }

        Serial.println("-------------------------------------");
    }

    powerModemDown();
}

void loop()
{
    // Debug AT
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

// Add this function to handle sleep mode
void enterSleepMode(uint32_t battery_voltage) {
    if (battery_voltage < DEEP_SLEEP_THRESHOLD) {
        Serial.println("Battery low, entering deep sleep...");
        
        // Properly handle Dallas temperature sensor
        Dallas_sensors.setResolution(9);  // Set lowest resolution to save power
        Dallas_sensors.requestTemperatures();  // Complete any pending operations
        
        // Power down other peripherals
        digitalWrite(BOARD_PWRKEY_PIN, LOW);
        
        // Enter deep sleep for longer duration
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MINUTES * 2 * 60 * uS_TO_S_FACTOR);
    } else {
        Serial.println("Entering normal sleep cycle...");
        esp_sleep_enable_timer_wakeup(SLEEP_DURATION_MINUTES * 60 * uS_TO_S_FACTOR);
    }
    
    // Configure GPIO pins for sleep
    gpio_deep_sleep_hold_en();
    
    Serial.println("Entering deep sleep...");
    Serial.flush();
    
    esp_deep_sleep_start();
}

// Add this function for modem power optimization
void optimizeModemPower() {
    // Disable unnecessary modem features
    modem.sendAT("+CFUN=0");  // Minimum functionality
    modem.sendAT("+CSCLK=2"); // Enable deep sleep mode
    
    // Only enable full functionality when needed
    modem.sendAT("+CFUN=1");
    // Wait for network registration
    delay(2000);
}

// Add this function for efficient sensor reading
bool readSensorsWithTimeout() {
    unsigned long timeout = millis();
    const unsigned long SENSOR_TIMEOUT = 1000; // 1 second timeout
    
    Dallas_sensors.requestTemperatures();
    while (!Dallas_sensors.isConversionComplete()) {
        if (millis() - timeout > SENSOR_TIMEOUT) {
            return false;
        }
        delay(10);
    }
    return true;
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

