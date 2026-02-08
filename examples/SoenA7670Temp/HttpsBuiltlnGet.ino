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

// debug: set 0 to remove Serial prints to save power
#define DEBUG 0

#if DEBUG
  #define LOG(fmt, ...) Serial.printf(fmt, ##__VA_ARGS__)
#else
  #define LOG(fmt, ...)
#endif

#include "utilities.h"
#include "WiFi.h"
#include <esp_wifi.h>
#include <TinyGsmClient.h>
//#include <DallasTemperature.h>
//#include <OneWire.h>
#include <esp32-hal-adc.h>
#include <max6675.h>
#include <esp_task_wdt.h>

//#define ONE_WIRE_BUS 0

#define uS_TO_S_FACTOR 1000000ULL /* Conversion factor for micro seconds to seconds */
#define TIME_TO_SLEEP 60 * 20     /* Time ESP32 will go to sleep (in seconds) */

// Setup a oneWire instance to communicate with any OneWire devices (not just Maxim/Dallas temperature ICs)
//OneWire oneWire(ONE_WIRE_BUS);

// Pass our oneWire reference to Dallas Temperature.
//DallasTemperature Dallas_sensors(&oneWire);

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

const char *request_url[] = {
    "https://api.thingspeak.com/update?api_key=TC63IQADZJ2IEBN5"};

void powerModemDownNoSleep()
{
    // Pull up DTR to put the modem into sleep
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, HIGH);

    // Delay sometime ...
    delay(5000);

    Serial.println("Check modem online .");
    while (!modem.testAT())
    {
        Serial.print(".");
        delay(500);
    }
    Serial.println("Modem is online !");

    delay(5000);

    Serial.println("Enter modem power off!");

    if (modem.poweroff())
    {
        Serial.println("Modem enter power off modem!");
    }
    else
    {
        Serial.println("modem power off failed!");
    }

    delay(5000);

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
}

#if 0
void chargingFunction()
{
    uint32_t battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
    battery_voltage *= 2; // The hardware voltage divider resistor is half of the actual voltage, multiply it by 2 to get the true voltage

    Serial.print("Check Charging - battery voltage = ");
    Serial.println(battery_voltage);

    if (battery_voltage < 3800)
    {
        pinMode(RELAY_PIN, OUTPUT);
        digitalWrite(RELAY_PIN, LOW);
        delay(500);
        digitalWrite(RELAY_PIN, HIGH); // Start charging
        Serial.println("RELAY PIN High");
        delay(500);
        digitalWrite(RELAY_PIN, LOW);

        powerModemDownNoSleep();

        while (battery_voltage < 4200)
        {
            battery_voltage = analogReadMilliVolts(BOARD_BAT_ADC_PIN);
            battery_voltage *= 2;

            Serial.print("Charging - battery voltage = ");
            Serial.println(battery_voltage);
            delay(60000);
        }

        digitalWrite(RELAY_PIN, HIGH); // Stop charging
        delay(1000);
        digitalWrite(RELAY_PIN, HIGH);
    }
}
#endif 
void powerModemDown()
{
    // Disable WiFi and Bluetooth before sleep
    WiFi.mode(WIFI_OFF);
    esp_wifi_stop(); // ensure WiFi radio is fully stopped
    btStop();

    // Pull up DTR to put the modem into sleep
    pinMode(MODEM_DTR_PIN, OUTPUT);
    digitalWrite(MODEM_DTR_PIN, HIGH);
    delay(5000);

    LOG("Check modem online.\n");
    unsigned long start = millis();
    while (!modem.testAT())
    {
        esp_task_wdt_reset();  // Reset watchdog during modem checks
        if (millis() - start > 10000) break;
        LOG(".");
        delay(500);
    }
    LOG("Modem is online!\n");
    delay(5000);

    LOG("Enter modem power off!\n");
    if (modem.poweroff())
    {
        LOG("Modem power off successful!\n");
    }
    else
    {
        LOG("Modem power off failed!\n");
    }

    delay(5000);

    // Flush serial to ensure all data is sent
    Serial.flush();
    SerialAT.flush();

#ifdef BOARD_POWERON_PIN
    digitalWrite(BOARD_POWERON_PIN, LOW);
#endif

#ifdef MODEM_RESET_PIN
    // Set reset pin low during sleep
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    gpio_hold_en((gpio_num_t)MODEM_RESET_PIN);
    gpio_deep_sleep_hold_en();
#endif

    LOG("Entering deep sleep...\n");
    Serial.flush();

    // Disable watchdog before entering deep sleep
    esp_task_wdt_deinit();

    esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
    delay(200);
    esp_deep_sleep_start();
}

void setup()
{
    Serial.begin(115200); // Set console baud rate

    // reduce CPU frequency to save power while active
    setCpuFrequencyMhz(80);

    LOG("Start Sketch\n");

    SerialAT.begin(115200, SERIAL_8N1, MODEM_RX_PIN, MODEM_TX_PIN);

    esp_task_wdt_init(30, true);  // 30 second watchdog timeout
    esp_task_wdt_add(NULL);

    // Disable WiFi and Bluetooth to save power
    WiFi.mode(WIFI_OFF);
    btStop();

    if (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_TIMER)
    {
        Serial.println("Wakeup from deep sleep - allowing modem extra startup time");
        int i = 15;  // Increased to 15 seconds for full modem stabilization
        while (i > 0)
        {
            Serial.printf("Modem will start in %d seconds\n", i);
            Serial.flush();
            delay(1000);
            i--;
        }
        Serial.println("TurnON Modem!");
    }

    //analogSetAttenuation(ADC_11db);
    //analogReadResolution(12);

#if CONFIG_IDF_TARGET_ESP32
    //analogSetWidth(12);
#endif

#ifdef BOARD_POWERON_PIN
    pinMode(BOARD_POWERON_PIN, OUTPUT);
    digitalWrite(BOARD_POWERON_PIN, HIGH);
    delay(1000);  // Add delay after powering on
#endif

    // Set modem reset pin - reset modem
    pinMode(MODEM_RESET_PIN, OUTPUT);
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(200);  // Increased from 100ms
    digitalWrite(MODEM_RESET_PIN, MODEM_RESET_LEVEL);
    delay(4000);  // Increased from 3000ms - critical for post-sleep recovery
    digitalWrite(MODEM_RESET_PIN, !MODEM_RESET_LEVEL);
    delay(500);  // Add stabilization delay

    pinMode(BOARD_PWRKEY_PIN, OUTPUT);
    digitalWrite(BOARD_PWRKEY_PIN, LOW);
    delay(150);  // Increased from 100ms
    digitalWrite(BOARD_PWRKEY_PIN, HIGH);
    delay(150);  // Increased from 100ms
    digitalWrite(BOARD_PWRKEY_PIN, LOW);

    // Allow extra time for modem to power on
    delay(3000);  // Increased from 2000ms - wait for modem to boot after deep sleep

    // Check if the modem is online
    Serial.println("Start modem...");

    int retry = 0;
    while (!modem.testAT(1000))
    {
        Serial.println(".");
        esp_task_wdt_reset();  // Reset watchdog during modem checks
        if (retry++ > 15)  // Increased retry count
        {
            Serial.println("Modem AT test failed, retrying full reset...");
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(200);
            digitalWrite(BOARD_PWRKEY_PIN, HIGH);
            delay(200);
            digitalWrite(BOARD_PWRKEY_PIN, LOW);
            delay(3000);  // Wait longer after retry
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

    //Dallas_sensors.requestTemperatures(); // Send the command to get temperatures
    //Serial.print("Temperature for the device 1 (index 0) is: ");
    float temp = 0; //Dallas_sensors.getTempCByIndex(0);
    //Serial.println(temp);

    float tempIn = thermocouple.readCelsius();
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

    // Initialize HTTPS client      
    modem.https_begin();

    // Send data only ONCE per cycle
    retry = 3;
    bool sent_successfully = false;

    while (retry-- && !sent_successfully)
    {
        Serial.print("Request URL : ");
        Serial.println(request_url[0]);  // Use first URL only
        
        char str[192];
        sprintf(str, "%s&field1=%i&field2=%.0f&field3=%.0f&field4=%i", 
                request_url[0], 0, temp, tempIn, battery_voltage);

        Serial.print("Request URL with data: ");
        Serial.println(str);

        // Set GET URL
        if (!modem.https_set_url(str))
        {
            Serial.print("Failed to set URL, retry...\n");
            delay(2000);
            continue;
        }

        // Send GET request
        int httpCode = modem.https_get();
        if (httpCode != 200)
        {
            Serial.print("HTTP get failed ! error code = ");
            Serial.println(httpCode);
            delay(2000);
            continue;
        }

        // Get HTTPS header information
        String header = modem.https_header();
        Serial.print("HTTP Header : ");
        Serial.println(header);
        delay(500);

        // Get HTTPS response
        String body = modem.https_body();
        Serial.print("HTTP body : ");
        Serial.println(body);

        sent_successfully = true;  // Mark as sent, exit loop
        delay(1000);
    }

    if (!sent_successfully)
    {
        Serial.println("Failed to send data after 3 retries");
    }

    Serial.println("-------------------------------------");

    // Disable Serial to save power before deep sleep
    Serial.println("Preparing for deep sleep...");
    Serial.flush();
    
    // Disable watchdog before deep sleep to prevent it from triggering during sleep
    esp_task_wdt_deinit();
    
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
