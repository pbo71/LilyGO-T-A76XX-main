// XIAO nRF52840 BLE temperature sender
//
// Measures a DS18B20 every MEASURE_INTERVAL_MS and broadcasts the value as a BLE advertisement.
// There is no connection or pairing: a receiver (the A7670 board, or nRF Connect on a phone)
// just listens for the advertisement and reads the manufacturer data.
//
// Manufacturer data (little endian), 9 bytes:
//   [0..1] company ID 0xFFFF (reserved for testing)
//   [2]    format version (1)
//   [3..4] counter, +1 per measurement, so a receiver can see the value is fresh
//   [5..6] temperature in 1/100 C (int16), 0x7FFF = no sensor
//   [7..8] battery in mV (uint16)
//
// Wiring: DS18B20 VDD -> 3V3, GND -> GND, DATA -> D2, 4.7 kOhm between DATA and 3V3.

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // Needed for USB Serial with the Adafruit based nRF52 core
#include <bluefruit.h>
#include <OneWire.h>
#include <DallasTemperature.h>

#define DEVICE_NAME "SoenTemp"
#define DS18B20_PIN D2
#define MEASURE_INTERVAL_MS (5UL * 60 * 1000)  // New measurement every 5 minutes
#define ADV_INTERVAL_UNITS 1600                // Advertise every 1 s (units of 0.625 ms)
#define TX_POWER_DBM 8                         // Maximum for the nRF52840, for range
#define TEMP_INVALID 0x7FFF
#define COMPANY_ID 0xFFFF
#define FORMAT_VERSION 1

OneWire oneWire(DS18B20_PIN);
DallasTemperature sensors(&oneWire);

uint16_t counter = 0;

#pragma pack(push, 1)
struct Payload {
    uint16_t company;
    uint8_t version;
    uint16_t counter;
    int16_t temp_centi;
    uint16_t battery_mv;
};
#pragma pack(pop)

// Battery voltage through the on-board divider (1 M / 510 k) on PIN_VBAT.
// VBAT_ENABLE must be LOW to read, and Seeed warns it must never be driven HIGH with a battery
// connected (the ADC pin can then see the full battery voltage). It is kept LOW all the time.
uint16_t readBatteryMilliVolts()
{
    analogReference(AR_INTERNAL_3_0);  // 3.0 V full scale
    analogReadResolution(12);
    uint32_t sum = 0;
    for (int i = 0; i < 8; i++) {
        sum += analogRead(PIN_VBAT);
        delay(2);
    }
    float mv = (sum / 8.0f) * 3000.0f / 4096.0f * (1510.0f / 510.0f);
    return (uint16_t)mv;
}

int16_t readTemperatureCenti()
{
    sensors.requestTemperatures();  // Blocks ~750 ms at 12 bit
    float c = sensors.getTempCByIndex(0);
    if (c == DEVICE_DISCONNECTED_C || c == 85.0f || c < -55.0f || c > 125.0f) {
        return TEMP_INVALID;  // 85.0 is the power-on value, not a real reading
    }
    return (int16_t)lroundf(c * 100.0f);
}

void startAdvertising(const Payload &p)
{
    Bluefruit.Advertising.stop();
    Bluefruit.Advertising.clearData();
    Bluefruit.ScanResponse.clearData();
    Bluefruit.Advertising.addFlags(BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE);
    Bluefruit.Advertising.addManufacturerData(&p, sizeof(p));
    Bluefruit.ScanResponse.addName();  // Name in the scan response, so nRF Connect shows it
    Bluefruit.Advertising.setType(BLE_GAP_ADV_TYPE_NONCONNECTABLE_SCANNABLE_UNDIRECTED);
    Bluefruit.Advertising.setInterval(ADV_INTERVAL_UNITS, ADV_INTERVAL_UNITS);
    Bluefruit.Advertising.restartOnDisconnect(false);
    Bluefruit.Advertising.start(0);  // 0 = advertise until stopped
}

void measureAndAdvertise()
{
    Payload p;
    p.company = COMPANY_ID;
    p.version = FORMAT_VERSION;
    p.counter = ++counter;
    p.temp_centi = readTemperatureCenti();
    p.battery_mv = readBatteryMilliVolts();

    if (p.temp_centi == TEMP_INVALID) {
        Serial.printf("#%u  temp: no sensor  battery: %u mV\n", p.counter, p.battery_mv);
    } else {
        Serial.printf("#%u  temp: %.2f C  battery: %u mV\n", p.counter, p.temp_centi / 100.0f, p.battery_mv);
    }
    startAdvertising(p);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);  // Give USB serial a moment, so the first lines are not lost

    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    sensors.begin();
    sensors.setResolution(12);
    Serial.printf("XIAO BLE sender, %d DS18B20 found\n", sensors.getDeviceCount());

    Bluefruit.begin();
    Bluefruit.setTxPower(TX_POWER_DBM);
    Bluefruit.setName(DEVICE_NAME);
    Bluefruit.autoConnLed(false);  // No LED, saves power

    measureAndAdvertise();
}

void loop()
{
    // delay() lets the SoftDevice sleep the CPU between advertisements
    delay(MEASURE_INTERVAL_MS);
    measureAndAdvertise();
}
