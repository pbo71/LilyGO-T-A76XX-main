// XIAO nRF52840 BLE temperature sender
//
// Measures a DS18B20 every MEASURE_INTERVAL_MS and broadcasts the value as a BLE advertisement.
// There is no connection or pairing: a receiver (the A7670 board, or nRF Connect on a phone)
// just listens for the advertisement, checks the name and format version, and reads the
// manufacturer data. The company ID 0xFFFF is shared by other test devices, so the name matters.
//
// Manufacturer data (little endian), 9 bytes:
//   [0..1] company ID 0xFFFF (reserved for testing)
//   [2]    format version (1)
//   [3..4] counter, +1 per measurement, so a receiver can see the value is fresh
//   [5..6] temperature in 1/100 C (int16), 0x7FFF = no sensor
//   [7..8] battery in mV (uint16)
//
// Long range: by default the advertisement goes out on the Coded PHY (BLE 5 long range, about 4x the
// range of 1M) as a non-connectable extended advertisement with the name and data in one packet. Only
// BLE 5 Coded PHY receivers hear it (the ESP32-S3 board, many newer phones); the classic ESP32 A7670
// board does not. Build with -D ADV_LEGACY to send the old 1M legacy advertisement instead.
//
// Wiring: DS18B20 VDD -> 3V3, GND -> GND, DATA -> D2, 4.7 kOhm between DATA and 3V3.

#include <Arduino.h>
#include <Adafruit_TinyUSB.h>  // Needed for USB Serial with the Adafruit based nRF52 core
#include <bluefruit.h>
#include <OneWire.h>
#include <DallasTemperature.h>

// Each module gets its own name, so a receiver only uses its own module.
// Override per module with -D DEVICE_NAME=\"...\" in platformio.ini.
#ifndef DEVICE_NAME
#define DEVICE_NAME "TestTempBTModule"
#endif
#define DS18B20_PIN D2
#define PIN_HICHG 22  // P0.13 on the BQ25100 charger: LOW = 100 mA, HIGH = 50 mA
#define PIN_CHG 23    // P0.17 ~CHG from the BQ25100: LOW while charging (green LED on)
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

#ifndef ADV_LEGACY
// Bluefruit always advertises on 1M, so the Coded PHY set is configured on the SoftDevice directly.
// The SoftDevice reads the buffer while advertising, so it is static and only changed while stopped.
static uint8_t advData[64];
static uint8_t advHandle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
static bool advRunning = false;

void startAdvertising(const Payload &p)
{
    if (advRunning) {
        sd_ble_gap_adv_stop(advHandle);
        advRunning = false;
    }

    uint8_t len = 0;
    advData[len++] = 2;  // Flags
    advData[len++] = BLE_GAP_AD_TYPE_FLAGS;
    advData[len++] = BLE_GAP_ADV_FLAG_BR_EDR_NOT_SUPPORTED;
    const uint8_t nameLen = strlen(DEVICE_NAME);
    advData[len++] = nameLen + 1;  // Complete local name
    advData[len++] = BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME;
    memcpy(&advData[len], DEVICE_NAME, nameLen);
    len += nameLen;
    advData[len++] = sizeof(p) + 1;  // Manufacturer data
    advData[len++] = BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA;
    memcpy(&advData[len], &p, sizeof(p));
    len += sizeof(p);

    static ble_gap_adv_data_t data;
    data.adv_data.p_data = advData;
    data.adv_data.len = len;
    data.scan_rsp_data.p_data = NULL;
    data.scan_rsp_data.len = 0;

    ble_gap_adv_params_t params;
    memset(&params, 0, sizeof(params));
    params.properties.type = BLE_GAP_ADV_TYPE_EXTENDED_NONCONNECTABLE_NONSCANNABLE_UNDIRECTED;
    params.interval = ADV_INTERVAL_UNITS;
    params.duration = 0;  // Until stopped
    params.filter_policy = BLE_GAP_ADV_FP_ANY;
    params.primary_phy = BLE_GAP_PHY_CODED;
    params.secondary_phy = BLE_GAP_PHY_CODED;

    uint32_t err = sd_ble_gap_adv_set_configure(&advHandle, &data, &params);
    if (err == NRF_SUCCESS) err = sd_ble_gap_tx_power_set(BLE_GAP_TX_POWER_ROLE_ADV, advHandle, TX_POWER_DBM);
    if (err == NRF_SUCCESS) err = sd_ble_gap_adv_start(advHandle, BLE_CONN_CFG_TAG_DEFAULT);
    if (err != NRF_SUCCESS) {
        Serial.printf("Coded PHY advertising failed: error 0x%lX\n", (unsigned long)err);
        return;
    }
    advRunning = true;
}
#else
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
#endif

void measureAndAdvertise()
{
    Payload p;
    p.company = COMPANY_ID;
    p.version = FORMAT_VERSION;
    p.counter = ++counter;
    p.temp_centi = readTemperatureCenti();
    p.battery_mv = readBatteryMilliVolts();

    const char *charge = digitalRead(PIN_CHG) == LOW ? "charging" : "not charging";
    if (p.temp_centi == TEMP_INVALID) {
        Serial.printf("#%u  temp: no sensor  battery: %u mV (%s)\n", p.counter, p.battery_mv, charge);
    } else {
        Serial.printf("#%u  temp: %.2f C  battery: %u mV (%s)\n", p.counter, p.temp_centi / 100.0f, p.battery_mv,
                      charge);
    }
    startAdvertising(p);
}

void setup()
{
    Serial.begin(115200);
    delay(1500);  // Give USB serial a moment, so the first lines are not lost

    pinMode(VBAT_ENABLE, OUTPUT);
    digitalWrite(VBAT_ENABLE, LOW);

    // Charge with 100 mA instead of the default 50 mA. Charging only happens on USB indoors.
    pinMode(PIN_HICHG, OUTPUT);
    digitalWrite(PIN_HICHG, LOW);
    pinMode(PIN_CHG, INPUT_PULLUP);

    sensors.begin();
    sensors.setResolution(12);
#ifdef ADV_LEGACY
    const char *phy = "1M legacy";
#else
    const char *phy = "Coded PHY (long range)";
#endif
    Serial.printf("XIAO BLE sender %s, %s, %d DS18B20 found\n", DEVICE_NAME, phy, sensors.getDeviceCount());

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
