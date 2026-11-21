// Pins of the LilyGO T-SIM7670G-S3-Standard, from the upstream LilyGO-Modem-Series
// utilities.h (LILYGO_SIM7670G_S3_STAN). Only this board, so no board selection is needed.
#pragma once

#define MODEM_BAUDRATE (115200)

// Default I2C pins (on the pin header)
#define BOARD_SDA_PIN (3)
#define BOARD_SCL_PIN (2)

#define MODEM_DTR_PIN (7)
#define MODEM_TX_PIN (4)
#define MODEM_RX_PIN (5)
#define MODEM_RING_PIN (6)

// GPS NMEA output of the modem
#define MODEM_GPS_RX_PIN (48)
#define MODEM_GPS_TX_PIN (45)
#define MODEM_GPS_PPS_PIN (17)

// The modem boot pin needs to follow the startup sequence. There is no modem reset pin.
#define BOARD_PWRKEY_PIN (46)
#define MODEM_POWERON_PULSE_MS (100)  // SIM7670G power-on pulse (upstream value); 2.5 s or more switches it off
#define MODEM_POWEROFF_PULSE_MS (3000)  // SIM7670G power-off pulse (upstream value)

// Connected to voltage dividers (half the voltage); cannot be used for anything else
#define BOARD_BAT_ADC_PIN (8)
#define BOARD_SOLAR_ADC_PIN (18)

// SD card socket
#define BOARD_MISO_PIN (13)
#define BOARD_MOSI_PIN (11)
#define BOARD_SCK_PIN (12)
#define BOARD_SD_CS_PIN (10)

// Power save mode of the board supply (1 disabled, 0 enabled). Not used yet.
#define BOARD_POWER_SAVE_MODE_PIN (42)

// Camera pins, also on the pin header: 21, 1, 41, 9, 14, 37, 40, 39, 38, 36, 16, 47, 15, 35.
// Free for sensors as long as no camera is fitted.

#define SerialAT Serial1

// SIM7670G: the local TinyGSM fork drives it with the SIM7672 driver (same AT commands)
#ifndef TINY_GSM_MODEM_SIM7672
#define TINY_GSM_MODEM_SIM7672
#endif

// GPS antenna power is controlled by a modem GPIO
#define MODEM_GPS_ENABLE_GPIO (1)
#define MODEM_GPS_ENABLE_LEVEL (1)

#define PRODUCT_MODEL_NAME "LilyGo-SIM7670G-S3-Standard"
