# Plan: vandtemperatur ved søen over LoRa

Et levende dokument. Ret det til, efterhånden som tests og beslutninger falder på plads.

## Mål

Måle vandtemperaturen i søen ca. 99 m fra hytten (delvis gennem træer) og få den med i de timelige
uploads fra hyttens board. Stabilitet er vigtigere end hyppighed, fordi vandtemperaturen ændrer sig langsomt.

## Arkitektur

```
 Søen (stolpe)                       Hytten
┌──────────────────┐   LoRa 868   ┌───────────────┐  WAKE+UART+GND  ┌──────────────┐  LTE
│ XIAO ESP32S3     │ ───────────► │ Bro: XIAO     │ ◄─────────────► │ S3-board     │ ────► ThingSpeak
│ + SX1262         │ ◄─────────── │ ESP32S3+SX1262│   (1 kΩ serie)  │ (SIM7670G)   │
│ DS18B20, batteri │     ACK      │ eget batteri  │                 │ eget batteri │
└──────────────────┘              └───────────────┘                 └──────────────┘
```

- **Sender (søen):** sover i dyb dvale. Vågner ca. hvert 30. minut, måler, sender en LoRa-pakke og venter
  på kvittering (ACK). Sender igen op til 3 gange med tilfældig ventetid, hvis ACK mangler.
- **Bro (hytten):** SX1262 lytter i intervaller (RX duty cycle). ESP32 sover og vækkes af DIO1, når en pakke
  kommer, gemmer målingen og kvitterer. Vækkes også af WAKE fra S3-boardet og svarer over UART.
- **S3-board (hytten):** sætter WAKE høj ved hver timelig opvågning, læser den seneste måling over UART og
  uploader den sammen med de øvrige felter.

LoRa punkt-til-punkt (rå LoRa med RadioLib), ikke LoRaWAN: ingen gateway, netværksserver eller operatør.

## Hardware

| Enhed | Hardware | Status |
|---|---|---|
| Sender | XIAO ESP32S3 & Wio-SX1262 kit (Seeed) | Bestilt 2026-11-21 |
| Bro | XIAO ESP32S3 & Wio-SX1262 kit (Seeed) | Bestilt 2026-11-21 |
| Hyttens board | LilyGO T-SIM7670G-S3-Standard, `s3/S3Firmware` | Kører (test i soveværelset) |
| Bluetooth-test | XIAO nRF52840, `xiao/XiaoBleSender` | Kører i skuret (Coded PHY) |

Hvert kit har XIAO ESP32S3, Wio-SX1262, LoRa-antenne og 2,4 GHz-antenne og leveres med Meshtastic.

### Forbindelse bro ↔ S3-board

| Signal | Retning | Note |
|---|---|---|
| WAKE | S3 → bro | Vækker broen fra dyb dvale (ext0) |
| TX/RX | begge veje | UART, fx 9600 eller 115200 baud |
| GND | fælles | Fælles stel, hver enhed har sit eget batteri |

1 kΩ i serie på hver signalledning, så en enhed uden strøm ikke fødes gennem beskyttelsesdioderne.
Pins på S3-boardet vælges blandt de ledige på stiftrækken (kamerapins).

## Protokol (udkast)

**Sender → bro:** formatversion, tæller, temperatur (1/100 °C), batteri (mV), antal gensendinger.
**Bro → sender (ACK):** tæller, RSSI og SNR for den modtagne pakke.
**Bro → S3 (UART):** temperatur, batteri, målingens alder, LoRa RSSI/SNR. Broen sender kun en værdi, der er
under ca. 90 minutter gammel. Ellers sender den "ingen data", så en gammel værdi ikke uploades som ny.

Radioindstillinger (åbne): frekvens, SF10-SF12, båndbredde, sendestyrke.

## Strøm

| Enhed | Forventet gennemsnit | Kommentar |
|---|---|---|
| Sender | ca. 0,1-0,2 mA | Dyb dvale, kort opvågning hvert 30. min. Skal måles |
| Bro, altid vågen | ca. 5-8 mA | Kun til de første tests |
| Bro, optimeret | ca. 1 mA eller mindre | RX duty cycle + dyb dvale + WAKE. Skal måles |

Estimater, ikke målinger. Mål med en strømprofiler (fx PPK2).

## Regler for 868 MHz (EU)

- 868,0-868,6 MHz: højst 1 % sendetid, 25 mW ERP.
- 869,4-869,65 MHz: højst 10 % sendetid, 500 mW ERP.

Én pakke hvert 30. minut ligger langt under begge grænser.

## Faser

### 1. Bluetooth i haven (i gang)
- [x] XIAO nRF52840 sender Coded PHY, S3 modtager (bcf425e)
- [x] S3 i soveværelset hører skuret: ca. -89 til -94 dBm
- [x] XIAO flyttet væk fra ståltaget (sammenlign RSSI)
- [ ] XIAO i 3D-printet PLA-kasse på stolpe udenfor
- [ ] Følg batteriets fald i kulde

### 2. Meshtastic-test, når kittene kommer
- [ ] Forbind iPhone (Meshtastic-app) til et kit
- [ ] Gåtur med det ene kit, aflæs RSSI/SNR på afstand (over 100 m med huse/træer)

### 3. Egen firmware
- [ ] Tjek diagrammet: SX1262-pins på ESP32S3-kittet og ledig pin til DS18B20
- [ ] Spændingsdeler til batterimåling på senderen (XIAO ESP32S3 kan ikke selv måle)
- [ ] PlatformIO-projekt til senderen (dyb dvale, DS18B20, LoRa, ACK, gensendelse)
- [ ] PlatformIO-projekt til broen, først den simple version (altid vågen)
- [ ] S3: WAKE + UART, nye felter (navn i NVS / feltplan, se åbne spørgsmål)
- [ ] Watchdog, håndtering af manglende føler, beskyttelse mod lav spænding

### 4. Langtidstest i haven
- [ ] Senderen i kasse på stolpe, føleren i en spand vand
- [ ] Uger uden huller i kurven
- [ ] Mål strømforbruget på sender og bro
- [ ] Margintest: sænk sendestyrken (fx 0 dBm) og se, om broen stadig hører
- [ ] Optimer broen (RX duty cycle, dyb dvale) og mål igen

### 5. Søen
- [ ] Kasse printet i ASA eller PETG, pakning i låget, drænhul, silicagel
- [ ] Stolpe i søen: antennen mindst 1 m over vandet, kassen vender mod hytten
- [ ] Broen i et vindue mod søen
- [ ] Hyttens board skiftes fra den gamle A7670 til et S3-board

## Åbne spørgsmål

- Hvilke ThingSpeak-felter skal vandtemperatur, senderens batteri og LoRa-signal have? Alle 8 er brugt
  på testkanalen (field6-8 er Bluetooth i dag).
- Sendestyrke og SF: fast, eller lad senderen justere efter RSSI/SNR fra ACK?
- Strøm til broen ved søen: 18650, solpanel eller powerbank (pas på auto-sluk ved lav strøm).
- Faste tidsvinduer (synkroniseret via ACK) for at spare mere på broen?
- Batterikapacitet for senderen i frost.

## Beslutninger

| Dato | Beslutning |
|---|---|
| 2026-11-21 | Bluetooth Coded PHY giver for lidt margin til 99 m gennem træer. LoRa til søen |
| 2026-11-21 | To XIAO ESP32S3 & Wio-SX1262 kit bestilt |
| 2026-11-21 | Test i haven først. Søen er ca. 200 km væk |
| 2026-11-21 | Måling ca. hvert 30. minut, stabilitet frem for hyppighed |
| 2026-11-21 | Broen har sit eget batteri |
| 2026-11-21 | Ledning mellem bro og S3-board: WAKE + UART + GND, ingen Bluetooth imellem |

## Erfaringer

- Den gamle logger ved søen brugte Sigfox og var ustabil. LTE har været meget bedre.
- Coded PHY ændrer ikke RSSI, kun hvor svagt et signal der kan modtages (ca. -105 mod -97 dBm på 1M).
- iPhone (iPhone 13, nRF Connect) kan ikke se Coded PHY.
- Metal tæt på antennen (ståltag 25 cm væk) koster signal.
- PLA er fint til en vintertest. Til flere år udendørs bruges ASA eller PETG.
