# OTA-opdatering af A7670ETest

Enheden henter ny firmware over mobilnettet fra et offentligt GitHub-repo, der kun indeholder firmwarefiler. Telefonnummer og ThingSpeak-nøgle ligger ikke i firmwaren. De gemmes én gang i enhedens flash-hukommelse (NVS), så firmwaren kan ligge offentligt.

## Sådan virker det

1. **Tjek:** Efter upload til ThingSpeak tjekker enheden én gang i døgnet og ved koldstart:
   ```
   https://raw.githubusercontent.com/pbo71/esp-firmware/main/a7670e/version.txt
   ```
   Det sker kun, når batteriet er over 4,0 V eller enheden kører på USB.
2. **Manifest:** `version.txt` indeholder `<version> <størrelse> <md5>`. Er versionen højere end `FIRMWARE_VERSION`, hentes `firmware.bin` i bidder på 32 KB med HTTP Range-forespørgsler.
3. **Kontrol:** Når filen er hentet, kontrolleres MD5. Først derefter skrives firmwaren til den ledige app-partition. Ved en fejl kører den gamle firmware videre.
4. **Første kørsel:** Enheden genstarter med det samme ind i den nye firmware. En opvågning fra deep sleep ville starte den gamle firmware igen, fordi bootloaderen er bygget med `CONFIG_BOOTLOADER_SKIP_VALIDATE_IN_DEEP_SLEEP`. Den nye firmware skal nå at registrere sig på netværket i første kørsel, også i stille-perioden. Ellers starter bootloaderen automatisk den gamle version igen.
5. **Rollback:** Når en version er rullet tilbage, husker enheden det og installerer ikke den version igen. Ret fejlen og udgiv en ny, højere version.

## Første opsætning

### 1. Opret firmware-repoet

Opret et nyt **offentligt** repo på GitHub, `pbo71/esp-firmware`, med en README, så der findes en `main`-branch. Klon det lokalt, f.eks. ved siden af dette repo:

```bash
git clone https://github.com/pbo71/esp-firmware.git ../esp-firmware
```

Bruger du et andet navn eller en anden branch, skal du rette `OTA_BASE_URL` i `HttpsBuiltlnGet.ino`.

### 2. Skriv hemmelighederne til enheden

Udfyld `examples/A7670ETest/secrets.h` ud fra `secrets.h.example`, og flash provision-buildet via USB én gang:

```bash
pio run -e T-A7670X-provision -t upload -t monitor
```

I serielmonitoren skal der stå `PROVISION: secrets written to NVS`. Bagefter kan du flashe det almindelige build, `T-A7670X`, så meget du vil. Hemmelighederne bliver liggende i NVS.

Provision-buildet indeholder hemmelighederne og må **aldrig** udgives. `tools/ota_publish.py` nægter at udgive det.

### 3. Prøvekørsel (henter kun, flasher ikke)

`OTA_DRY_RUN` står som standard på `1`. Så henter enheden filen og tjekker MD5, men skriver intet.

1. Enheden kører version 1. Den har du lige flashet i trin 2.
2. Sæt `FIRMWARE_VERSION` til `2`, byg og udgiv:
   ```bash
   pio run -e T-A7670X
   python tools/ota_publish.py ../esp-firmware
   git -C ../esp-firmware add a7670e
   git -C ../esp-firmware commit -m "Firmware version 2"
   git -C ../esp-firmware push
   ```
3. Tryk på reset, så enheden laver en koldstart og tjekker med det samme. I loggen skal der stå:
   ```
   OTA: downloading version 2 (... bytes) - DRY RUN
   OTA: 10% ...
   OTA: DRY RUN - download and MD5 OK, nothing written
   ```

### 4. Første rigtige opdatering

1. Sæt `OTA_DRY_RUN` til `0`, og lad `FIRMWARE_VERSION` stå på `2`. Flash via USB med `pio run -e T-A7670X -t upload`.
2. Sæt `FIRMWARE_VERSION` til `3`, byg og udgiv som i trin 3.
3. Tryk på reset. Loggen viser `OTA: version 3 installed, restarting into it`.
4. Lige efter genstarten viser loggen `Firmware version: 3 (new, waiting for confirmation)` og derefter `OTA: firmware version 3 confirmed`.

## En ny release

1. Lav dine ændringer, og hæv `FIRMWARE_VERSION`.
2. Kør `pio run -e T-A7670X` og `python tools/ota_publish.py ../esp-firmware`.
3. Commit og push i `esp-firmware`.

Enhederne opdaterer ved deres næste daglige tjek. `raw.githubusercontent.com` kan cache en fil i nogle minutter efter et push.

## Fejlsøgning

| Log | Betydning |
|---|---|
| `OTA: version.txt HTTP 404` | Repo, sti eller branch passer ikke med `OTA_BASE_URL`, eller repoet er ikke offentligt. |
| `OTA: chunk at ...: HTTP 200` | Serveren ignorerer Range. GitHub raw understøtter det, så tjek URL'en. |
| `OTA: MD5 mismatch` | Filen og `version.txt` passer ikke sammen. Udgiv igen med scriptet. |
| `OTA: battery ...mV below 4000mV` | Tjekket udskydes, til batteriet er højere. |
| `OTA: version N was rolled back` | Den nye firmware nåede ikke netværket i første kørsel. Ret fejlen, og udgiv N+1. |
| `WARNING: secrets missing in NVS` | Provision-buildet er ikke flashet (trin 2). |

## Begrænsninger i dette koncept

- **Kilden kontrolleres ikke:** TLS kører uden certifikatkontrol (`authmode` 0), og MD5 kommer fra samme server som firmwaren. MD5 beskytter mod en ødelagt download, men ikke mod en, der kan ændre dit firmware-repo eller sidder midt i forbindelsen. Det kan løses senere med signeret firmware eller et CA-certifikat i modemmet.
- **Dataforbrug:** En opdatering bruger cirka 1 MB data. Det daglige tjek bruger under 1 KB.
- **Plads:** En app-partition er 1,25 MB. Firmwaren fylder cirka 1,06 MB.
