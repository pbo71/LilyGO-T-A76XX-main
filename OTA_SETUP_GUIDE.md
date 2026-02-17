# OTA Firmware Update Setup Guide

## 🚀 Quick Start - GitHub Releases Method

### Step 1: Build og få .bin filen

1. Build dit projekt:
   ```bash
   platformio run
   ```

2. Find din firmware fil her:
   ```
   .pio/build/[your_board]/firmware.bin
   ```

### Step 2: Upload til GitHub Releases

1. Gå til dit GitHub repository: https://github.com/pbo71/LilyGO-T-A76XX-main

2. Klik på "Releases" → "Create a new release"

3. Tag version: `v1.0.0` (første release)

4. Upload **2 filer**:
   - `firmware.bin` - din compiled firmware
   - `version.txt` - en fil med kun tallet: `1`

5. Publish release

### Step 3: Få download URLs

Efter release, højreklik på filerne og få URLs:

```
https://github.com/pbo71/LilyGO-T-A76XX-main/releases/download/v1.0.0/firmware.bin
https://github.com/pbo71/LilyGO-T-A76XX-main/releases/download/v1.0.0/version.txt
```

### Step 4: Opdater koden

Åbn `HttpsBuiltlnGet.ino` og erstat URLs på linje 55-56:

```cpp
#define OTA_FIRMWARE_URL "https://github.com/pbo71/LilyGO-T-A76XX-main/releases/download/v1.0.0/firmware.bin"
#define OTA_VERSION_URL "https://github.com/pbo71/LilyGO-T-A76XX-main/releases/download/v1.0.0/version.txt"
```

### Step 5: Release en opdatering

Når du vil opdatere enheder i marken:

1. **Lav ændringer** i din kode

2. **Opdater version** i koden:
   ```cpp
   #define CURRENT_FIRMWARE_VERSION 2  // Øg til 2
   ```

3. **Build ny firmware**:
   ```bash
   platformio run
   ```

4. **Opret ny GitHub Release**: `v2.0.0`
   - Upload ny `firmware.bin`
   - Upload `version.txt` med indhold: `2`

5. **Opdater URLs** i din kode til v2.0.0

6. Enheder downloader automatisk næste gang de vågner! 🎉

---

## 🌐 Alternative Metoder

### Metode 2: Simpel Python Webserver (Lokalt test)

Perfekt til test før production:

```bash
# Placer firmware i en mappe
cd firmware_folder
python -m http.server 8000

# Brug ngrok til at expose lokalt:
ngrok http 8000

# Brug ngrok HTTPS URL i koden
```

### Metode 3: Firebase Storage (Gratis tier)

1. Opret Firebase projekt
2. Upload filer til Storage
3. Lav filer public eller brug signed URLs
4. URL format:
   ```
   https://firebasestorage.googleapis.com/v0/b/[bucket]/o/firmware.bin?alt=media
   ```

### Metode 4: AWS S3 (Professional)

1. Opret S3 bucket
2. Upload filer
3. Sæt bucket policy til public read
4. Brug CloudFront for HTTPS

---

## 📝 version.txt Format

Filen skal BARE indeholde versionsnummeret:

```
2
```

Ingen newlines, ingen ekstra tekst.

---

## 🔒 Sikkerhedstips

1. ✅ Brug ALTID HTTPS (aldrig HTTP)
2. ✅ Test firmware lokalt først
3. ✅ Start med små version increments
4. ✅ Hold backup af alle firmware versioner
5. ✅ Log OTA events til ThingSpeak for monitoring

---

## 🐛 Troubleshooting

### "Version check failed, HTTP code: 404"
- Tjek at URLs er korrekte
- Verificer at filer er public accessible
- Test URL i browser først

### "Invalid firmware size"
- Tjek at .bin filen er uploaded korrekt
- Max størrelse er 2MB (defineret i kode)

### "Not enough space for OTA update"
- ESP32 partition skal have plads til ny firmware
- Tjek platformio.ini partition scheme

### "Battery too low for OTA"
- OTA kræver minimum 3.8V
- Lad enheden oplade først

---

## 📊 Monitor OTA Process

Tilføj logging til ThingSpeak:

```cpp
// Efter succesfuld OTA
char ota_log[64];
snprintf(ota_log, 64, "OTA: v%d -> v%d", old_version, new_version);
// Send til ThingSpeak eller SMS
```

---

## 🎯 Production Workflow

```
1. Develop → Test lokalt
2. Build firmware
3. Tag version i Git
4. Create GitHub Release
5. Upload firmware + version.txt
6. Verify URLs in browser
7. Monitor første enheder
8. Full rollout
```

---

## 📱 Test Checklist

- [ ] Build firmware successfully
- [ ] Upload to GitHub Release
- [ ] Test URL i browser (skal downloade fil)
- [ ] Verify version.txt indhold
- [ ] Test på én enhed først
- [ ] Monitor serial output under OTA
- [ ] Verify device reboots med ny version
- [ ] Check normal operation efter OTA

