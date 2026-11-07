"""Prepare an OTA release of examples/A7670ETest for the firmware repository.

Usage (from the repository root, after `pio run -e T-A7670X`):

    python tools/ota_publish.py <path to local clone of the firmware repo>

The script copies .pio/build/T-A7670X/firmware.bin to <clone>/a7670e/firmware.bin and
writes <clone>/a7670e/version.txt as "<version> <size> <md5>". Commit and push the clone
afterwards; devices pick the release up at their next daily OTA check.
"""
import hashlib
import os
import re
import shutil
import sys

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SKETCH = os.path.join(ROOT, 'examples', 'A7670ETest', 'HttpsBuiltlnGet.ino')
SECRETS = os.path.join(ROOT, 'examples', 'A7670ETest', 'secrets.h')
BIN = os.path.join(ROOT, '.pio', 'build', 'T-A7670X', 'firmware.bin')
TARGET_DIR = 'a7670e'


def fail(msg):
    print('ERROR: ' + msg)
    sys.exit(1)


def main():
    if len(sys.argv) != 2:
        fail('usage: python tools/ota_publish.py <path to firmware repo clone>')
    clone = os.path.abspath(sys.argv[1])
    if not os.path.isdir(os.path.join(clone, '.git')):
        fail(clone + ' is not a git clone')

    sketch = open(SKETCH, encoding='utf-8').read()
    m = re.search(r'^#define FIRMWARE_VERSION (\d+)', sketch, re.M)
    if not m:
        fail('FIRMWARE_VERSION not found in ' + SKETCH)
    version = int(m.group(1))
    m = re.search(r'^#define OTA_DRY_RUN (\d)', sketch, re.M)
    if m and m.group(1) != '0':
        print('WARNING: OTA_DRY_RUN is 1 in this build, so it will not install later updates itself')

    if not os.path.isfile(BIN):
        fail(BIN + ' not found, run: pio run -e T-A7670X')
    if os.path.getmtime(BIN) < os.path.getmtime(SKETCH):
        fail('firmware.bin is older than the sketch, rebuild with: pio run -e T-A7670X')

    data = open(BIN, 'rb').read()
    # Refuse anything that contains secrets: OTA firmware is published publicly
    if b'PROVISION: secrets written to NVS' in data:
        fail('this is a provision build, it must never be published')
    if os.path.isfile(SECRETS):
        for value in re.findall(r'^#define \w+ "([^"]+)"', open(SECRETS, encoding='utf-8').read(), re.M):
            if value.encode() in data:
                fail('firmware.bin contains a value from secrets.h, not publishing')

    out_dir = os.path.join(clone, TARGET_DIR)
    manifest = os.path.join(out_dir, 'version.txt')
    if os.path.isfile(manifest):
        old = open(manifest, encoding='utf-8').read().split()
        if old and old[0].isdigit() and int(old[0]) >= version:
            fail('published version is %s, raise FIRMWARE_VERSION above it first' % old[0])

    os.makedirs(out_dir, exist_ok=True)
    shutil.copyfile(BIN, os.path.join(out_dir, 'firmware.bin'))
    md5 = hashlib.md5(data).hexdigest()
    with open(manifest, 'w', encoding='ascii', newline='\n') as f:
        f.write('%d %d %s\n' % (version, len(data), md5))

    print('Prepared version %d (%d bytes, md5 %s) in %s' % (version, len(data), md5, out_dir))
    print('Publish it with:')
    print('  git -C "%s" add %s' % (clone, TARGET_DIR))
    print('  git -C "%s" commit -m "Firmware version %d"' % (clone, version))
    print('  git -C "%s" push' % clone)


if __name__ == '__main__':
    main()
