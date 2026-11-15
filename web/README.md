# Målinger on iot.boegebjerg.eu

A public page with the measurements from a ThingSpeak channel. A cron job copies new entries
into MySQL every 10 minutes and the page reads from the database. The ThingSpeak read key stays
on the server.

| File | Purpose |
|---|---|
| index.html | The page (latest values and charts, Danish time) |
| api.php | JSON for the page |
| fetch.php | Copies new ThingSpeak entries into the database (cron) |
| db.php | Shared database code |
| config.example.php | Template for config.php (database, channel, keys) |
| schema.sql | Creates the tables |
| .htaccess | Blocks config.php, db.php and schema.sql from the web |

## Setup (cPanel, Nordicway)

1. **Database:** cPanel -> *MySQL Databases*: create a database and a user, and give the user
   all privileges on it. cPanel puts your account name in front of both names.
2. **Tables:** cPanel -> *phpMyAdmin*: select the database, open *SQL*, paste schema.sql and run it.
3. **Config:** copy config.example.php to config.php and fill in the database values, the
   channel ID and (for a private channel) the read API key.
4. **Upload:** cPanel -> *File Manager*: upload all files including .htaccess and config.php
   directly into the subdomain folder /home/boegebje/iot.boegebjerg.eu (next to cgi-bin and
   php.ini, which belong to the web hotel and stay as they are).
5. **First fetch:** cPanel -> *Cron Jobs*, or run it once by hand (see below). The first run copies
   up to the last 8000 entries.
6. **Cron:** cPanel -> *Cron Jobs*, every 10 minutes (`*/10 * * * *`):
   `/usr/local/bin/php /home/boegebje/iot.boegebjerg.eu/fetch.php >/dev/null 2>&1`
7. Open https://iot.boegebjerg.eu/

## Checks

- https://iot.boegebjerg.eu/config.php must answer *403 Forbidden*.
- If cron does not run PHP as CLI, fetch.php answers *Forbidden*. Then set a long random
  `cron_token` in config.php and let cron call the URL instead:
  `curl -s "https://iot.boegebjerg.eu/fetch.php?token=..." >/dev/null`
