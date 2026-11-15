-- Database tables for the measurement website. Run once in phpMyAdmin (or the web hotel's SQL tool).

-- One row per ThingSpeak entry. Times are UTC; the page shows Danish time.
CREATE TABLE IF NOT EXISTS measurements (
    channel_id INT UNSIGNED NOT NULL,
    entry_id INT UNSIGNED NOT NULL,
    created_at DATETIME NOT NULL,
    field1 DOUBLE NULL,
    field2 DOUBLE NULL,
    field3 DOUBLE NULL,
    field4 DOUBLE NULL,
    field5 DOUBLE NULL,
    field6 DOUBLE NULL,
    field7 DOUBLE NULL,
    field8 DOUBLE NULL,
    PRIMARY KEY (channel_id, entry_id),
    KEY idx_time (channel_id, created_at)
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;

-- Channel name and field names as set in ThingSpeak, refreshed by fetch.php
CREATE TABLE IF NOT EXISTS channels (
    channel_id INT UNSIGNED NOT NULL PRIMARY KEY,
    name VARCHAR(255) NOT NULL,
    field_names TEXT NOT NULL,
    updated_at DATETIME NOT NULL
) ENGINE=InnoDB DEFAULT CHARSET=utf8mb4;
