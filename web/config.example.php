<?php
// Copy this file to config.php and fill in the values. config.php is never committed to git.
return [
    // MySQL database from the web hotel control panel
    'db_host' => 'localhost',
    'db_name' => '',
    'db_user' => '',
    'db_pass' => '',

    // ThingSpeak channel to copy. The read key is only needed for a private channel
    // (ThingSpeak: Channel -> API Keys -> Read API Key). It stays on the server.
    'channel_id' => 0,  // The number without leading zeros (PHP reads 0123 as octal)
    'read_key' => '',

    // More channels to copy, each with its own page: channel ID => read key ('' when public)
    'more_channels' => [],

    // Secret for running fetch.php as a URL (cron "call URL"): fetch.php?token=...
    // Use a long random string. Not needed when cron runs php fetch.php directly.
    'cron_token' => '',
];
