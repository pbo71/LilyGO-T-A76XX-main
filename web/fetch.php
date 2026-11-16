<?php
// Copies new entries from ThingSpeak into the database. Run it from cron every 10 minutes:
//   php /path/to/fetch.php                      (cron runs a command)
//   https://boegebjerg.eu/.../fetch.php?token=  (cron calls a URL, token from config.php)
// It only asks for entries newer than the latest one stored, so it is cheap to run often.

require __DIR__ . '/db.php';
$cfg = load_config();

if (PHP_SAPI !== 'cli') {
    $token = $_GET['token'] ?? '';
    if ($cfg['cron_token'] === '' || !hash_equals($cfg['cron_token'], $token)) {
        http_response_code(403);
        exit("Forbidden\n");
    }
    header('Content-Type: text/plain; charset=utf-8');
}

$channels = channels($cfg);
if (!$channels) {
    exit("channel_id is not set in config.php\n");
}
$pdo = open_db($cfg);
$failed = false;
foreach ($channels as $channel => $readKey) {
    try {
        echo fetch_channel($pdo, $channel, $readKey), "\n";
    } catch (Throwable $e) {
        $failed = true;
        echo "Channel $channel: ", $e->getMessage(), "\n";
    }
}
exit($failed ? 1 : 0);

function fetch_channel(PDO $pdo, int $channel, string $readKey): string
{
    // Continue from the newest stored entry (ThingSpeak's start is inclusive; duplicates are ignored)
    $stmt = $pdo->prepare('SELECT MAX(created_at) FROM measurements WHERE channel_id = ?');
    $stmt->execute([$channel]);
    $last = $stmt->fetchColumn();

    $params = ['results' => 8000];  // ThingSpeak's maximum per request
    if ($readKey !== '') {
        $params['api_key'] = $readKey;
    }
    if ($last) {
        $params['start'] = $last;  // UTC, "YYYY-MM-DD HH:MM:SS"
    }
    $url = "https://api.thingspeak.com/channels/$channel/feeds.json?" . http_build_query($params, '', '&', PHP_QUERY_RFC3986);

    $ch = curl_init($url);
    curl_setopt_array($ch, [
        CURLOPT_RETURNTRANSFER => true,
        CURLOPT_TIMEOUT => 30,
        CURLOPT_USERAGENT => 'measurement-site-fetch',
    ]);
    $body = curl_exec($ch);
    $code = curl_getinfo($ch, CURLINFO_HTTP_CODE);
    curl_close($ch);

    $data = $body === false ? null : json_decode($body, true);
    if ($code !== 200 || !is_array($data) || !isset($data['feeds'])) {
        // A private channel without a valid read key answers "-1"
        throw new RuntimeException("ThingSpeak request failed (HTTP $code) - check the channel ID and read key");
    }

    // Channel and field names, so the page uses the names set in ThingSpeak
    $names = [];
    for ($i = 1; $i <= 8; $i++) {
        $names["field$i"] = $data['channel']["field$i"] ?? null;
    }
    $pdo->prepare('REPLACE INTO channels (channel_id, name, field_names, updated_at) VALUES (?, ?, ?, UTC_TIMESTAMP())')
        ->execute([$channel, $data['channel']['name'] ?? '', json_encode($names, JSON_UNESCAPED_UNICODE)]);

    // Store the entries. Missing fields and values like "nan" become NULL
    $insert = $pdo->prepare(
        'INSERT IGNORE INTO measurements (channel_id, entry_id, created_at,
            field1, field2, field3, field4, field5, field6, field7, field8)
         VALUES (?, ?, ?, ?, ?, ?, ?, ?, ?, ?, ?)'
    );
    $added = 0;
    $pdo->beginTransaction();
    foreach ($data['feeds'] as $feed) {
        $time = gmdate('Y-m-d H:i:s', strtotime($feed['created_at']));
        $row = [$channel, (int)$feed['entry_id'], $time];
        for ($i = 1; $i <= 8; $i++) {
            $v = $feed["field$i"] ?? null;
            $row[] = (is_numeric($v) && is_finite((float)$v)) ? (float)$v : null;
        }
        $insert->execute($row);
        $added += $insert->rowCount();
    }
    $pdo->commit();

    return "Channel $channel: " . count($data['feeds']) . " entries from ThingSpeak, $added new";
}
