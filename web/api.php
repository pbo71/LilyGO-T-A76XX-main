<?php
// JSON for the page: api.php?range=24h|7d|30d|1y
// Returns the field names, the latest value of every field and the measurements in the range.

require __DIR__ . '/db.php';
$cfg = load_config();
$channel = (int)$cfg['channel_id'];

$ranges = ['24h' => 1, '7d' => 7, '30d' => 30, '1y' => 365];
$range = $_GET['range'] ?? '24h';
if (!isset($ranges[$range])) {
    $range = '24h';
}

header('Content-Type: application/json; charset=utf-8');
header('Cache-Control: public, max-age=120');

try {
    $pdo = open_db($cfg);

    $stmt = $pdo->prepare('SELECT name, field_names FROM channels WHERE channel_id = ?');
    $stmt->execute([$channel]);
    $info = $stmt->fetch() ?: ['name' => '', 'field_names' => '{}'];

    // Latest non-empty value per field (fields are left out when a sensor is missing)
    $latest = [];
    for ($i = 1; $i <= 8; $i++) {
        $stmt = $pdo->prepare("SELECT field$i AS v, UNIX_TIMESTAMP(created_at) AS t FROM measurements
                               WHERE channel_id = ? AND field$i IS NOT NULL ORDER BY created_at DESC LIMIT 1");
        $stmt->execute([$channel]);
        $r = $stmt->fetch();
        $latest["field$i"] = $r ? ['v' => (float)$r['v'], 't' => (int)$r['t'] * 1000] : null;
    }

    $stmt = $pdo->prepare('SELECT UNIX_TIMESTAMP(created_at) AS t, field1, field2, field3, field4, field5, field6, field7, field8
                           FROM measurements
                           WHERE channel_id = ? AND created_at >= UTC_TIMESTAMP() - INTERVAL ? DAY
                           ORDER BY created_at');
    $stmt->execute([$channel, $ranges[$range]]);
    $rows = [];
    foreach ($stmt as $r) {
        $row = ['t' => (int)$r['t'] * 1000];
        for ($i = 1; $i <= 8; $i++) {
            $row["field$i"] = $r["field$i"] === null ? null : (float)$r["field$i"];
        }
        $rows[] = $row;
    }

    echo json_encode([
        'channel' => $info['name'],
        'fields' => json_decode($info['field_names'], true),
        'latest' => $latest,
        'range' => $range,
        'rows' => $rows,
    ], JSON_UNESCAPED_UNICODE);
} catch (Throwable $e) {
    http_response_code(500);
    echo json_encode(['error' => 'database error']);
}
