<?php
// Shared setup for fetch.php and api.php

function load_config(): array
{
    $file = __DIR__ . '/config.php';
    if (!is_file($file)) {
        http_response_code(500);
        exit("config.php is missing - copy config.example.php to config.php\n");
    }
    return require $file;
}

function open_db(array $cfg): PDO
{
    $dsn = 'mysql:host=' . $cfg['db_host'] . ';dbname=' . $cfg['db_name'] . ';charset=utf8mb4';
    $pdo = new PDO($dsn, $cfg['db_user'], $cfg['db_pass'], [
        PDO::ATTR_ERRMODE => PDO::ERRMODE_EXCEPTION,
        PDO::ATTR_DEFAULT_FETCH_MODE => PDO::FETCH_ASSOC,
    ]);
    $pdo->exec("SET time_zone = '+00:00'");  // All times in the database are UTC
    return $pdo;
}
