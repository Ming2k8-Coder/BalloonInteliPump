#include "bip_webserver.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "bip_state.h"
#include "bip_diagnostics.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include <stdio.h>
#include <string.h>

static const char *TAG = "BIP_WEB";
static httpd_handle_t server = NULL;

static const char INDEX_HTML[] = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>Balloon Intelligent Pump (BIP)</title>
    <style>
        :root {
            --bg: #0f172a; --card: #1e293b; --accent: #6366f1;
            --rose: #f43f5e; --emerald: #10b981; --amber: #f59e0b; --text: #f8fafc;
        }
        body {
            font-family: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, sans-serif;
            background: var(--bg); color: var(--text); margin: 0; padding: 20px;
        }
        .header { display: flex; justify-content: space-between; align-items: center; border-bottom: 1px solid #334155; padding-bottom: 15px; }
        .grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(220px, 1fr)); gap: 15px; margin-top: 20px; }
        .card { background: var(--card); border-radius: 12px; padding: 20px; border: 1px solid #334155; box-shadow: 0 10px 15px -3px rgba(0,0,0,0.3); }
        .val { font-size: 2.2rem; font-weight: 700; color: var(--accent); margin-top: 5px; }
        .unit { font-size: 0.9rem; color: #94a3b8; }
        .btn-group { display: flex; flex-wrap: wrap; gap: 10px; margin-top: 20px; }
        button {
            background: var(--accent); color: white; border: none; padding: 12px 20px;
            font-weight: 600; border-radius: 8px; cursor: pointer; transition: all 0.2s;
        }
        button:hover { opacity: 0.9; transform: translateY(-1px); }
        button.danger { background: var(--rose); }
        button.success { background: var(--emerald); }
        button.warning { background: var(--amber); }
        .console { background: #020617; font-family: monospace; padding: 15px; border-radius: 8px; margin-top: 20px; height: 180px; overflow-y: auto; color: #38bdf8; }
    </style>
</head>
<body>
    <div class="header">
        <h2>🎈 Balloon Intelligent Pump</h2>
        <span id="status-badge" style="background:#10b981; padding:6px 12px; border-radius:20px; font-weight:600; font-size:0.85rem;">CONNECTED</span>
    </div>

    <div class="grid">
        <div class="card"><div>Pressure</div><div class="val" id="val-p">0.00</div><div class="unit">kPa</div></div>
        <div class="card"><div>Motor Duty</div><div class="val" id="val-pwm">0</div><div class="unit">% PWM</div></div>
        <div class="card"><div>Voltage</div><div class="val" id="val-v">0.0</div><div class="unit">Volts</div></div>
        <div class="card"><div>Current</div><div class="val" id="val-i">0.00</div><div class="unit">Amps</div></div>
        <div class="card"><div>MCU Temp</div><div class="val" id="val-temp">0.0</div><div class="unit">°C</div></div>
    </div>

    <h3>🎮 Mode Controls</h3>
    <div class="btn-group">
        <button class="danger" onclick="sendCmd('STOP')">🛑 EMERGENCY STOP</button>
        <button class="success" onclick="sendCmd('START_SMART')">🧠 Smart Yield</button>
        <button class="warning" onclick="sendCmd('START_BURST')">💥 Burst Test</button>
        <button onclick="sendCmd('START_PULSE')">🫁 Breathing Pulse</button>
        <button onclick="sendCmd('START_PATTERN')">🌊 Waveform Pattern</button>
        <button onclick="sendCmd('ZERO_PRESSURE')">🎯 Tare Baseline</button>
        <button onclick="sendCmd('RUN_SELF_TEST')">🩺 Self Test</button>
        <button onclick="sendCmd('GET_POP_DUMP')">💾 Dump Pop RAM</button>
    </div>

    <h3>📜 Telemetry Console</h3>
    <div class="console" id="console-log">Connecting to ESP32 telemetry...</div>

    <script>
        function log(msg) {
            const c = document.getElementById('console-log');
            c.innerHTML += `<div>[${new Date().toLocaleTimeString()}] ${msg}</div>`;
            c.scrollTop = c.scrollHeight;
        }

        async function sendCmd(cmd) {
            try {
                const res = await fetch(`/api/command?cmd=${encodeURIComponent(cmd)}`);
                const text = await res.text();
                log(`Command: ${cmd} -> ${text}`);
            } catch(e) { log(`Error: ${e}`); }
        }

        setInterval(async () => {
            try {
                const res = await fetch('/api/status');
                const d = await res.json();
                document.getElementById('val-p').innerText = d.pressure.toFixed(2);
                document.getElementById('val-pwm').innerText = Math.round((d.pwm/255)*100);
                document.getElementById('val-v').innerText = d.voltage.toFixed(1);
                document.getElementById('val-i').innerText = d.current.toFixed(2);
                document.getElementById('val-temp').innerText = d.mcu_temp.toFixed(1);
            } catch(e) {}
        }, 100);
    </script>
</body>
</html>
)rawliteral";

// GET / Handler (Serves Embedded Web Dashboard)
static esp_err_t index_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, INDEX_HTML, HTTPD_RESP_USE_STRLEN);
}

// GET /api/status Handler (JSON Status Endpoint)
static esp_err_t status_get_handler(httpd_req_t *req) {
    char json[256];
    snprintf(json, sizeof(json),
             "{\"pressure\":%.2f,\"pwm\":%d,\"voltage\":%.2f,\"current\":%.2f,\"mcu_temp\":%.1f,\"mode\":%d,\"heap\":%lu}",
             pressureSensor.getValue(), pumpMotor.getCurrentPWM(),
             voltageSensor.getValue(), currentSensor.getValue(),
             read_mcu_temp(), (int)currentMode, esp_get_free_heap_size());
    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

// GET /api/command Handler (REST Control API)
static esp_err_t command_get_handler(httpd_req_t *req) {
    char buf[100];
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) == ESP_OK) {
        char param[64];
        if (httpd_query_key_value(buf, "cmd", param, sizeof(param)) == ESP_OK) {
            if (strcmp(param, "STOP") == 0) {
                pumpMotor.emergencyStop();
                post_mode_change(MODE_IDLE);
            } else if (strcmp(param, "START_SMART") == 0) {
                post_mode_change(MODE_SMART);
            } else if (strcmp(param, "START_BURST") == 0) {
                post_mode_change(MODE_BURST);
            } else if (strcmp(param, "START_PULSE") == 0) {
                post_mode_change(MODE_PULSE);
            } else if (strcmp(param, "START_PATTERN") == 0) {
                post_mode_change(MODE_PATTERN);
            } else if (strcmp(param, "ZERO_PRESSURE") == 0) {
                pressureSensor.tare();
            } else if (strcmp(param, "RUN_SELF_TEST") == 0) {
                print_diagnostic_report();
            } else if (strcmp(param, "GET_POP_DUMP") == 0) {
                dump_pop_recording();
            }
            httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send_err(req, HTTPD_400_BAD_REQUEST, "Missing cmd parameter");
    return ESP_FAIL;
}

esp_err_t start_web_server() {
    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.stack_size = 4096;

    ESP_LOGI(TAG, "Starting Embedded ESP32 Web Server on port %d...", config.server_port);
    if (httpd_start(&server, &config) == ESP_OK) {
        httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &index_uri);

        httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &status_uri);

        httpd_uri_t cmd_uri = { .uri = "/api/command", .method = HTTP_GET, .handler = command_get_handler, .user_ctx = NULL };
        httpd_register_uri_handler(server, &cmd_uri);

        ESP_LOGI(TAG, "Web Server started successfully!");
        return ESP_OK;
    }
    ESP_LOGE(TAG, "Failed to start Web Server");
    return ESP_FAIL;
}

void stop_web_server() {
    if (server) {
        httpd_stop(server);
        server = NULL;
    }
}
