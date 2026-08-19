#include "bip_webserver.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "bip_state.h"
#include "bip_balloon_physics.h"
#include "bip_volume_estimator.h"
#include "bip_storage.h"
#include "esp_log.h"
#include "esp_timer.h"
#include <string.h>
#include <stdio.h>
#include <stdlib.h>

static const char *TAG = "BIP_WEBSERVER";
static httpd_handle_t server_handle = NULL;

static const char index_html[] = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8"><meta name="viewport" content="width=device-width,initial-scale=1.0">
<title>BalloonInteliPump Dashboard</title>
<style>
:root{--bg:#0f172a;--card:rgba(30,41,59,0.7);--accent:#3b82f6;--pink:#ec4899;--text:#f8fafc;--border:rgba(255,255,255,0.1);}
*{box-sizing:border-box;margin:0;padding:0;font-family:system-ui,-apple-system,sans-serif;}
body{background:var(--bg);color:var(--text);padding:1rem;min-height:100vh;}
header{display:flex;justify-content:space-between;align-items:center;padding:1rem;background:var(--card);backdrop-filter:blur(10px);border:1px solid var(--border);border-radius:12px;margin-bottom:1rem;}
h1{font-size:1.4rem;background:linear-gradient(45deg,var(--accent),var(--pink));-webkit-background-clip:text;-webkit-text-fill-color:transparent;}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(280px,1fr));gap:1rem;}
.card{background:var(--card);backdrop-filter:blur(10px);border:1px solid var(--border);border-radius:12px;padding:1.2rem;}
.val{font-size:2rem;font-weight:700;margin:0.4rem 0;color:var(--accent);}
.btn{background:var(--accent);color:#fff;border:none;padding:0.6rem 1.2rem;border-radius:8px;font-weight:600;cursor:pointer;margin:0.2rem;transition:0.2s;}
.btn:hover{opacity:0.9;transform:translateY(-1px);}
.btn-danger{background:#ef4444;}
.btn-pink{background:var(--pink);}
input[type=range]{width:100%;margin:0.5rem 0;}
</style>
</head>
<body>
<header>
  <h1>🎈 BalloonInteliPump Dashboard</h1>
  <div id="status-badge" style="color:#10b981;font-weight:600;">● Connected (SoftAP)</div>
</header>
<div class="grid">
  <div class="card">
    <h3>Internal Gauge Pressure</h3>
    <div class="val" id="press">0.00 <span style="font-size:1rem;">kPa</span></div>
    <p>Stretch Ratio (λ): <span id="lambda">1.00</span></p>
    <p>Stress: <span id="stress">0.0</span> kPa</p>
  </div>
  <div class="card">
    <h3>Balloon Geometry</h3>
    <div class="val" id="diam" style="color:var(--pink);">0.0 <span style="font-size:1rem;">cm</span></div>
    <p>Volume: <span id="vol">0.000</span> L</p>
    <p>Impact: <span id="impact">NONE</span></p>
  </div>
  <div class="card">
    <h3>Motor & Power Drive</h3>
    <div class="val" id="pwm" style="color:#f59e0b;">0 <span style="font-size:1rem;">PWM</span></div>
    <p>Bounces: <span id="bounces">0</span> | Rhythm: <span id="freq">0.0</span> Hz</p>
    <p>Fatigue Damage: <span id="damage">0.0%</span></p>
  </div>
  <div class="card">
    <h3>Mode Controls</h3>
    <button class="btn" onclick="sendCmd('START_SMART')">Smart Mode</button>
    <button class="btn btn-pink" onclick="sendCmd('START_RIDE')">Ride Mode</button>
    <button class="btn" onclick="sendCmd('START_PULSE')">Pulse Mode</button>
    <button class="btn" onclick="sendCmd('START_CONDITION 4')">Pre-Condition</button>
    <button class="btn btn-danger" onclick="sendCmd('STOP')">EMERGENCY STOP</button>
  </div>
  <div class="card">
    <h3>Target Diameter Control</h3>
    <input type="range" id="diamSlider" min="10" max="60" value="25" onchange="sendCmd('SET_DIAMETER '+this.value)">
    <p>Target: <span id="sliderVal">25</span> cm</p>
  </div>
  <div class="card">
    <h3>Pre-Ride Calculator</h3>
    <button class="btn" onclick="sendCmd('CALC_RIDE 75.0 3 0.5')">Calculate 75kg Rider (36")</button>
    <p id="calcRes" style="margin-top:0.5rem;font-size:0.9rem;color:#94a3b8;"></p>
  </div>
</div>
<script>
document.getElementById('diamSlider').oninput=function(){document.getElementById('sliderVal').innerText=this.value;};
function sendCmd(cmd){
  fetch('/api/cmd?c='+encodeURIComponent(cmd),{method:'POST'})
  .then(r=>r.text()).then(t=>{if(t.includes('RIDE_ADVICE'))document.getElementById('calcRes').innerText=t;});
}
function updateUI(d){
  document.getElementById('press').innerHTML=d.pressure_kpa.toFixed(2)+' <span style="font-size:1rem;">kPa</span>';
  document.getElementById('lambda').innerText=d.lambda.toFixed(2);
  document.getElementById('stress').innerText=d.stress_kpa.toFixed(1);
  document.getElementById('diam').innerHTML=d.diameter_cm.toFixed(1)+' <span style="font-size:1rem;">cm</span>';
  document.getElementById('vol').innerText=d.vol_l.toFixed(3);
  document.getElementById('pwm').innerHTML=d.pwm+' <span style="font-size:1rem;">PWM</span>';
  document.getElementById('bounces').innerText=d.bounces;
  document.getElementById('freq').innerText=d.rhythm_hz.toFixed(2);
  document.getElementById('damage').innerText=(d.damage*100).toFixed(1)+'%';
  const imp=['NONE','BOUNCE','BURST','SQUEEZE'];
  document.getElementById('impact').innerText=imp[d.impact]||'NONE';
}
if(window.EventSource){
  const evs=new EventSource('/api/stream');
  evs.onmessage=function(e){updateUI(JSON.parse(e.data));};
  evs.onerror=function(){setInterval(()=>{fetch('/api/status').then(r=>r.json()).then(d=>updateUI(d)).catch(e=>{});},250);};
}else{
  setInterval(()=>{fetch('/api/status').then(r=>r.json()).then(d=>updateUI(d)).catch(e=>{});},250);
}
</script>
</body>
</html>
)rawhtml";

static esp_err_t root_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/html");
    return httpd_resp_send(req, index_html, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t status_get_handler(httpd_req_t *req) {
    BalloonMaterialPhysics phys = get_balloon_physics_state();
    OnlineMaterialParams mat = get_online_material_params();
    BounceRhythmState rhythm = get_bounce_rhythm();
    FatigueState fat = get_fatigue_state();
    BalloonPhysicsEstimate vol = get_balloon_physics_estimate();

    char json[384];
    snprintf(json, sizeof(json),
             "{\"uptime_s\":%lld,\"heap\":%lu,\"mcu_temp\":%.1f,\"mode\":%d,\"pressure_kpa\":%.2f,\"pwm\":%d,\"diameter_cm\":%.2f,\"vol_l\":%.3f,\"lambda\":%.2f,\"stress_kpa\":%.1f,\"c10\":%.1f,\"c01\":%.1f,\"impact\":%d,\"bounces\":%lu,\"damage\":%.4f,\"rhythm_hz\":%.2f}",
             esp_timer_get_time() / 1000000, (unsigned long)esp_get_free_heap_size(), read_mcu_temp(),
             (int)currentMode, pressureSensor.getValue(), pumpMotor.getCurrentPWM(),
             vol.diameter_cm, vol.volume_liters, phys.stretch_ratio, phys.hyperelastic_stress_kpa,
             mat.estimated_C10, mat.estimated_C01, (int)phys.impact_type,
             (unsigned long)rhythm.bounce_count, fat.accumulated_damage, rhythm.frequency_hz);

    httpd_resp_set_type(req, "application/json");
    return httpd_resp_send(req, json, HTTPD_RESP_USE_STRLEN);
}

static esp_err_t stream_get_handler(httpd_req_t *req) {
    httpd_resp_set_type(req, "text/event-stream");
    httpd_resp_set_hdr(req, "Cache-Control", "no-cache");
    httpd_resp_set_hdr(req, "Connection", "keep-alive");

    for (int count = 0; count < 600; count++) { // Stream for up to 30 seconds per connection
        BalloonMaterialPhysics phys = get_balloon_physics_state();
        OnlineMaterialParams mat = get_online_material_params();
        BounceRhythmState rhythm = get_bounce_rhythm();
        FatigueState fat = get_fatigue_state();
        BalloonPhysicsEstimate vol = get_balloon_physics_estimate();

        char sse_chunk[450];
        int len = snprintf(sse_chunk, sizeof(sse_chunk),
                  "data: {\"uptime_s\":%lld,\"heap\":%lu,\"mcu_temp\":%.1f,\"mode\":%d,\"pressure_kpa\":%.2f,\"pwm\":%d,\"diameter_cm\":%.2f,\"vol_l\":%.3f,\"lambda\":%.2f,\"stress_kpa\":%.1f,\"c10\":%.1f,\"c01\":%.1f,\"impact\":%d,\"bounces\":%lu,\"damage\":%.4f,\"rhythm_hz\":%.2f}\n\n",
                  esp_timer_get_time() / 1000000, (unsigned long)esp_get_free_heap_size(), read_mcu_temp(),
                  (int)currentMode, pressureSensor.getValue(), pumpMotor.getCurrentPWM(),
                  vol.diameter_cm, vol.volume_liters, phys.stretch_ratio, phys.hyperelastic_stress_kpa,
                  mat.estimated_C10, mat.estimated_C01, (int)phys.impact_type,
                  (unsigned long)rhythm.bounce_count, fat.accumulated_damage, rhythm.frequency_hz);

        esp_err_t res = httpd_resp_send_chunk(req, sse_chunk, len);
        if (res != ESP_OK) break;
        vTaskDelay(pdMS_TO_TICKS(50));
    }
    httpd_resp_send_chunk(req, NULL, 0);
    return ESP_OK;
}

extern void execute_command(const char* cmd);

static esp_err_t cmd_post_handler(httpd_req_t *req) {
    char buf[128] = {0};
    int ret = httpd_req_get_url_query_str(req, buf, sizeof(buf));
    if (ret == ESP_OK) {
        char param[128] = {0};
        if (httpd_query_key_value(buf, "c", param, sizeof(param)) == ESP_OK) {
            execute_command(param);
            httpd_resp_send(req, "OK", HTTPD_RESP_USE_STRLEN);
            return ESP_OK;
        }
    }
    httpd_resp_send_500(req);
    return ESP_FAIL;
}

esp_err_t start_bip_webserver(void) {
    if (server_handle != NULL) return ESP_OK;

    httpd_config_t config = HTTPD_DEFAULT_CONFIG();
    config.max_uri_handlers = 8;
    config.stack_size = 4096;

    esp_err_t ret = httpd_start(&server_handle, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start HTTP WebServer: %d", ret);
        return ret;
    }

    httpd_uri_t root_uri = { .uri = "/", .method = HTTP_GET, .handler = root_get_handler, .user_ctx = NULL };
    httpd_uri_t status_uri = { .uri = "/api/status", .method = HTTP_GET, .handler = status_get_handler, .user_ctx = NULL };
    httpd_uri_t stream_uri = { .uri = "/api/stream", .method = HTTP_GET, .handler = stream_get_handler, .user_ctx = NULL };
    httpd_uri_t cmd_uri = { .uri = "/api/cmd", .method = HTTP_POST, .handler = cmd_post_handler, .user_ctx = NULL };

    httpd_register_uri_handler(server_handle, &root_uri);
    httpd_register_uri_handler(server_handle, &status_uri);
    httpd_register_uri_handler(server_handle, &stream_uri);
    httpd_register_uri_handler(server_handle, &cmd_uri);

    ESP_LOGI(TAG, "Native ESP-IDF WebServer & REST API started on port 80!");
    return ESP_OK;
}

void stop_bip_webserver(void) {
    if (server_handle != NULL) {
        httpd_stop(server_handle);
        server_handle = NULL;
        ESP_LOGI(TAG, "HTTP WebServer stopped.");
    }
}

bool is_webserver_running(void) {
    return (server_handle != NULL);
}

