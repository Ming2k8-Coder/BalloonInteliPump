#include <stdio.h>
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "freertos/ringbuf.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "nvs_flash.h"
#include "esp_timer.h"
#include "driver/uart.h"
#include "lwip/sockets.h"

#include "bip_config.h"
#include "bip_sensors.h"
#include "bip_motor.h"
#include "bip_storage.h"
#include "bip_state.h"
#include "bip_diagnostics.h"
#include "bip_volume_estimator.h"
#include "bip_balloon_physics.h"

static const char *TAG = "BIP_MAIN";

// FreeRTOS Lock-Free RingBuffer for Core 1 -> Core 0 Telemetry Passing
static RingbufHandle_t telemetryRingBuf = NULL;
static spi_device_handle_t ads1220_spi_handle = NULL;
static adc_oneshot_unit_handle_t adc_handle = NULL;
static TaskHandle_t s_controlTaskHandle = NULL;

static bool isConnected = false;
static bool wifiConnected = false;
static uint32_t totalSamplesProcessed = 0;

static void execute_command(const char* cmd);

// ==========================================
// 1. FREERTOS CORE 1 TASK: CONTROL & SENSING (Hardware ISR Synchronized)
// ==========================================
static void control_task(void *pvParameters) {
    ESP_LOGI(TAG, "Control Task running on Core %d (Priority %d)", xPortGetCoreID(), uxTaskPriorityGet(NULL));
    
    s_controlTaskHandle = xTaskGetCurrentTaskHandle();
    init_drdy_isr(s_controlTaskHandle);

    TelemetrySample localSample;

    while (1) {
        // Wait for Hardware ADS1220 DRDY Falling Edge Interrupt Notification (or timeout fallback)
        uint32_t ulNotificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(2));
        
        // 1. Read ADS1220 Pressure Sensor (High Speed 2000 SPS)
        if (ads1220_spi_handle != NULL) {
            int32_t raw_p = read_ads1220_raw(ads1220_spi_handle);
            float press_volts = (float)raw_p * (5.0f / 8388607.0f);
            pressureSensor.setRawValue(press_volts);
        }

        // 2. Read Voltage & Current ADCs
        if (adc_handle != NULL) {
            update_adc_sensors(adc_handle);
        }

        // 3. Safety Check & Motor Update
        pumpMotor.update(currentSensor.getValue(), voltageSensor.getValue(), 12.0f);
        if (pumpMotor.isSafetyTriggered() && currentMode != MODE_ERROR) {
            trigger_error("MOTOR SAFETY", pumpMotor.getSafetyError());
        }

        // 4. Update PID & State Machine
        update_state_machine();
        float mcu_temp = read_mcu_temp();
        update_volume_estimator_ext(pressureSensor.getValue(), (float)pumpMotor.getCurrentPWM(), mcu_temp, 0.0005f);
        update_balloon_physics_ext(pressureSensor.getValue(), get_local_dp_dt(), get_local_d2p_dt2(), mcu_temp, 0.0005f);
        totalSamplesProcessed++;

        // 5. Continuous High-Precision Pop Black Box RAM Recording (2000 SPS)
        record_pop_sample(pressureSensor.getValue(), pressureSensor.getRawValue(),
                          pumpMotor.getCurrentPWM(), currentSensor.getValue());

        // 6. Zero-Copy Push to Lock-Free FreeRTOS RingBuffer for Core 0 Transmission
        if (isConnected && telemetryRingBuf != NULL) {
            localSample.timestamp = (double)esp_timer_get_time() / 1000.0;
            localSample.pressure = pressureSensor.getValue();
            localSample.pwm = pumpMotor.getCurrentPWM();
            localSample.voltage = voltageSensor.getValue();
            localSample.current = currentSensor.getValue();
            localSample.mode = (uint8_t)currentMode;
            localSample.rawPressure = pressureSensor.getRawValue();
            localSample.rawVoltage = voltageSensor.getRawValue();
            localSample.rawCurrent = currentSensor.getRawValue();
            localSample.mcuTemp = mcu_temp;

            xRingbufferSend(telemetryRingBuf, &localSample, sizeof(TelemetrySample), 0);
        }
    }
}

// ==========================================
// 2. FREERTOS CORE 0 TASK: NETWORKING & BSD UDP SOCKETS
// ==========================================
static void net_telemetry_task(void *pvParameters) {
    ESP_LOGI(TAG, "Network Telemetry Task running on Core %d", xPortGetCoreID());

    int sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_IP);
    if (sock < 0) {
        ESP_LOGE(TAG, "Unable to create UDP socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }

    int broadcast = 1;
    setsockopt(sock, SOL_SOCKET, SO_BROADCAST, &broadcast, sizeof(broadcast));

    struct sockaddr_in dest_addr;
    dest_addr.sin_addr.s_addr = inet_addr(UDP_BROADCAST_IP);
    dest_addr.sin_family = AF_INET;
    dest_addr.sin_port = htons(UDP_PORT);

    char packetBuffer[1500];
    int offset = 0;
    int sampleBatchCount = 0;

    while (1) {
        size_t item_size = 0;
        TelemetrySample *sample = (TelemetrySample *)xRingbufferReceive(telemetryRingBuf, &item_size, pdMS_TO_TICKS(50));
        
        if (sample != NULL && item_size == sizeof(TelemetrySample)) {
            if (wifiConnected && isConnected) {
                char payload[128];
                snprintf(payload, sizeof(payload), "BIP,%.3f,%.2f,%d,%.2f,%.2f,%d,%.2f,%.2f,%.2f",
                         sample->timestamp, sample->pressure, sample->pwm,
                         sample->voltage, sample->current, sample->mode,
                         sample->rawPressure, sample->rawVoltage, sample->rawCurrent);

                uint8_t checksum = 0;
                for (char* p = payload; *p; p++) checksum ^= (uint8_t)*p;

                char line[160];
                int line_len = snprintf(line, sizeof(line), "$%s*%02X\n", payload, checksum);
                
                if (offset + line_len < sizeof(packetBuffer) - 1) {
                    strcpy(packetBuffer + offset, line);
                    offset += line_len;
                    sampleBatchCount++;
                }

                if (sampleBatchCount >= TELEM_BUFFER_SIZE) {
                    sendto(sock, packetBuffer, offset, 0, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
                    offset = 0;
                    sampleBatchCount = 0;
                }
            }
            vRingbufferReturnItem(telemetryRingBuf, (void *)sample);
        }
    }
}

// ==========================================
// 3. UART COMMAND LISTENER TASK
// ==========================================
static void uart_cmd_task(void *pvParameters) {
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };
    uart_param_config(UART_NUM_0, &uart_config);
    uart_driver_install(UART_NUM_0, 256, 0, 0, NULL, 0);

    char rx_buffer[128];
    int rx_index = 0;

    while (1) {
        uint8_t byte;
        int len = uart_read_bytes(UART_NUM_0, &byte, 1, pdMS_TO_TICKS(20));
        if (len > 0) {
            if (byte == '\n' || byte == '\r') {
                if (rx_index > 0) {
                    rx_buffer[rx_index] = '\0';
                    execute_command(rx_buffer);
                    rx_index = 0;
                }
            } else if (rx_index < sizeof(rx_buffer) - 1) {
                rx_buffer[rx_index++] = byte;
            } else {
                rx_index = 0;
            }
        }
    }
}

// Command Executor
static void execute_command(const char* cmd) {
    if (strcmp(cmd, "CONNECT") == 0) {
        isConnected = true;
        printf("CONNECTED\n");
    } else if (strcmp(cmd, "START_MANUAL") == 0) {
        post_mode_change(MODE_MANUAL);
    } else if (strcmp(cmd, "START_SMART") == 0) {
        post_mode_change(MODE_SMART);
    } else if (strcmp(cmd, "START_BURST") == 0) {
        post_mode_change(MODE_BURST);
    } else if (strcmp(cmd, "START_PULSE") == 0) {
        post_mode_change(MODE_PULSE);
        printf("PULSE_STARTED\n");
    } else if (strncmp(cmd, "SET_PULSE ", 10) == 0) {
        float base = 15.0f, amp = 3.0f, freq = 0.5f;
        sscanf(cmd + 10, "%f %f %f", &base, &amp, &freq);
        set_pulse_params(base, amp, freq);
        post_mode_change(MODE_PULSE);
        printf("PULSE_SET base=%.1f amp=%.1f freq=%.2f\n", base, amp, freq);
    } else if (strcmp(cmd, "START_PATTERN") == 0) {
        post_mode_change(MODE_PATTERN);
        printf("PATTERN_STARTED\n");
    } else if (strncmp(cmd, "SET_PATTERN ", 12) == 0) {
        int pat = 0; float min_p = 10.0f, max_p = 25.0f, period = 10.0f;
        sscanf(cmd + 12, "%d %f %f %f", &pat, &min_p, &max_p, &period);
        set_pattern((WaveformPattern)pat, min_p, max_p, period);
        post_mode_change(MODE_PATTERN);
        printf("PATTERN_SET pat=%d min=%.1f max=%.1f period=%.1f\n", pat, min_p, max_p, period);
    } else if (strncmp(cmd, "SET_DIAMETER ", 13) == 0) {
        float diam = atof(cmd + 13);
        set_target_diameter(diam);
        printf("DIAMETER_SET %.1f cm\n", diam);
    } else if (strcmp(cmd, "START_RIDE") == 0) {
        post_mode_change(MODE_RIDE);
        printf("RIDE_STARTED\n");
    } else if (strncmp(cmd, "SET_RIDER ", 10) == 0) {
        float weight = 70.0f, sink = 8.0f;
        sscanf(cmd + 10, "%f %f", &weight, &sink);
        set_ride_params(weight, sink);
        printf("RIDE_PARAMS_SET rider=%.1f kg, sink=%.1f cm\n", weight, sink);
    } else if (strncmp(cmd, "START_CONDITION", 15) == 0) {
        int cycles = 4;
        if (strlen(cmd) > 15) cycles = atoi(cmd + 16);
        start_balloon_conditioning(cycles);
        printf("CONDITION_STARTED cycles=%d\n", cycles);
    } else if (strcmp(cmd, "GET_FATIGUE") == 0) {
        FatigueState f = get_fatigue_state();
        printf("FATIGUE,damage=%.4f,bounces=%lu,max_stress=%.1f,capacity_pct=%.1f\n",
               f.accumulated_damage, (unsigned long)f.total_bounce_count,
               f.max_bounce_stress_kpa, f.safe_pressure_remaining_pct);
    } else if (strcmp(cmd, "RESET_FATIGUE") == 0) {
        reset_fatigue_tracker();
        printf("FATIGUE_RESET\n");
    } else if (strncmp(cmd, "CALC_RIDE ", 10) == 0) {
        float w = 70.0f, f = 0.5f; int t = 3;
        sscanf(cmd + 10, "%f %d %f", &w, &t, &f);
        RideInflationAdvice a = compute_ride_inflation(w, (BalloonType)t, f);
        printf("RIDE_ADVICE,P_target=%.2f,P_max=%.2f,sink=%.1f,margin=%.1f%%\n",
               a.recommended_pressure_kpa, a.max_safe_pressure_kpa,
               a.predicted_sink_depth_cm, a.burst_safety_margin_pct);
    } else if (strncmp(cmd, "SET_HYSTERESIS ", 15) == 0) {
        float deadband = atof(cmd + 15);
        set_hysteresis_band(deadband);
        printf("HYSTERESIS_SET %.2f kPa\n", deadband);
    } else if (strncmp(cmd, "SET_PUMP_MODEL ", 15) == 0) {
        float dz = 35.0f, stall = 72.0f;
        sscanf(cmd + 15, "%f %f", &dz, &stall);
        set_pump_model_params(dz, stall);
        printf("PUMP_MODEL_SET deadzone=%.1f stall=%.1f\n", dz, stall);
    } else if (strncmp(cmd, "SET_SHAPE_MODEL ", 16) == 0) {
        float neck_v = 0.05f, shape = 0.85f;
        sscanf(cmd + 16, "%f %f", &neck_v, &shape);
        set_balloon_shape_params(neck_v, shape);
        printf("SHAPE_MODEL_SET neck=%.3f L shape=%.2f\n", neck_v, shape);
    } else if (strcmp(cmd, "STOP") == 0) {
        pumpMotor.emergencyStop();
        post_mode_change(MODE_IDLE);
    } else if (strcmp(cmd, "VALVE_ON") == 0) {
        write_solenoid_pwm(255);
        printf("VALVE_OPEN\n");
    } else if (strcmp(cmd, "VALVE_OFF") == 0) {
        write_solenoid_pwm(0);
        printf("VALVE_CLOSED\n");
    } else if (strncmp(cmd, "SET_VALVE_PWM ", 14) == 0) {
        int val = atoi(cmd + 14);
        write_solenoid_pwm(std::clamp(val, 0, 255));
        printf("VALVE_PWM_SET %d\n", val);
    } else if (strncmp(cmd, "SET_PWM ", 8) == 0) {
        int val = atoi(cmd + 8);
        set_manual_mode(false);
        pumpMotor.setTargetPWM(std::clamp(val, 0, 255));
    } else if (strncmp(cmd, "SET_TARGET ", 11) == 0) {
        float val = atof(cmd + 11);
        set_manual_target(val);
    } else if (strncmp(cmd, "SET_RAMP ", 9) == 0) {
        float rate = atof(cmd + 9);
        pumpMotor.setRampRate(rate);
        printf("RAMP_SET %.2f\n", rate);
    } else if (strncmp(cmd, "SET_BALLOON_TYPE ", 17) == 0) {
        int btype = atoi(cmd + 17);
        set_balloon_preset((BalloonType)std::clamp(btype, 0, 3));
        printf("BALLOON_TYPE_SET %d\n", btype);
    } else if (strcmp(cmd, "ZERO_PRESSURE") == 0 || strcmp(cmd, "ZERO_TARE") == 0) {
        pressureSensor.tare();
        printf("PRESSURE_ZEROED\n");
    } else if (strcmp(cmd, "GET_POP_DUMP") == 0 || strcmp(cmd, "DUMP_POP") == 0) {
        dump_pop_recording();
    } else if (strcmp(cmd, "RUN_SELF_TEST") == 0 || strcmp(cmd, "SELF_TEST") == 0) {
        print_diagnostic_report();
    } else if (strcmp(cmd, "GET_JSON_STATUS") == 0 || strcmp(cmd, "JSON_STATUS") == 0) {
        BalloonMaterialPhysics phys = get_balloon_physics_state();
        OnlineMaterialParams mat = get_online_material_params();
        BounceRhythmState rhythm = get_bounce_rhythm();
        FatigueState fat = get_fatigue_state();
        BalloonPhysicsEstimate vol = get_balloon_physics_estimate();

        printf("{\"uptime_s\":%lld,\"heap\":%lu,\"mcu_temp\":%.1f,\"mode\":%d,\"pressure_kpa\":%.2f,\"pwm\":%d,\"diameter_cm\":%.2f,\"vol_l\":%.3f,\"lambda\":%.2f,\"stress_kpa\":%.1f,\"c10\":%.1f,\"c01\":%.1f,\"impact\":%d,\"bounces\":%lu,\"damage\":%.4f,\"rhythm_hz\":%.2f}\n",
               esp_timer_get_time() / 1000000, esp_get_free_heap_size(), read_mcu_temp(),
               (int)currentMode, pressureSensor.getValue(), pumpMotor.getCurrentPWM(),
               vol.diameter_cm, vol.volume_liters, phys.stretch_ratio, phys.hyperelastic_stress_kpa,
               mat.estimated_C10, mat.estimated_C01, (int)phys.impact_type,
               (unsigned long)rhythm.bounce_count, fat.accumulated_damage, rhythm.frequency_hz);
    } else if (strcmp(cmd, "CAL_SAVE") == 0) {
        save_system_config();
        printf("CAL_SAVED\n");
    }
}



#include "bip_webserver.h"

// WiFi SoftAP Initialization
static void wifi_init_softap(void) {
    ESP_ERROR_CHECK(esp_netif_init());
    ESP_ERROR_CHECK(esp_event_loop_create_default());
    esp_netif_create_default_wifi_ap();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    wifi_config_t wifi_config = {};
    strcpy((char*)wifi_config.ap.ssid, WIFI_SSID);
    strcpy((char*)wifi_config.ap.password, WIFI_PASSWORD);
    wifi_config.ap.ssid_len = strlen(WIFI_SSID);
    wifi_config.ap.channel = 1;
    wifi_config.ap.authmode = WIFI_AUTH_WPA2_PSK;
    wifi_config.ap.max_connection = 4;

    if (strlen(WIFI_PASSWORD) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_start());

    wifiConnected = true;
    ESP_LOGI(TAG, "WiFi SoftAP initialized. SSID:%s Password:%s", WIFI_SSID, WIFI_PASSWORD);

    // Start Native ESP-IDF WebServer & REST API
    start_bip_webserver();
}

// ==========================================
// 4. ESP-IDF APPLICATION ENTRY POINT
// ==========================================
extern "C" void app_main(void) {
    ESP_LOGI(TAG, "=== Starting BalloonInteliPump Native ESP-IDF Firmware ===");

    // 1. Initialize NVS Storage
    init_storage();

    // 2. Initialize Hardware Drivers
    init_spi_ads1220(&ads1220_spi_handle);
    init_internal_adc(&adc_handle);
    init_mcu_temp_sensor();
    pumpMotor.begin();
    init_solenoid_valve();
    init_states();

    // 3. Initialize WiFi, Balloon Physics & 32KB FreeRTOS Lock-Free RingBuffer
    wifi_init_softap();
    init_balloon_physics(BALLOON_11INCH);
    telemetryRingBuf = xRingbufferCreate(TELEM_RING_BUF_SIZE, RINGBUF_TYPE_NOSPLIT);

    // 4. Spawn Dual-Core FreeRTOS Tasks with Core Affinity
    xTaskCreatePinnedToCore(control_task, "control_task", 4096, NULL, CONTROL_TASK_PRIO, NULL, CONTROL_TASK_CORE);
    xTaskCreatePinnedToCore(net_telemetry_task, "net_task", 4096, NULL, NET_TASK_PRIO, NULL, NET_TASK_CORE);
    xTaskCreatePinnedToCore(uart_cmd_task, "uart_task", 3072, NULL, 4, NULL, 0);

    ESP_LOGI(TAG, "All Advanced ESP-IDF Native tasks initialized and running!");
}

