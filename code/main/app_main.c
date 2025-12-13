// ==== Includes ====
#include <stdio.h>
#include <math.h>
#include <esp_rmaker_core.h>
#include <esp_rmaker_standard_params.h>
#include <esp_rmaker_standard_devices.h>
#include <app_network.h>
#include "app_priv.h"
#include "lcd_i2c.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "esp_log.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_timer.h"

// ==== Log Tag ====
static const char *TAG = "MQ2_APP";

// =========================================================
//                  RainMaker Device + Params
// =========================================================
static esp_rmaker_device_t *dev_mq2;
static esp_rmaker_param_t *param_ppm;
static esp_rmaker_param_t *param_pm25;
static esp_rmaker_param_t *param_power;
static esp_rmaker_param_t *param_ratio;
static esp_rmaker_param_t *param_status;
static esp_rmaker_param_t *param25_status;   

// =========================================================
//                     Constants & Globals
// =========================================================
#define DEFAULT_POWER          false
#define BUFFER_SIZE            5
#define READ_INTERVAL_MS       200
#define REPORT_INTERVAL_MS     1000
static volatile bool alert_mode_enabled = false;
static QueueHandle_t btn_evt_queue = NULL;

// CO Buffer
static float ppm_buf[BUFFER_SIZE] = {0};
static int ppm_index = 0;
static int ppm_count = 0;

// NEW: PM2.5 Buffer
static float pm25_buf[BUFFER_SIZE] = {0};
static int pm25_index = 0;
static int pm25_count = 0;

// =========================================================
//            RainMaker Bulk Write Callback
// =========================================================
static esp_err_t bulk_write_cb(const esp_rmaker_device_t *device,
        const esp_rmaker_param_write_req_t req[], uint8_t count,
        void *priv_data, esp_rmaker_write_ctx_t *ctx) {
    for (int i = 0; i < count; i++) {
        const char *name = esp_rmaker_param_get_name(req[i].param);
        esp_rmaker_param_val_t val = req[i].val;
        if (strcmp(name, ESP_RMAKER_DEF_POWER_NAME) == 0) {
            alert_mode_enabled = val.val.b;
            gpio_set_level(LED_MODE_PIN, alert_mode_enabled);
            ESP_LOGI(TAG, "RainMaker: Alert mode %s", alert_mode_enabled ? "ON" : "OFF");
        }
        esp_rmaker_param_update(req[i].param, val);
    }
    return ESP_OK;
}

// =========================================================
//                         Button ISR
// =========================================================
static void IRAM_ATTR button_isr(void *arg) {
    uint32_t pin = (uint32_t)arg;
    xQueueSendFromISR(btn_evt_queue, &pin, NULL);
}

static void button_task(void *arg) {
    uint32_t pin;
    TickType_t last_tick = 0;
    TickType_t debounce = pdMS_TO_TICKS(150);
    for (;;) {
        if (xQueueReceive(btn_evt_queue, &pin, portMAX_DELAY)) {
            TickType_t now = xTaskGetTickCount();
            if ((now - last_tick) > debounce) {
                alert_mode_enabled ^= 1;
                gpio_set_level(LED_MODE_PIN, alert_mode_enabled);
                ESP_LOGI(TAG, "Button: Alert mode %s",
                    alert_mode_enabled ? "ON" : "OFF");
                last_tick = now;
            }
        }
    }
}

// =========================================================
//          MQ2 + GP2Y Sampling Timer (200ms)
// =========================================================
static void measuring_timer_cb(void *arg) {
    // Read CO PPM
    float ppm = read_CO_ppm();
    ppm_buf[ppm_index] = ppm;
    ppm_index = (ppm_index + 1) % BUFFER_SIZE;
    if (ppm_count < BUFFER_SIZE)
        ppm_count++;
    // Read PM2.5:
    float pm25 = read_PM25_ppm();
    pm25_buf[pm25_index] = pm25;
    pm25_index = (pm25_index + 1) % BUFFER_SIZE;
    if (pm25_count < BUFFER_SIZE)
        pm25_count++;
}

// =========================================================
//           LCD + Cloud Reporting Timer (1s)
// =========================================================
static void report_timer_cb(void *arg) {
    if (ppm_count == 0) return;
    // ---- Average CO PPM ----
    float sum_ppm = 0;
    for (int i = 0; i < ppm_count; i++) sum_ppm += ppm_buf[i];
    float avg_ppm = sum_ppm / ppm_count;
    // NEW: Average PM2.5
    float sum_pm25 = 0;
    for (int i = 0; i < pm25_count; i++) sum_pm25 += pm25_buf[i];
    float avg_pm25 = (pm25_count > 0) ? (sum_pm25 / pm25_count) : 0;
    // ---- LCD update ----
    // Nồng độ CO:
    char buf[32];
    lcd_put_cursor(0, 0);
    lcd_send_string("CO: ");
    lcd_put_cursor(0, 4);
    snprintf(buf, sizeof(buf), "%.2f    ", avg_ppm);
    lcd_send_string(buf);
    // NEW: Nồng độ PM2.5:
    char bufpm25[32];
    lcd_put_cursor(1, 0);
    lcd_send_string("PM2.5: ");
    lcd_put_cursor(1, 7);
    snprintf(bufpm25, sizeof(bufpm25), "%.3f    ", avg_pm25);
    lcd_send_string(bufpm25);
    // ---- Compute Rs/R0 ----
    uint16_t adc = adc1_get_raw(MQ2_CHANNEL);
    float Vout = (adc / 4095.0f) * 3.3f;
    float RS = RL_VALUE * (3.3f - Vout) / Vout;
    float ratio = RS / get_R0();
    // ---- Update RainMaker ----
    esp_rmaker_param_update_and_report(param_ppm,   esp_rmaker_float(avg_ppm));
    esp_rmaker_param_update_and_report(param_pm25,  esp_rmaker_float(avg_pm25));  // NEW
    esp_rmaker_param_update_and_report(param_power, esp_rmaker_bool(alert_mode_enabled));
    esp_rmaker_param_update_and_report(param_ratio, esp_rmaker_float(ratio));
    // Status message (CO-based + PM2.5-based)
    char status_msg[64];
    int co_level = 0;
    if (avg_ppm < 4.5) {
        snprintf(status_msg, sizeof(status_msg), "CO tốt.");
        co_level = 0;
    } else if (avg_ppm < 9.5) {
        snprintf(status_msg, sizeof(status_msg), "CO trung bình.");
        co_level = 1;
    } else if (avg_ppm < 12.5) {
        snprintf(status_msg, sizeof(status_msg), "CO không tốt.");
        co_level = 2;
    } else if (avg_ppm < 15.5) {
        snprintf(status_msg, sizeof(status_msg), "CO xấu. Cẩn thận!");
        co_level = 3;
    } else {
        snprintf(status_msg, sizeof(status_msg), "CO rất xấu! NGUY HIỂM!");
        co_level = 4;
    }
    // NEW: PM2.5 Status Message
    char status2_msg[64];
    int pm_level = 0;
    if (avg_pm25 < 12) {
        snprintf(status2_msg, sizeof(status2_msg), "PM2.5 tốt.");
        pm_level = 0;
    } else if (avg_pm25 < 35.4) {
        snprintf(status2_msg, sizeof(status2_msg), "PM2.5 an toàn.");
        pm_level = 1;
    } else if (avg_pm25 < 55.4) {
        snprintf(status2_msg, sizeof(status2_msg), "PM2.5 trung bình.");
        pm_level = 2;
    } else if (avg_pm25 < 150.4) {
        snprintf(status2_msg, sizeof(status2_msg), "PM2.5 kém.");
        pm_level = 3;
    } else {
        snprintf(status2_msg, sizeof(status2_msg), "PM2.5 rất xấu!");
        pm_level = 4;
    }
    // Cảnh báo LED đỏ dựa trên mức độ nguy hiểm cao hơn:
    int danger = (co_level > pm_level) ? co_level : pm_level;
    if (alert_mode_enabled) {
        int duty_table[] = {0, 255/16, 255/8, 255/4, 255};
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, duty_table[danger]);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        if (danger >= 3) {
            gpio_set_level(BUZZ_PIN, 1); // Bật Buzz
            esp_rom_delay_us(100000);
            gpio_set_level(BUZZ_PIN, 0); // Tắt Buzz
            esp_rom_delay_us(100000);
        } else {
            gpio_set_level(BUZZ_PIN, 0); // Tắt Buzz
        }
    } else {
        ledc_set_duty(LEDC_MODE, LEDC_CHANNEL, 0);
        ledc_update_duty(LEDC_MODE, LEDC_CHANNEL);
        gpio_set_level(BUZZ_PIN, 0);
    }
    // Cập nhật trạng thái CO và PM2.5:
    esp_rmaker_param_update_and_report(param_status, esp_rmaker_str(status_msg));
    esp_rmaker_param_update_and_report(param25_status, esp_rmaker_str(status2_msg));
}

// =========================================================
//                        MAIN
// =========================================================
void app_main(void) {
    // Bắt đầu ứng dụng:
    ESP_LOGI(TAG, "Starting MQ2 + PM2.5 + LCD + LED + RainMaker");
    // ---- NVS ----
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES ||
        err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ESP_ERROR_CHECK(nvs_flash_init());
    }
    // ---- WiFi + RainMaker Base ----
    app_network_init();
    esp_rmaker_config_t cfg = { .enable_time_sync = true };
    esp_rmaker_node_t *node = esp_rmaker_node_init(
        &cfg, "CO and PM2.5 monitor", "Sensor");
    dev_mq2 = esp_rmaker_device_create("Máy đo CO và bụi PM2.5", "Sensor", NULL);
    esp_rmaker_device_add_bulk_cb(dev_mq2, bulk_write_cb, NULL);
    esp_rmaker_node_add_device(node, dev_mq2);
    // ---- Khởi tạo Param và thêm vào Rainmaker ----
    param_ppm       = esp_rmaker_param_create("Nồng độ CO (ppm):", "ppm",
                    esp_rmaker_float(0), PROP_FLAG_READ);
    param_pm25      = esp_rmaker_param_create("Nồng độ PM 2.5 (mg/m³):", "mg/m3",               
                    esp_rmaker_float(0), PROP_FLAG_READ);
    param_power     = esp_rmaker_power_param_create(
                    ESP_RMAKER_DEF_POWER_NAME, DEFAULT_POWER);
    param_ratio     = esp_rmaker_param_create("Rs/R0 cho cảm biến đo nồng độ CO:", "ratio",
                    esp_rmaker_float(0), PROP_FLAG_READ);
    param_status    = esp_rmaker_param_create(
                    "Trạng thái CO:", "co_status",
                    esp_rmaker_str("CO an toàn. Không thành vấn đề."),
                    PROP_FLAG_READ);
    param25_status  = esp_rmaker_param_create(                                          
                    "Trạng thái PM2.5:", "pm25_status",
                    esp_rmaker_str("PM2.5 an toàn. Không thành vấn đề."),
                    PROP_FLAG_READ);
    esp_rmaker_device_add_param(dev_mq2, param_ppm);
    esp_rmaker_device_add_param(dev_mq2, param_pm25);                                   
    esp_rmaker_device_add_param(dev_mq2, param_power);
    esp_rmaker_device_add_param(dev_mq2, param_ratio);
    esp_rmaker_device_add_param(dev_mq2, param_status);
    esp_rmaker_device_add_param(dev_mq2, param25_status);                               
    // ---- Start RainMaker ----
    esp_rmaker_ota_enable_default();
    esp_rmaker_start();
    if (app_network_start(POP_TYPE_RANDOM) != ESP_OK) {
        ESP_LOGE(TAG, "Failed WiFi provisioning");
        return;
    }
    // ---- Drivers ----
    app_driver_init();
    // ---- Button ----
    btn_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_reset_pin(ALERT_BUTTON_PIN);
    gpio_set_direction(ALERT_BUTTON_PIN, GPIO_MODE_INPUT);
    gpio_pullup_en(ALERT_BUTTON_PIN);
    gpio_set_intr_type(ALERT_BUTTON_PIN, GPIO_INTR_NEGEDGE);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(ALERT_BUTTON_PIN, button_isr, (void*)ALERT_BUTTON_PIN);
    xTaskCreate(button_task, "button_task", 4096, NULL, 10, NULL);
    // ---- Timers ----
    const esp_timer_create_args_t mq2_args = {
        .callback = measuring_timer_cb,
        .name = "mq2_timer"
        };
    esp_timer_handle_t mq2_timer;
    esp_timer_create(&mq2_args, &mq2_timer);
    esp_timer_start_periodic(mq2_timer, READ_INTERVAL_MS * 1000);
    const esp_timer_create_args_t rep_args = {
        .callback = report_timer_cb,
        .name = "report_timer"
    };
    esp_timer_handle_t rep_timer;
    esp_timer_create(&rep_args, &rep_timer);
    esp_timer_start_periodic(rep_timer, REPORT_INTERVAL_MS * 1000);
    // ---- PWM ----
    // Configure LEDC timer
    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .duty_resolution = LEDC_DUTY_RES,
        .timer_num = LEDC_TIMER,
        .freq_hz = LEDC_FREQUENCY
    };
    ledc_timer_config(&ledc_timer);
    // Configure LEDC channel
    ledc_channel_config_t ledc_channel = {
        .gpio_num = LED_WARNING_PIN,
        .speed_mode = LEDC_MODE,
        .channel = LEDC_CHANNEL,
        .timer_sel = LEDC_TIMER,
        .duty = 0
    };
    ledc_channel_config(&ledc_channel);
}
