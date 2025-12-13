#include "app_priv.h"
#include "esp_log.h"
#include "driver/adc.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"
#include "esp_rom_sys.h"
#include "lcd_i2c.h"
#include <math.h>

static const char *TAG = "MQ2_DRIVER";
static float R0 = 0;

// ======== Calibration ========
static void calibrate_R0(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 50; i++) {
        sum += adc1_get_raw(MQ2_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(10));   // giữ nguyên
    }
    uint16_t avg_adc = sum / 50;
    float V_out = (avg_adc / 4095.0f) * 3.3f;
    float RS = RL_VALUE * (3.3f - V_out) / V_out;
    R0 = RS / RATIO_CLEAN_AIR;
    ESP_LOGI(TAG, "R0 calibrated: %.2f ohm", R0);
}

// ======== ADC Init ========
static void ADC_init(void) {
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_config_channel_atten(MQ2_CHANNEL, ADC_ATTEN_DB_11);
}

// ======== PPM Calculation ========
static float CO_ppm_calc(uint16_t adc_raw) {
    float A = 87.9054905f;
    float B = -1.289602592f;
    float V_out = (adc_raw / 4095.0f) * 3.3f;
    float RS = RL_VALUE * (3.3f - V_out) / V_out;
    float ratio = RS / R0;
    float ppm = A * powf(ratio, B);
    float offset = 17.0f;
    return (ppm > offset) ? (ppm - offset) : 0;
}

// ======== Read CO PPM ========
float read_CO_ppm(void) {
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += adc1_get_raw(MQ2_CHANNEL);
        esp_rom_delay_us(5000);    // 5 ms → không block scheduler
    }
    uint16_t adc_raw = sum / 10;
    return CO_ppm_calc(adc_raw);
}

// ======== PM2.5 Read ========
float read_PM25_ppm(void) {
    // Bật LED GP2Y trước khi đọc
    gpio_set_level(GP2Y_LED_POWER, 1);
    vTaskDelay(pdMS_TO_TICKS(0.28)); // ~280us datasheet
    uint32_t sum = 0;
    for (int i = 0; i < 10; i++) {
        sum += adc1_get_raw(PM25_ADC_CHANNEL);
        vTaskDelay(pdMS_TO_TICKS(2));
    }
    uint16_t adc_raw = sum / 10;
    gpio_set_level(GP2Y_LED_POWER, 0); // Tắt LED
    float V_out = (adc_raw / 4095.0f) * 3.3f;
    float dust_density = 0.17 * V_out * 1000; // ug/m³ (có thể hiệu chỉnh hệ số 0.17)
    if (dust_density < 0) dust_density = 0;
    return dust_density;
}

// ======== Getter ========
float get_R0(void) {
    return R0;
}

// ======== Driver Init ========
void app_driver_init(void) {
    // Thông báo khởi tạo driver:
    ESP_LOGI(TAG, "Initializing driver");
    // Khởi tạo GPIO:
    gpio_config_t led_cfg = {
        .pin_bit_mask = (1ULL << LED_WARNING_PIN) | (1ULL << LED_MODE_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&led_cfg);
    gpio_set_level(LED_WARNING_PIN, 0);
    gpio_set_level(LED_MODE_PIN, 0);
    // Khởi tạo GPIO cho còi:
    gpio_config_t buzz_cfg = {
        .pin_bit_mask = (1ULL << BUZZ_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE
    };
    gpio_config(&buzz_cfg);
    gpio_set_level(BUZZ_PIN, 0);
    // Khởi tạo LCD (tiêu chí I2C):
    lcd_init();
    lcd_clear();
    // Khởi tạo ADC:
    ADC_init();
    // Hiệu chuẩn R0 và thông báo ra LCD:
    lcd_put_cursor(0, 0);
    lcd_send_string("Calibrating...");
    vTaskDelay(pdMS_TO_TICKS(2000)); // giữ lại: init phase
    calibrate_R0();
    // Thông báo hệ thống sẵn sàng:
    lcd_clear();
    lcd_put_cursor(0, 0);
    lcd_send_string("System ready");
    vTaskDelay(pdMS_TO_TICKS(1000)); // giữ lại
    // Thông báo đã hoàn thành khởi tạo:
    lcd_clear();
    ESP_LOGI(TAG, "Driver init completed");
}
