#pragma once
#include <stdint.h>
#include <stdbool.h>
#include "driver/gpio.h"
#include "driver/adc.h"

// ================ MQ2 + GP2Y =================
#define MQ2_CHANNEL       ADC1_CHANNEL_0        // GPIO0
#define PM25_ADC_CHANNEL  ADC1_CHANNEL_1        // GPIO1
#define RL_VALUE          5000
#define RATIO_CLEAN_AIR   3.0                   // Tỷ lệ Rs/R0 trong không khí sạch (được cung cấp bởi datasheet)

// ==================== LED ====================
// GPIO Pins:
#define GP2Y_LED_POWER    19    // LED của GP2Y dùng để kích hoạt cảm biến bụi (dùng cho GP2Y1010AU0F)
#define LED_WARNING_PIN   3     // LED đỏ 1 - cảnh báo khi CO hoặc PM2.5 cao
#define LED_MODE_PIN      2     // LED xanh - chế độ cảnh báo
#define ALERT_BUTTON_PIN  18    // Nút GPIO18 để toggle alert mode từ button hoặc RainMaker
// CO Alert Mode:
#define CO_THRESHOLD      4.0   // ngưỡng cảnh báo CO (ppm)
#define PM25_THRESHOLD    35.0  // ngưỡng cảnh báo PM2.5 (mg/m³)

// ==================== Functions ====================
void app_driver_init(void);
float read_CO_ppm(void);
float read_PM25_ppm(void);

// ==================== Getter ====================
float get_R0(void); // getter R0