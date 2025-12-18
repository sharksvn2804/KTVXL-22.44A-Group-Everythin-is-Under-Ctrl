#pragma once
#include <stdint.h>
#include <stdbool.h>
#include <driver/ledc.h>
#include "driver/gpio.h"
#include "driver/adc.h"

// ================= MQ2 + GP2Y + ADC =================
#define MQ2_CHANNEL       ADC1_CHANNEL_0        // GPIO0
#define PM25_ADC_CHANNEL  ADC1_CHANNEL_1        // GPIO1
#define RL_VALUE          5000
#define RATIO_CLEAN_AIR   3.0                   // Tỷ lệ Rs/R0 trong không khí sạch (được cung cấp bởi datasheet)

// ==================== GPIO + PWM ====================
// Định nghĩa các chân GPIO:
#define GP2Y_LED_POWER    19    // LED của GP2Y dùng để kích hoạt cảm biến bụi (dùng cho GP2Y1010AU0F)
#define LED_MODE_PIN      2     // LED xanh - chế độ cảnh báo
#define ALERT_BUTTON_PIN  18    // Nút GPIO18 để toggle alert mode từ button hoặc RainMaker
#define BUZZ_PIN          6     // Buzz cảnh báo
// GPIO cho PWM LED cảnh báo:
#define LED_WARNING_PIN   3     // LED đỏ 1 - cảnh báo khi CO hoặc PM2.5
#define LEDC_CHANNEL      LEDC_CHANNEL_0
#define LEDC_TIMER        LEDC_TIMER_0
#define LEDC_MODE         LEDC_LOW_SPEED_MODE
#define LEDC_DUTY_RES     LEDC_TIMER_8_BIT
#define LEDC_FREQUENCY    5000  // 5 kHz

// ==================== Functions ====================
void app_driver_init(void);
float read_CO_ppm(void);
float read_PM25_ppm(void);

// ==================== Getter ====================
float get_R0(void); // getter R0
