/*
 * Xiaomiao ESP32-WROVER-B LVGL 9.5 hardware pager.
 *
 * Board resources from README.md:
 *   ST7735-compatible SPI TFT, MicroSD on shared SPI2, 6 active-low keys,
 *   GPIO14 passive buzzer, GPIO36/GPIO39 ADC sensors, I2C0 GD32/MPU6050,
 *   and GPIO25/26/32/33 extension IO.
 */

#include <assert.h>
#include <math.h>
#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <unistd.h>

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "driver/ledc.h"
#include "driver/sdspi_host.h"
#include "driver/spi_master.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_chip_info.h"
#include "esp_err.h"
#include "esp_heap_caps.h"
#include "esp_idf_version.h"
#include "esp_lcd_panel_io.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_vfs_fat.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "lvgl.h"
#include "sdmmc_cmd.h"
#include "sdkconfig.h"

#ifndef CONFIG_IDF_TARGET
#define CONFIG_IDF_TARGET "esp32"
#endif

#ifndef CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ
#define CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ 240
#endif

#ifndef CONFIG_ESPTOOLPY_FLASHFREQ
#define CONFIG_ESPTOOLPY_FLASHFREQ "unknown"
#endif

#ifndef CONFIG_ESPTOOLPY_FLASHSIZE
#define CONFIG_ESPTOOLPY_FLASHSIZE "unknown"
#endif

#ifndef CONFIG_ESPTOOLPY_FLASHMODE
#define CONFIG_ESPTOOLPY_FLASHMODE "unknown"
#endif

#if CONFIG_ESPTOOLPY_FLASHMODE_QIO
#define UI_FLASH_MODE "QIO"
#elif CONFIG_ESPTOOLPY_FLASHMODE_QOUT
#define UI_FLASH_MODE "QOUT"
#elif CONFIG_ESPTOOLPY_FLASHMODE_DIO
#define UI_FLASH_MODE "DIO"
#elif CONFIG_ESPTOOLPY_FLASHMODE_DOUT
#define UI_FLASH_MODE "DOUT"
#else
#define UI_FLASH_MODE CONFIG_ESPTOOLPY_FLASHMODE
#endif

#if CONFIG_ESPTOOLPY_FLASHFREQ_80M
#define UI_FLASH_FREQ "80 MHz"
#elif CONFIG_ESPTOOLPY_FLASHFREQ_40M
#define UI_FLASH_FREQ "40 MHz"
#elif CONFIG_ESPTOOLPY_FLASHFREQ_26M
#define UI_FLASH_FREQ "26 MHz"
#elif CONFIG_ESPTOOLPY_FLASHFREQ_20M
#define UI_FLASH_FREQ "20 MHz"
#else
#define UI_FLASH_FREQ CONFIG_ESPTOOLPY_FLASHFREQ
#endif

#if CONFIG_ESPTOOLPY_FLASHSIZE_4MB
#define UI_FLASH_SIZE "4 MB"
#elif CONFIG_ESPTOOLPY_FLASHSIZE_2MB
#define UI_FLASH_SIZE "2 MB"
#elif CONFIG_ESPTOOLPY_FLASHSIZE_8MB
#define UI_FLASH_SIZE "8 MB"
#elif CONFIG_ESPTOOLPY_FLASHSIZE_16MB
#define UI_FLASH_SIZE "16 MB"
#else
#define UI_FLASH_SIZE CONFIG_ESPTOOLPY_FLASHSIZE
#endif

#if CONFIG_IDF_TARGET_ESP32
#define UI_TARGET_NAME "ESP32"
#else
#define UI_TARGET_NAME CONFIG_IDF_TARGET
#endif

#ifndef CONFIG_SPIRAM_SPEED
#define CONFIG_SPIRAM_SPEED 0
#endif

#define LCD_HOST                    SPI2_HOST
#define LCD_PIXEL_CLOCK_HZ          (60 * 1000 * 1000)
#define LCD_NATIVE_H_RES            128
#define LCD_NATIVE_V_RES            160
#define LCD_H_RES                   160
#define LCD_V_RES                   128
#define LCD_DRAW_BUF_LINES          LCD_V_RES
#define LCD_DRAW_BUF_COUNT          3
#define LCD_DPI                     60
#define LCD_CMD_BITS                8
#define LCD_PARAM_BITS              8

#define PIN_NUM_LCD_SCLK            GPIO_NUM_18
#define PIN_NUM_LCD_MOSI            GPIO_NUM_23
#define PIN_NUM_LCD_MISO            GPIO_NUM_19
#define PIN_NUM_LCD_CS              GPIO_NUM_5
#define PIN_NUM_LCD_DC              GPIO_NUM_4
#define PIN_NUM_SD_CS               GPIO_NUM_22
#define PIN_NUM_BUZZER              GPIO_NUM_14
#define PIN_NUM_I2C_SCL             GPIO_NUM_15
#define PIN_NUM_I2C_SDA             GPIO_NUM_21
#define PIN_NUM_EXT_OUT1            GPIO_NUM_25
#define PIN_NUM_EXT_OUT2            GPIO_NUM_26
#define PIN_NUM_EXT_IN1             GPIO_NUM_32
#define PIN_NUM_EXT_IN2             GPIO_NUM_33

#define LCD_X_GAP                   0
#define LCD_Y_GAP                   0

#define LVGL_TICK_PERIOD_MS         1
#define LVGL_TASK_STACK_SIZE        (10 * 1024)
#define LVGL_TASK_PRIORITY          5
#define LVGL_TASK_MIN_DELAY_MS      1
#define LVGL_TASK_MAX_DELAY_MS      16

#define BUTTON_ACTIVE_LEVEL         0
#define BUTTON_DEBOUNCE_MS          25
#define UI_REFRESH_PERIOD_MS        16
#define UI_ACTION_MSG_MS            850
#define UI_HISTORY_POINTS           48
#define LIGHT_HISTORY_AVG_SAMPLES   4
#define THERM_UPDATE_PERIOD_MS      1000
#define THERM_HISTORY_MIN_PCT       35
#define THERM_HISTORY_MAX_PCT       65
#define I2C_TIMEOUT_MS              30
#define I2C_FREQ_HZ                 100000
#define GD32_REPROBE_PERIOD_MS      1500
#define MPU_REPROBE_PERIOD_MS       1500
#define SD_SPI_MAX_FREQ_KHZ         10000

#define GD32_ADDR                   0x40
#define GD32_LED1_REG               0xA0
#define GD32_LED2_REG               0xA1
#define GD32_MOTOR1_REG             0x0E
#define GD32_MOTOR2_REG             0x06

#define MPU6050_ADDR                0x68
#define MPU6050_REG_ACCEL_XOUT_H    0x3B
#define MPU6050_REG_PWR_MGMT_1      0x6B
#define MPU6050_REG_WHO_AM_I        0x75
#define MPU6050_WHO_AM_I_VALUE      0x68

#define ADC_LIGHT_CHAN              ADC_CHANNEL_0
#define ADC_TEMP_CHAN               ADC_CHANNEL_3
#define ADC_EXT_IN1_CHAN            ADC_CHANNEL_4
#define ADC_EXT_IN2_CHAN            ADC_CHANNEL_5
#define ADC_RAW_MAX                 4095

#define BUZZER_LEDC_MODE            LEDC_LOW_SPEED_MODE
#define BUZZER_LEDC_TIMER           LEDC_TIMER_0
#define BUZZER_LEDC_CHANNEL         LEDC_CHANNEL_0
#define BUZZER_DUTY                 128

#define EXT_LEDC_TIMER              LEDC_TIMER_1
#define EXT_LEDC_CHANNEL1           LEDC_CHANNEL_1
#define EXT_LEDC_CHANNEL2           LEDC_CHANNEL_2
#define EXT_PWM_FREQ_HZ             1000
#define EXT_PWM_DUTY_MAX            255

#define ST7735_SWRESET              0x01
#define ST7735_SLPOUT               0x11
#define ST7735_NORON                0x13
#define ST7735_INVOFF               0x20
#define ST7735_DISPOFF              0x28
#define ST7735_DISPON               0x29
#define ST7735_CASET                0x2A
#define ST7735_RASET                0x2B
#define ST7735_RAMWR                0x2C
#define ST7735_MADCTL               0x36
#define ST7735_COLMOD               0x3A
#define ST7735_FRMCTR1              0xB1
#define ST7735_FRMCTR2              0xB2
#define ST7735_FRMCTR3              0xB3
#define ST7735_INVCTR               0xB4
#define ST7735_PWCTR1               0xC0
#define ST7735_PWCTR2               0xC1
#define ST7735_PWCTR3               0xC2
#define ST7735_PWCTR4               0xC3
#define ST7735_PWCTR5               0xC4
#define ST7735_VMCTR1               0xC5
#define ST7735_GMCTRP1              0xE0
#define ST7735_GMCTRN1              0xE1

#define MADCTL_MY                   0x80
#define MADCTL_MX                   0x40
#define MADCTL_MV                   0x20
#define MADCTL_RGB                  0x00

typedef struct {
    gpio_num_t gpio;
    uint32_t key;
    const char *name;
} board_button_t;

typedef enum {
    UI_PAGE_LIGHT = 0,
    UI_PAGE_THERM,
    UI_PAGE_MOTION,
    UI_PAGE_LED1,
    UI_PAGE_LED2,
    UI_PAGE_BUZZER,
    UI_PAGE_MOTOR1,
    UI_PAGE_MOTOR2,
    UI_PAGE_SD,
    UI_PAGE_GPIO25,
    UI_PAGE_GPIO26,
    UI_PAGE_ADC32,
    UI_PAGE_ADC33,
    UI_PAGE_SYSTEM,
    UI_PAGE_ABOUT,
    UI_PAGE_COUNT,
} ui_page_t;

typedef struct {
    bool i2c_ready;
    bool gd32_present;
    bool mpu_present;
    bool sd_mounted;
    bool buzzer_ready;
    bool adc_ready;
    bool ext_pwm_ready;
    bool led1_on;
    bool led2_on;
    bool motor_running[2];
    bool motor_dir[2];
    bool ext_out[2];
    uint8_t motor_speed[2];
    uint8_t ext_pwm[2];
    uint8_t mpu_whoami;
    uint32_t samples;
    int light_raw;
    int temp_raw;
    int ext_raw[2];
    int16_t acc[3];
    int16_t gyro[3];
    float pitch;
    float roll;
    char gesture[12];
    char sd_name[24];
    uint32_t sd_mb;
    esp_err_t last_adc_err;
    esp_err_t last_gd32_err;
    esp_err_t last_mpu_err;
    esp_err_t last_sd_err;
    char action[32];
} board_state_t;

typedef struct {
    lv_obj_t *screen;
    lv_obj_t *page;
    lv_obj_t *title;
    lv_obj_t *value;
    lv_obj_t *sub;
    lv_obj_t *bar;
    lv_obj_t *chart;
    lv_obj_t *chart_head;
    lv_obj_t *status;
    lv_obj_t *hint;
    lv_obj_t *accent;
    lv_chart_series_t *chart_series;
    uint32_t chart_history_version;
    lv_group_t *group;
    ui_page_t page_id;
} ui_state_t;

static const char *TAG = "xiaomiao_dash";

static const board_button_t s_buttons[] = {
    {GPIO_NUM_2, LV_KEY_UP, "UP"},
    {GPIO_NUM_13, LV_KEY_DOWN, "DOWN"},
    {GPIO_NUM_27, LV_KEY_LEFT, "LEFT"},
    {GPIO_NUM_35, LV_KEY_RIGHT, "RIGHT"},
    {GPIO_NUM_34, LV_KEY_ENTER, "A"},
    {GPIO_NUM_12, LV_KEY_ESC, "B"},
};

static lv_draw_buf_t s_draw_buf3;
static ui_state_t s_ui;
static board_state_t s_board = {
    .gesture = "ABSENT",
    .sd_name = "NO CARD",
    .motor_speed = {120, 120},
    .ext_pwm = {128, 128},
    .last_sd_err = ESP_ERR_NOT_FOUND,
    .action = "Ready",
};

static adc_oneshot_unit_handle_t s_adc_handle;
static esp_lcd_panel_io_handle_t s_lcd_io_handle;
static i2c_master_bus_handle_t s_i2c_bus;
static i2c_master_dev_handle_t s_gd32_dev;
static i2c_master_dev_handle_t s_mpu_dev;
static sdmmc_card_t *s_sd_card;
static uint32_t s_buzzer_stop_at;
static uint32_t s_buzzer_freq_hz = 988;
static uint32_t s_action_until_ms;
static uint32_t s_last_gd32_probe_ms;
static uint32_t s_last_mpu_probe_ms;
static bool s_gd32_probe_seen;
static bool s_mpu_probe_seen;
static bool s_lcd_display_on;
static volatile bool s_lcd_first_flush_done;
static int32_t s_light_history[UI_HISTORY_POINTS];
static int32_t s_therm_history[UI_HISTORY_POINTS];
static uint32_t s_light_history_version;
static uint32_t s_therm_history_version;
static uint32_t s_light_hist_accum;
static uint8_t s_light_hist_count;
static uint32_t s_therm_accum;
static uint16_t s_therm_accum_count;
static uint32_t s_last_therm_publish_ms;

static int pct_from_raw(int raw)
{
    raw = MAX(0, MIN(raw, ADC_RAW_MAX));
    return (raw * 100) / ADC_RAW_MAX;
}

static void sensor_history_init(void)
{
    for (size_t i = 0; i < UI_HISTORY_POINTS; ++i) {
        s_light_history[i] = LV_CHART_POINT_NONE;
        s_therm_history[i] = LV_CHART_POINT_NONE;
    }
    s_light_history_version = 0;
    s_therm_history_version = 0;
    s_light_hist_accum = 0;
    s_light_hist_count = 0;
    s_therm_accum = 0;
    s_therm_accum_count = 0;
    s_last_therm_publish_ms = 0;
}

static void sensor_history_push(int32_t *history, uint32_t *version, int value)
{
    for (size_t i = 1; i < UI_HISTORY_POINTS; ++i) {
        history[i - 1] = history[i];
    }
    history[UI_HISTORY_POINTS - 1] = MAX(0, MIN(value, 100));
    (*version)++;
}

static void sensor_history_push_range(int32_t *history, uint32_t *version, int value, int min_value, int max_value)
{
    sensor_history_push(history, version, MAX(min_value, MIN(value, max_value)));
}

static int16_t i16_be(const uint8_t *p)
{
    return (int16_t)((uint16_t)p[0] << 8 | p[1]);
}

static void copy_text(char *dst, size_t dst_size, const char *src)
{
    if (dst_size == 0) {
        return;
    }
    snprintf(dst, dst_size, "%s", src ? src : "");
}

static void set_action(const char *msg)
{
    copy_text(s_board.action, sizeof(s_board.action), msg);
    s_action_until_ms = lv_tick_get() + UI_ACTION_MSG_MS;
}

static esp_err_t i2c_write(i2c_master_dev_handle_t dev, const uint8_t *data, size_t len)
{
    if (!dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit(dev, data, len, I2C_TIMEOUT_MS);
}

static esp_err_t i2c_write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t value)
{
    const uint8_t data[] = {reg, value};
    return i2c_write(dev, data, sizeof(data));
}

static esp_err_t i2c_read_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *data, size_t len)
{
    if (!dev) {
        return ESP_ERR_INVALID_STATE;
    }
    return i2c_master_transmit_receive(dev, &reg, 1, data, len, I2C_TIMEOUT_MS);
}

static void gd32_mark_absent(void)
{
    s_board.gd32_present = false;
    s_board.led1_on = false;
    s_board.led2_on = false;
    s_board.motor_running[0] = false;
    s_board.motor_running[1] = false;
}

static esp_err_t gd32_write_reg(uint8_t reg, uint8_t value)
{
    esp_err_t err = i2c_write_reg(s_gd32_dev, reg, value);
    s_board.last_gd32_err = err;
    if (err == ESP_OK) {
        s_board.gd32_present = true;
    }
    else {
        gd32_mark_absent();
    }
    return err;
}

static esp_err_t gd32_motor_stop_all(void)
{
    const uint8_t data[] = {0x00, 0x00, 0x00, 0x00, 0x00};
    esp_err_t err = i2c_write(s_gd32_dev, data, sizeof(data));
    s_board.last_gd32_err = err;
    if (err == ESP_OK) {
        s_board.gd32_present = true;
        s_board.motor_running[0] = false;
        s_board.motor_running[1] = false;
    }
    else {
        gd32_mark_absent();
    }
    return err;
}

static esp_err_t gd32_motor_set(uint8_t motor, bool dir, uint8_t speed)
{
    const uint8_t reg = (motor == 0) ? GD32_MOTOR1_REG : GD32_MOTOR2_REG;
    const uint16_t pwm = ((uint16_t)speed) << 4;
    const uint8_t pwm_l = pwm & 0xFF;
    const uint8_t pwm_h = pwm >> 8;
    uint8_t data[9] = {reg, 0, 0, 0, 0, 0, 0, 0, 0};

    if (dir) {
        data[3] = pwm_l;
        data[4] = pwm_h;
    }
    else {
        data[7] = pwm_l;
        data[8] = pwm_h;
    }

    esp_err_t err = i2c_write(s_gd32_dev, data, sizeof(data));
    s_board.last_gd32_err = err;
    if (err == ESP_OK) {
        s_board.gd32_present = true;
        s_board.motor_running[motor] = speed > 0;
    }
    else {
        gd32_mark_absent();
    }
    return err;
}

static void buzzer_stop(void)
{
    if (!s_board.buzzer_ready) {
        return;
    }
    ledc_stop(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, 0);
    s_buzzer_stop_at = 0;
}

static void buzzer_beep(uint32_t freq_hz, uint32_t ms)
{
    if (!s_board.buzzer_ready) {
        return;
    }
    esp_err_t err = ledc_set_freq(BUZZER_LEDC_MODE, BUZZER_LEDC_TIMER, freq_hz);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buzzer frequency %lu Hz failed: %s", (unsigned long)freq_hz, esp_err_to_name(err));
        return;
    }
    err = ledc_set_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL, BUZZER_DUTY);
    if (err == ESP_OK) {
        err = ledc_update_duty(BUZZER_LEDC_MODE, BUZZER_LEDC_CHANNEL);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buzzer duty update failed: %s", esp_err_to_name(err));
        return;
    }
    s_buzzer_stop_at = lv_tick_get() + ms;
}

static void hardware_process_timers(void)
{
    uint32_t now = lv_tick_get();

    if (s_buzzer_stop_at && (int32_t)(now - s_buzzer_stop_at) >= 0) {
        buzzer_stop();
    }
}

static void mpu_probe_and_init(bool force)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (!force && s_mpu_probe_seen && now_ms - s_last_mpu_probe_ms < MPU_REPROBE_PERIOD_MS) {
        return;
    }
    s_mpu_probe_seen = true;
    s_last_mpu_probe_ms = now_ms;

    s_board.mpu_present = false;
    s_board.mpu_whoami = 0;
    copy_text(s_board.gesture, sizeof(s_board.gesture), "ABSENT");

    if (!s_i2c_bus || !s_mpu_dev) {
        s_board.last_mpu_err = ESP_ERR_INVALID_STATE;
        return;
    }

    esp_err_t err = i2c_master_probe(s_i2c_bus, MPU6050_ADDR, I2C_TIMEOUT_MS);
    if (err != ESP_OK) {
        s_board.last_mpu_err = err;
        return;
    }

    uint8_t who = 0;
    err = i2c_read_reg(s_mpu_dev, MPU6050_REG_WHO_AM_I, &who, 1);
    if (err != ESP_OK) {
        s_board.last_mpu_err = err;
        return;
    }

    s_board.mpu_whoami = who;
    if (who != MPU6050_WHO_AM_I_VALUE) {
        s_board.last_mpu_err = ESP_ERR_INVALID_ARG;
        return;
    }

    err = i2c_write_reg(s_mpu_dev, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        s_board.last_mpu_err = err;
        return;
    }

    s_board.mpu_present = true;
    s_board.last_mpu_err = ESP_OK;
    copy_text(s_board.gesture, sizeof(s_board.gesture), "READY");
}

static void mpu_read(void)
{
    if (!s_board.mpu_present) {
        mpu_probe_and_init(false);
        return;
    }

    uint8_t data[14] = {0};
    esp_err_t err = i2c_read_reg(s_mpu_dev, MPU6050_REG_ACCEL_XOUT_H, data, sizeof(data));
    s_board.last_mpu_err = err;
    if (err != ESP_OK) {
        s_board.mpu_present = false;
        copy_text(s_board.gesture, sizeof(s_board.gesture), "ABSENT");
        return;
    }

    s_board.acc[0] = i16_be(&data[0]);
    s_board.acc[1] = i16_be(&data[2]);
    s_board.acc[2] = i16_be(&data[4]);
    s_board.gyro[0] = i16_be(&data[8]);
    s_board.gyro[1] = i16_be(&data[10]);
    s_board.gyro[2] = i16_be(&data[12]);

    const float ax = s_board.acc[0] / 16384.0f;
    const float ay = s_board.acc[1] / 16384.0f;
    const float az = s_board.acc[2] / 16384.0f;
    s_board.pitch = atan2f(-ax, sqrtf(ay * ay + az * az)) * 57.29578f;
    s_board.roll = atan2f(ay, az) * 57.29578f;

    if (s_board.pitch > 25.0f) {
        copy_text(s_board.gesture, sizeof(s_board.gesture), "TILT UP");
    }
    else if (s_board.pitch < -25.0f) {
        copy_text(s_board.gesture, sizeof(s_board.gesture), "TILT DN");
    }
    else if (s_board.roll > 25.0f) {
        copy_text(s_board.gesture, sizeof(s_board.gesture), "TILT R");
    }
    else if (s_board.roll < -25.0f) {
        copy_text(s_board.gesture, sizeof(s_board.gesture), "TILT L");
    }
    else {
        copy_text(s_board.gesture, sizeof(s_board.gesture), "LEVEL");
    }
}

static esp_err_t adc_read_one(adc_channel_t channel, int *raw)
{
    esp_err_t err = adc_oneshot_read(s_adc_handle, channel, raw);
    if (err != ESP_OK && s_board.last_adc_err == ESP_OK) {
        s_board.last_adc_err = err;
    }
    return err;
}

static esp_err_t adc_read_sensors(void)
{
    if (!s_board.adc_ready) {
        if (s_board.last_adc_err == ESP_OK) {
            s_board.last_adc_err = ESP_ERR_INVALID_STATE;
        }
        return s_board.last_adc_err;
    }

    if (!s_adc_handle) {
        s_board.last_adc_err = ESP_ERR_INVALID_STATE;
        return s_board.last_adc_err;
    }

    int raw = 0;
    s_board.last_adc_err = ESP_OK;
    if (adc_read_one(ADC_LIGHT_CHAN, &raw) == ESP_OK) {
        s_board.light_raw = raw;
        s_light_hist_accum += pct_from_raw(raw);
        s_light_hist_count++;
        if (s_light_hist_count >= LIGHT_HISTORY_AVG_SAMPLES) {
            sensor_history_push(s_light_history,
                                &s_light_history_version,
                                (int)((s_light_hist_accum + s_light_hist_count / 2) / s_light_hist_count));
            s_light_hist_accum = 0;
            s_light_hist_count = 0;
        }
    }
    if (adc_read_one(ADC_TEMP_CHAN, &raw) == ESP_OK) {
        const uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);
        s_therm_accum += raw;
        s_therm_accum_count++;
        if (s_last_therm_publish_ms == 0 || now_ms - s_last_therm_publish_ms >= THERM_UPDATE_PERIOD_MS) {
            const int avg_raw = (int)((s_therm_accum + s_therm_accum_count / 2) / s_therm_accum_count);
            s_board.temp_raw = avg_raw;
            sensor_history_push_range(s_therm_history,
                                      &s_therm_history_version,
                                      pct_from_raw(avg_raw),
                                      THERM_HISTORY_MIN_PCT,
                                      THERM_HISTORY_MAX_PCT);
            s_therm_accum = 0;
            s_therm_accum_count = 0;
            s_last_therm_publish_ms = now_ms;
        }
    }
    if (adc_read_one(ADC_EXT_IN1_CHAN, &raw) == ESP_OK) {
        s_board.ext_raw[0] = raw;
    }
    if (adc_read_one(ADC_EXT_IN2_CHAN, &raw) == ESP_OK) {
        s_board.ext_raw[1] = raw;
    }
    return s_board.last_adc_err;
}

static ledc_channel_t ext_pwm_channel(uint8_t index)
{
    return index == 0 ? EXT_LEDC_CHANNEL1 : EXT_LEDC_CHANNEL2;
}

static esp_err_t ext_output_set(uint8_t index, bool on)
{
    if (index >= 2 || !s_board.ext_pwm_ready) {
        return ESP_ERR_INVALID_STATE;
    }

    const ledc_channel_t channel = ext_pwm_channel(index);
    const uint32_t duty = on ? s_board.ext_pwm[index] : 0;

    esp_err_t err = ledc_set_duty(BUZZER_LEDC_MODE, channel, duty);
    if (err == ESP_OK) {
        err = ledc_update_duty(BUZZER_LEDC_MODE, channel);
    }
    if (err == ESP_OK) {
        s_board.ext_out[index] = on && duty > 0;
    }
    return err;
}

static void gd32_probe(bool force)
{
    uint32_t now_ms = (uint32_t)(esp_timer_get_time() / 1000);

    if (!force && s_gd32_probe_seen && now_ms - s_last_gd32_probe_ms < GD32_REPROBE_PERIOD_MS) {
        return;
    }
    s_gd32_probe_seen = true;
    s_last_gd32_probe_ms = now_ms;

    if (!s_i2c_bus || !s_gd32_dev) {
        s_board.last_gd32_err = ESP_ERR_INVALID_STATE;
        s_board.gd32_present = false;
    }
    else {
        s_board.last_gd32_err = i2c_master_probe(s_i2c_bus, GD32_ADDR, I2C_TIMEOUT_MS);
        s_board.gd32_present = (s_board.last_gd32_err == ESP_OK);
    }

    if (!s_board.gd32_present) {
        gd32_mark_absent();
    }
}

static void i2c_probe_devices(bool force)
{
    if (!s_i2c_bus) {
        s_board.i2c_ready = false;
        s_board.gd32_present = false;
        s_board.mpu_present = false;
        s_board.last_gd32_err = ESP_ERR_INVALID_STATE;
        s_board.last_mpu_err = ESP_ERR_INVALID_STATE;
        return;
    }

    s_board.i2c_ready = true;
    gd32_probe(force);

    if (force || !s_board.mpu_present) {
        mpu_probe_and_init(force);
    }
}

static void hardware_update(void)
{
    (void)adc_read_sensors();
    i2c_probe_devices(false);
    mpu_read();
    s_board.samples++;
}

static void sd_try_mount(void)
{
    if (s_board.sd_mounted) {
        return;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = LCD_HOST;
    host.max_freq_khz = SD_SPI_MAX_FREQ_KHZ;

    sdspi_device_config_t slot_config = SDSPI_DEVICE_CONFIG_DEFAULT();
    slot_config.host_id = LCD_HOST;
    slot_config.gpio_cs = PIN_NUM_SD_CS;
    slot_config.wait_for_miso = 20;

    esp_vfs_fat_mount_config_t mount_config = VFS_FAT_MOUNT_DEFAULT_CONFIG();
    mount_config.format_if_mount_failed = false;
    mount_config.max_files = 3;

    s_board.last_sd_err = esp_vfs_fat_sdspi_mount("/sdcard",
                                                  &host,
                                                  &slot_config,
                                                  &mount_config,
                                                  &s_sd_card);
    if (s_board.last_sd_err == ESP_OK && s_sd_card) {
        s_board.sd_mounted = true;
        memset(s_board.sd_name, 0, sizeof(s_board.sd_name));
        memcpy(s_board.sd_name,
               s_sd_card->cid.name,
               MIN(sizeof(s_sd_card->cid.name), sizeof(s_board.sd_name) - 1));
        s_board.sd_mb = (uint32_t)(((uint64_t)s_sd_card->csd.capacity * s_sd_card->csd.sector_size) / (1024 * 1024));
    }
    else {
        s_board.sd_mounted = false;
        s_sd_card = NULL;
        copy_text(s_board.sd_name, sizeof(s_board.sd_name), "NO CARD");
        s_board.sd_mb = 0;
    }
}

static esp_err_t sd_unmount(void)
{
    esp_err_t err = ESP_ERR_NOT_FOUND;

    if (s_board.sd_mounted && s_sd_card) {
        err = esp_vfs_fat_sdcard_unmount("/sdcard", s_sd_card);
        if (err != ESP_OK) {
            s_board.last_sd_err = err;
            set_action("SD unmount fail");
            return err;
        }
    }
    else {
        set_action("No SD card");
        s_board.last_sd_err = err;
        return err;
    }

    s_board.sd_mounted = false;
    s_sd_card = NULL;
    copy_text(s_board.sd_name, sizeof(s_board.sd_name), "NO CARD");
    s_board.sd_mb = 0;
    s_board.last_sd_err = ESP_ERR_NOT_FOUND;
    set_action("SD unmounted");
    return ESP_OK;
}

static void adc_init(void)
{
    adc_oneshot_unit_init_cfg_t init_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    esp_err_t err = adc_oneshot_new_unit(&init_cfg, &s_adc_handle);
    if (err != ESP_OK) {
        s_board.last_adc_err = err;
        ESP_LOGW(TAG, "ADC init failed: %s", esp_err_to_name(err));
        return;
    }

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    err = adc_oneshot_config_channel(s_adc_handle, ADC_LIGHT_CHAN, &chan_cfg);
    if (err == ESP_OK) {
        err = adc_oneshot_config_channel(s_adc_handle, ADC_TEMP_CHAN, &chan_cfg);
    }
    if (err == ESP_OK) {
        err = adc_oneshot_config_channel(s_adc_handle, ADC_EXT_IN1_CHAN, &chan_cfg);
    }
    if (err == ESP_OK) {
        err = adc_oneshot_config_channel(s_adc_handle, ADC_EXT_IN2_CHAN, &chan_cfg);
    }
    if (err != ESP_OK) {
        s_board.last_adc_err = err;
        ESP_LOGW(TAG, "ADC channel config failed: %s", esp_err_to_name(err));
        return;
    }
    s_board.last_adc_err = ESP_OK;
    s_board.adc_ready = true;
}

static void ext_io_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = EXT_LEDC_TIMER,
        .freq_hz = EXT_PWM_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Extension PWM timer init failed: %s", esp_err_to_name(err));
        return;
    }

    const ledc_channel_config_t channel_cfg[] = {
        {
            .gpio_num = PIN_NUM_EXT_OUT1,
            .speed_mode = BUZZER_LEDC_MODE,
            .channel = EXT_LEDC_CHANNEL1,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = EXT_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        },
        {
            .gpio_num = PIN_NUM_EXT_OUT2,
            .speed_mode = BUZZER_LEDC_MODE,
            .channel = EXT_LEDC_CHANNEL2,
            .intr_type = LEDC_INTR_DISABLE,
            .timer_sel = EXT_LEDC_TIMER,
            .duty = 0,
            .hpoint = 0,
            .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
        },
    };

    err = ledc_channel_config(&channel_cfg[0]);
    if (err == ESP_OK) {
        err = ledc_channel_config(&channel_cfg[1]);
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Extension PWM channel init failed: %s", esp_err_to_name(err));
        return;
    }
    s_board.ext_pwm_ready = true;
    (void)ext_output_set(0, false);
    (void)ext_output_set(1, false);
}

static void i2c_init(void)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = PIN_NUM_I2C_SDA,
        .scl_io_num = PIN_NUM_I2C_SCL,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    esp_err_t err = i2c_new_master_bus(&bus_cfg, &s_i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    const i2c_device_config_t gd32_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = GD32_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    const i2c_device_config_t mpu_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = MPU6050_ADDR,
        .scl_speed_hz = I2C_FREQ_HZ,
    };
    s_board.i2c_ready = true;

    err = i2c_master_bus_add_device(s_i2c_bus, &gd32_cfg, &s_gd32_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GD32 I2C device add failed: %s", esp_err_to_name(err));
        s_board.last_gd32_err = err;
        s_gd32_dev = NULL;
    }

    err = i2c_master_bus_add_device(s_i2c_bus, &mpu_cfg, &s_mpu_dev);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "MPU I2C device add failed: %s", esp_err_to_name(err));
        s_board.last_mpu_err = err;
        s_mpu_dev = NULL;
    }

    i2c_probe_devices(true);
}

static void buzzer_init(void)
{
    ledc_timer_config_t timer_cfg = {
        .speed_mode = BUZZER_LEDC_MODE,
        .duty_resolution = LEDC_TIMER_8_BIT,
        .timer_num = BUZZER_LEDC_TIMER,
        .freq_hz = 880,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    esp_err_t err = ledc_timer_config(&timer_cfg);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Buzzer timer init failed: %s", esp_err_to_name(err));
        return;
    }

    ledc_channel_config_t channel_cfg = {
        .gpio_num = PIN_NUM_BUZZER,
        .speed_mode = BUZZER_LEDC_MODE,
        .channel = BUZZER_LEDC_CHANNEL,
        .intr_type = LEDC_INTR_DISABLE,
        .timer_sel = BUZZER_LEDC_TIMER,
        .duty = 0,
        .hpoint = 0,
        .sleep_mode = LEDC_SLEEP_MODE_NO_ALIVE_NO_PD,
    };
    err = ledc_channel_config(&channel_cfg);
    if (err == ESP_OK) {
        s_board.buzzer_ready = true;
    }
    else {
        ESP_LOGW(TAG, "Buzzer channel init failed: %s", esp_err_to_name(err));
    }
}

static void hardware_init(void)
{
    adc_init();
    ext_io_init();
    i2c_init();
    buzzer_init();
    if (s_board.gd32_present) {
        gd32_motor_stop_all();
    }
    hardware_update();
}

static bool lcd_flush_ready_cb(esp_lcd_panel_io_handle_t panel_io,
                               esp_lcd_panel_io_event_data_t *edata,
                               void *user_ctx)
{
    (void)panel_io;
    (void)edata;

    lv_display_t *display = (lv_display_t *)user_ctx;
    s_lcd_first_flush_done = true;
    lv_display_flush_ready(display);
    return false;
}

static void lvgl_flush_cb(lv_display_t *display, const lv_area_t *area, uint8_t *px_map)
{
    esp_lcd_panel_io_handle_t io_handle = lv_display_get_user_data(display);
    const int width = area->x2 - area->x1 + 1;
    const int height = area->y2 - area->y1 + 1;
    const uint16_t x_start = area->x1 + LCD_X_GAP;
    const uint16_t x_end = area->x2 + LCD_X_GAP;
    const uint16_t y_start = area->y1 + LCD_Y_GAP;
    const uint16_t y_end = area->y2 + LCD_Y_GAP;
    const uint8_t caset[] = {
        x_start >> 8, x_start & 0xFF,
        x_end >> 8, x_end & 0xFF,
    };
    const uint8_t raset[] = {
        y_start >> 8, y_start & 0xFF,
        y_end >> 8, y_end & 0xFF,
    };

    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, ST7735_CASET, caset, sizeof(caset)));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, ST7735_RASET, raset, sizeof(raset)));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_handle, ST7735_RAMWR, px_map, width * height * sizeof(uint16_t)));
}

static void lvgl_tick_cb(void *arg)
{
    (void)arg;
    lv_tick_inc(LVGL_TICK_PERIOD_MS);
}

static void keypad_read_cb(lv_indev_t *indev, lv_indev_data_t *data)
{
    (void)indev;
    static int last_raw_index = -1;
    static int stable_index = -1;
    static uint32_t raw_changed_ms = 0;
    static uint32_t last_key = LV_KEY_ENTER;
    int raw_index = -1;
    const uint32_t now_ms = lv_tick_get();

    for (size_t i = 0; i < sizeof(s_buttons) / sizeof(s_buttons[0]); ++i) {
        if (gpio_get_level(s_buttons[i].gpio) == BUTTON_ACTIVE_LEVEL) {
            raw_index = (int)i;
            break;
        }
    }

    if (raw_index != last_raw_index) {
        last_raw_index = raw_index;
        raw_changed_ms = now_ms;
        if (raw_index < 0) {
            stable_index = -1;
        }
    }
    if (lv_tick_elaps(raw_changed_ms) >= BUTTON_DEBOUNCE_MS) {
        stable_index = last_raw_index;
    }

    if (stable_index >= 0) {
        last_key = s_buttons[stable_index].key;
        data->state = LV_INDEV_STATE_PRESSED;
        data->key = last_key;
    }
    else {
        data->state = LV_INDEV_STATE_RELEASED;
        data->key = last_key;
    }
}

static void buttons_init(void)
{
    uint64_t pin_mask = 0;
    uint64_t pullup_mask = 0;

    for (size_t i = 0; i < sizeof(s_buttons) / sizeof(s_buttons[0]); ++i) {
        pin_mask |= 1ULL << s_buttons[i].gpio;
        if (s_buttons[i].gpio != GPIO_NUM_34 && s_buttons[i].gpio != GPIO_NUM_35) {
            pullup_mask |= 1ULL << s_buttons[i].gpio;
        }
    }

    gpio_config_t io_conf = {
        .pin_bit_mask = pin_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&io_conf));

    gpio_config_t pullup_conf = {
        .pin_bit_mask = pullup_mask,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&pullup_conf));
}

static void st7735_tx_param(esp_lcd_panel_io_handle_t io_handle, int cmd, const void *param, size_t param_size)
{
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, cmd, param, param_size));
}

static void st7735_delay_ms(uint32_t ms)
{
    vTaskDelay(pdMS_TO_TICKS(ms));
}

static void st7735_clear_black(esp_lcd_panel_io_handle_t io_handle)
{
    static uint16_t line[LCD_H_RES * 8];
    const uint8_t caset[] = {0x00, 0x00, 0x00, LCD_H_RES - 1};

    memset(line, 0, sizeof(line));
    ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, ST7735_CASET, caset, sizeof(caset)));
    for (uint16_t y = 0; y < LCD_V_RES; y += 8) {
        const uint16_t y2 = MIN((uint16_t)(y + 7), (uint16_t)(LCD_V_RES - 1));
        const uint8_t raset[] = {
            y >> 8, y & 0xFF,
            y2 >> 8, y2 & 0xFF,
        };
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_param(io_handle, ST7735_RASET, raset, sizeof(raset)));
        ESP_ERROR_CHECK(esp_lcd_panel_io_tx_color(io_handle, ST7735_RAMWR, line, (y2 - y + 1) * LCD_H_RES * sizeof(uint16_t)));
    }
}

static void st7735_init_black_tab_rot90(esp_lcd_panel_io_handle_t io_handle)
{
    const uint8_t frmctr[] = {0x01, 0x2C, 0x2D};
    const uint8_t frmctr3[] = {0x01, 0x2C, 0x2D, 0x01, 0x2C, 0x2D};
    const uint8_t invctr[] = {0x07};
    const uint8_t pwctr1[] = {0xA2, 0x02, 0x84};
    const uint8_t pwctr2[] = {0xC5};
    const uint8_t pwctr3[] = {0x0A, 0x00};
    const uint8_t pwctr4[] = {0x8A, 0x2A};
    const uint8_t pwctr5[] = {0x8A, 0xEE};
    const uint8_t vmctr1[] = {0x0E};
    const uint8_t madctl_default[] = {MADCTL_MX | MADCTL_MY | MADCTL_RGB};
    const uint8_t colmod[] = {0x05};
    const uint8_t caset[] = {0x00, 0x00, 0x00, LCD_NATIVE_H_RES - 1};
    const uint8_t raset[] = {0x00, 0x00, 0x00, LCD_NATIVE_V_RES - 1};
    const uint8_t gamma_pos[] = {
        0x02, 0x1C, 0x07, 0x12,
        0x37, 0x32, 0x29, 0x2D,
        0x29, 0x25, 0x2B, 0x39,
        0x00, 0x01, 0x03, 0x10,
    };
    const uint8_t gamma_neg[] = {
        0x03, 0x1D, 0x07, 0x06,
        0x2E, 0x2C, 0x29, 0x2D,
        0x2E, 0x2E, 0x37, 0x3F,
        0x00, 0x00, 0x02, 0x10,
    };
    const uint8_t madctl_rot90[] = {MADCTL_MX | MADCTL_MV | MADCTL_RGB};

    ESP_LOGI(TAG, "Initialize ST7735R panel with MicroPython init(2) compatible sequence");
    st7735_tx_param(io_handle, ST7735_DISPOFF, NULL, 0);
    st7735_tx_param(io_handle, ST7735_SWRESET, NULL, 0);
    st7735_delay_ms(150);
    st7735_tx_param(io_handle, ST7735_SLPOUT, NULL, 0);
    st7735_delay_ms(500);
    st7735_tx_param(io_handle, ST7735_FRMCTR1, frmctr, sizeof(frmctr));
    st7735_tx_param(io_handle, ST7735_FRMCTR2, frmctr, sizeof(frmctr));
    st7735_tx_param(io_handle, ST7735_FRMCTR3, frmctr3, sizeof(frmctr3));
    st7735_tx_param(io_handle, ST7735_INVCTR, invctr, sizeof(invctr));
    st7735_tx_param(io_handle, ST7735_PWCTR1, pwctr1, sizeof(pwctr1));
    st7735_tx_param(io_handle, ST7735_PWCTR2, pwctr2, sizeof(pwctr2));
    st7735_tx_param(io_handle, ST7735_PWCTR3, pwctr3, sizeof(pwctr3));
    st7735_tx_param(io_handle, ST7735_PWCTR4, pwctr4, sizeof(pwctr4));
    st7735_tx_param(io_handle, ST7735_PWCTR5, pwctr5, sizeof(pwctr5));
    st7735_tx_param(io_handle, ST7735_VMCTR1, vmctr1, sizeof(vmctr1));
    st7735_tx_param(io_handle, ST7735_INVOFF, NULL, 0);
    st7735_tx_param(io_handle, ST7735_MADCTL, madctl_default, sizeof(madctl_default));
    st7735_tx_param(io_handle, ST7735_COLMOD, colmod, sizeof(colmod));
    st7735_tx_param(io_handle, ST7735_CASET, caset, sizeof(caset));
    st7735_tx_param(io_handle, ST7735_RASET, raset, sizeof(raset));
    st7735_tx_param(io_handle, ST7735_GMCTRP1, gamma_pos, sizeof(gamma_pos));
    st7735_tx_param(io_handle, ST7735_GMCTRN1, gamma_neg, sizeof(gamma_neg));
    st7735_tx_param(io_handle, ST7735_NORON, NULL, 0);
    st7735_delay_ms(10);
    st7735_tx_param(io_handle, ST7735_MADCTL, madctl_rot90, sizeof(madctl_rot90));
    st7735_clear_black(io_handle);
}

static void lcd_display_on(void)
{
    if (s_lcd_display_on || !s_lcd_io_handle) {
        return;
    }

    st7735_tx_param(s_lcd_io_handle, ST7735_DISPON, NULL, 0);
    st7735_delay_ms(20);
    s_lcd_display_on = true;
}

static esp_lcd_panel_io_handle_t lcd_init(void)
{
    ESP_LOGI(TAG, "Initialize SPI bus for ST7735 TFT");
    spi_bus_config_t buscfg = {
        .sclk_io_num = PIN_NUM_LCD_SCLK,
        .mosi_io_num = PIN_NUM_LCD_MOSI,
        .miso_io_num = PIN_NUM_LCD_MISO,
        .quadwp_io_num = -1,
        .quadhd_io_num = -1,
        .max_transfer_sz = LCD_H_RES * LCD_DRAW_BUF_LINES * sizeof(uint16_t),
    };
    ESP_ERROR_CHECK(spi_bus_initialize(LCD_HOST, &buscfg, SPI_DMA_CH_AUTO));

    ESP_LOGI(TAG, "Install panel IO");
    esp_lcd_panel_io_handle_t io_handle = NULL;
    esp_lcd_panel_io_spi_config_t io_config = {
        .dc_gpio_num = PIN_NUM_LCD_DC,
        .cs_gpio_num = PIN_NUM_LCD_CS,
        .pclk_hz = LCD_PIXEL_CLOCK_HZ,
        .lcd_cmd_bits = LCD_CMD_BITS,
        .lcd_param_bits = LCD_PARAM_BITS,
        .spi_mode = 0,
        .trans_queue_depth = 10,
    };
    ESP_ERROR_CHECK(esp_lcd_new_panel_io_spi((esp_lcd_spi_bus_handle_t)LCD_HOST,
                                             &io_config,
                                             &io_handle));
    s_lcd_io_handle = io_handle;
    s_lcd_display_on = false;
    s_lcd_first_flush_done = false;
    st7735_init_black_tab_rot90(io_handle);

    return io_handle;
}

static lv_display_t *lvgl_display_init(esp_lcd_panel_io_handle_t io_handle)
{
#if LCD_DRAW_BUF_LINES != LCD_V_RES
#error "Triple/full refresh mode requires LCD_DRAW_BUF_LINES to equal LCD_V_RES"
#endif
#if LCD_DRAW_BUF_COUNT != 3
#error "This build is configured for full-screen triple buffering"
#endif

    lv_display_t *display = lv_display_create(LCD_H_RES, LCD_V_RES);
    assert(display);

    const lv_color_format_t color_format = LV_COLOR_FORMAT_RGB565_SWAPPED;
    const uint32_t stride = lv_draw_buf_width_to_stride(LCD_H_RES, color_format);
    const size_t draw_buffer_sz = stride * LCD_DRAW_BUF_LINES;
    void *buf1 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    void *buf2 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    void *buf3 = spi_bus_dma_memory_alloc(LCD_HOST, draw_buffer_sz, 0);
    assert(buf1);
    assert(buf2);
    assert(buf3);

    lv_display_set_color_format(display, color_format);
    lv_display_set_dpi(display, LCD_DPI);
    lv_display_set_buffers(display, buf1, buf2, draw_buffer_sz, LV_DISPLAY_RENDER_MODE_FULL);
    lv_result_t res = lv_draw_buf_init(&s_draw_buf3,
                                       LCD_H_RES,
                                       LCD_DRAW_BUF_LINES,
                                       color_format,
                                       stride,
                                       buf3,
                                       draw_buffer_sz);
    assert(res == LV_RESULT_OK);
    lv_display_set_3rd_draw_buffer(display, &s_draw_buf3);
    lv_display_set_user_data(display, io_handle);
    lv_display_set_flush_cb(display, lvgl_flush_cb);

    ESP_LOGI(TAG,
             "LVGL display: %dx%d, dpi=%d, %d full-screen DMA buffers, SPI=%d MHz",
             LCD_H_RES,
             LCD_V_RES,
             LCD_DPI,
             LCD_DRAW_BUF_COUNT,
             LCD_PIXEL_CLOCK_HZ / 1000000);

    return display;
}

static const char *const s_page_names[UI_PAGE_COUNT] = {
    "LIGHT",
    "THERM",
    "MOTION",
    "LED 1",
    "LED 2",
    "BUZZER",
    "MOTOR 1",
    "MOTOR 2",
    "SD CARD",
    "GPIO25",
    "GPIO26",
    "GPIO32",
    "GPIO33",
    "SYSTEM",
    "ABOUT",
};

static const uint32_t UI_YELLOW = 0xF6D34A;
static const uint32_t UI_BLACK = 0x1B1713;
static const uint32_t UI_BROWN = 0x5C4220;
static const uint32_t UI_RED = 0xE64B3C;
static const uint32_t UI_CREAM = 0xFFF3B0;
static const int UI_HISTORY_CHART_PAD_X = 2;
static const int UI_HISTORY_CHART_PAD_Y = 3;
static const int UI_HISTORY_HEAD_SIZE = 7;

static void ui_refresh(void);
static void ui_show_page(ui_page_t page, int dir);

static const char *short_err(esp_err_t err)
{
    switch (err) {
    case ESP_OK:
        return "OK";
    case ESP_ERR_TIMEOUT:
        return "TIMEOUT";
    case ESP_ERR_NOT_FOUND:
        return "NOT FOUND";
    case ESP_ERR_INVALID_STATE:
        return "STATE";
    case ESP_ERR_INVALID_ARG:
        return "ARG";
    case ESP_FAIL:
        return "FAIL";
    default:
        return "ERR";
    }
}

static lv_obj_t *ui_label(lv_obj_t *parent,
                          const char *text,
                          int y,
                          uint32_t color,
                          const lv_font_t *font,
                          lv_text_align_t align)
{
    lv_obj_t *label = lv_label_create(parent);
    lv_label_set_text(label, text);
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_CLIP);
    lv_obj_set_size(label, LCD_H_RES - 16, LV_SIZE_CONTENT);
    lv_obj_set_pos(label, 8, y);
    lv_obj_set_style_text_color(label, lv_color_hex(color), 0);
    lv_obj_set_style_text_font(label, font, 0);
    lv_obj_set_style_text_align(label, align, 0);
    return label;
}

static lv_obj_t *ui_make_page(int x)
{
    lv_obj_t *page = lv_obj_create(s_ui.screen);
    lv_obj_remove_style_all(page);
    lv_obj_set_pos(page, x, 0);
    lv_obj_set_size(page, LCD_H_RES, LCD_V_RES);
    lv_obj_set_style_bg_color(page, lv_color_hex(UI_YELLOW), 0);
    lv_obj_set_style_bg_opa(page, LV_OPA_COVER, 0);
    if (s_ui.page_id == UI_PAGE_ABOUT) {
        lv_obj_add_flag(page, LV_OBJ_FLAG_SCROLLABLE);
        lv_obj_set_scroll_dir(page, LV_DIR_VER);
        lv_obj_set_scrollbar_mode(page, LV_SCROLLBAR_MODE_AUTO);
        lv_obj_set_style_width(page, 3, LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_color(page, lv_color_hex(UI_BROWN), LV_PART_SCROLLBAR);
        lv_obj_set_style_bg_opa(page, LV_OPA_80, LV_PART_SCROLLBAR);
    }
    else {
        lv_obj_clear_flag(page, LV_OBJ_FLAG_SCROLLABLE);
    }
    return page;
}

static lv_obj_t *ui_bar(lv_obj_t *parent, int value)
{
    lv_obj_t *bar = lv_bar_create(parent);
    lv_obj_set_pos(bar, 18, 86);
    lv_obj_set_size(bar, 124, 8);
    lv_bar_set_range(bar, 0, 100);
    lv_bar_set_value(bar, value, LV_ANIM_OFF);
    lv_obj_set_style_radius(bar, 4, 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(UI_CREAM), 0);
    lv_obj_set_style_bg_color(bar, lv_color_hex(UI_BLACK), LV_PART_INDICATOR);
    lv_obj_set_style_radius(bar, 4, LV_PART_INDICATOR);
    return bar;
}

static bool ui_page_has_history(ui_page_t page)
{
    return page == UI_PAGE_LIGHT || page == UI_PAGE_THERM;
}

static int32_t *ui_history_for_page(ui_page_t page)
{
    return page == UI_PAGE_THERM ? s_therm_history : s_light_history;
}

static uint32_t ui_history_version_for_page(ui_page_t page)
{
    return page == UI_PAGE_THERM ? s_therm_history_version : s_light_history_version;
}

static uint32_t ui_history_color_for_page(ui_page_t page)
{
    (void)page;
    return UI_BROWN;
}

static int ui_history_min_for_page(ui_page_t page)
{
    return page == UI_PAGE_THERM ? THERM_HISTORY_MIN_PCT : 0;
}

static int ui_history_max_for_page(ui_page_t page)
{
    return page == UI_PAGE_THERM ? THERM_HISTORY_MAX_PCT : 100;
}

static lv_obj_t *ui_history_chart(lv_obj_t *parent, int32_t *history, uint32_t color)
{
    lv_obj_t *chart = lv_chart_create(parent);
    lv_obj_set_pos(chart, 18, 77);
    lv_obj_set_size(chart, 124, 25);
    lv_obj_clear_flag(chart, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_set_style_pad_all(chart, 0, 0);
    lv_obj_set_style_pad_left(chart, UI_HISTORY_CHART_PAD_X, 0);
    lv_obj_set_style_pad_right(chart, UI_HISTORY_CHART_PAD_X, 0);
    lv_obj_set_style_pad_top(chart, UI_HISTORY_CHART_PAD_Y, 0);
    lv_obj_set_style_pad_bottom(chart, UI_HISTORY_CHART_PAD_Y, 0);
    lv_obj_set_style_radius(chart, 4, 0);
    lv_obj_set_style_border_width(chart, 0, 0);
    lv_obj_set_style_bg_opa(chart, LV_OPA_TRANSP, 0);
    lv_obj_set_style_line_width(chart, 1, LV_PART_MAIN);
    lv_obj_set_style_line_color(chart, lv_color_hex(UI_BROWN), LV_PART_MAIN);
    lv_obj_set_style_line_opa(chart, LV_OPA_TRANSP, LV_PART_MAIN);
    lv_obj_set_style_line_width(chart, 2, LV_PART_ITEMS);
    lv_obj_set_style_size(chart, 0, 0, LV_PART_INDICATOR);
    lv_chart_set_type(chart, LV_CHART_TYPE_LINE);
    lv_chart_set_point_count(chart, UI_HISTORY_POINTS);
    lv_chart_set_axis_range(chart,
                            LV_CHART_AXIS_PRIMARY_Y,
                            ui_history_min_for_page(s_ui.page_id),
                            ui_history_max_for_page(s_ui.page_id));
    lv_chart_set_div_line_count(chart, 2, 4);

    s_ui.chart_series = lv_chart_add_series(chart, lv_color_hex(color), LV_CHART_AXIS_PRIMARY_Y);
    if (s_ui.chart_series) {
        lv_chart_set_series_ext_y_array(chart, s_ui.chart_series, history);
    }
    lv_chart_refresh(chart);
    return chart;
}

static lv_obj_t *ui_history_head_dot(lv_obj_t *parent, uint32_t color)
{
    lv_obj_t *dot = lv_obj_create(parent);
    lv_obj_remove_style_all(dot);
    lv_obj_set_size(dot, UI_HISTORY_HEAD_SIZE, UI_HISTORY_HEAD_SIZE);
    lv_obj_set_style_radius(dot, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(dot, lv_color_hex(color), 0);
    lv_obj_set_style_bg_opa(dot, LV_OPA_COVER, 0);
    lv_obj_set_style_border_width(dot, 0, 0);
    lv_obj_add_flag(dot, LV_OBJ_FLAG_HIDDEN);
    return dot;
}

static void ui_update_history_head(void)
{
    if (!s_ui.chart || !s_ui.chart_series || !s_ui.chart_head) {
        return;
    }

    const int32_t *history = ui_history_for_page(s_ui.page_id);
    if (history[UI_HISTORY_POINTS - 1] == LV_CHART_POINT_NONE) {
        lv_obj_add_flag(s_ui.chart_head, LV_OBJ_FLAG_HIDDEN);
        return;
    }

    lv_point_t p;
    lv_chart_get_point_pos_by_id(s_ui.chart, s_ui.chart_series, UI_HISTORY_POINTS - 1, &p);
    const int32_t dot_half = UI_HISTORY_HEAD_SIZE / 2;
    lv_obj_set_pos(s_ui.chart_head,
                   lv_obj_get_x(s_ui.chart) + p.x - dot_half,
                   lv_obj_get_y(s_ui.chart) + p.y - dot_half);
    lv_obj_clear_flag(s_ui.chart_head, LV_OBJ_FLAG_HIDDEN);
    lv_obj_move_foreground(s_ui.chart_head);
}

static void ui_set_bar(int value)
{
    if (s_ui.bar) {
        lv_bar_set_value(s_ui.bar, MAX(0, MIN(value, 100)), LV_ANIM_ON);
    }
}

static void ui_refresh_history_chart(void)
{
    const uint32_t version = ui_history_version_for_page(s_ui.page_id);
    if (s_ui.chart && s_ui.chart_history_version != version) {
        lv_chart_refresh(s_ui.chart);
        ui_update_history_head();
        s_ui.chart_history_version = version;
    }
}

static void ui_set_hint(const char *normal)
{
    if (!s_ui.hint) {
        return;
    }
    if (s_action_until_ms && (int32_t)(s_action_until_ms - lv_tick_get()) > 0) {
        lv_label_set_text(s_ui.hint, s_board.action);
    }
    else {
        s_action_until_ms = 0;
        lv_label_set_text(s_ui.hint, normal);
    }
}

static unsigned ui_kb(size_t bytes)
{
    return (unsigned)((bytes + 512) / 1024);
}

static void ui_build_about_page(lv_obj_t *page)
{
    esp_chip_info_t chip_info;
    char details[1200];

    esp_chip_info(&chip_info);

    const size_t sram_total = heap_caps_get_total_size(MALLOC_CAP_INTERNAL);
    const size_t psram_total = heap_caps_get_total_size(MALLOC_CAP_SPIRAM);

    snprintf(details,
             sizeof(details),
             "Model\n"
             "  Xiaomiao Handheld\n"
             "  ESP32-WROVER-B\n"
             "  Author: ZYoung\n\n"
             "CPU\n"
             "  Xtensa LX6\n"
             "    %d MHz x%u\n"
             "  Chip rev: %u\n\n"
             "System\n"
             "  ESP-IDF: %s\n"
             "  FreeRTOS: %s\n"
             "  Target: %s\n"
             "  Build: %s\n\n"
             "Clocks\n"
             "  Flash: %s %s\n"
             "  PSRAM: %d MHz\n"
             "  LCD SPI2: %u MHz\n"
             "  SD SPI2: %u MHz\n"
             "  I2C0: %u kHz\n"
             "  Light ADC: 60 Hz\n\n"
             "Storage\n"
             "  Flash: %s\n"
             "  SRAM: %u KB\n"
             "  PSRAM: %u KB\n\n"
             "Display\n"
             "  ST7735 160x128\n"
             "  SPI2 %u MHz\n"
             "  RGB565 DMA x%u\n"
             "  LVGL %d.%d.%d\n\n"
             "Board IO\n"
             "  Keys: 6 active-low\n"
             "  SD: SPI2 CS22\n"
             "  ADC: 36/39/32/33\n"
             "  I2C: GD32 0x40\n"
             "       MPU6050 0x68\n"
             "  PWM: GPIO14 Buzzer\n"
             "       GPIO25/26 EXT\n\n"
             "wechat/tel:\n"
             "  15657325738\n",
             CONFIG_ESP_DEFAULT_CPU_FREQ_MHZ,
             (unsigned)chip_info.cores,
             (unsigned)chip_info.revision,
             esp_get_idf_version(),
             tskKERNEL_VERSION_NUMBER,
             UI_TARGET_NAME,
             __DATE__,
             UI_FLASH_MODE,
             UI_FLASH_FREQ,
             CONFIG_SPIRAM_SPEED,
             (unsigned)(LCD_PIXEL_CLOCK_HZ / 1000000),
             (unsigned)(SD_SPI_MAX_FREQ_KHZ / 1000),
             (unsigned)(I2C_FREQ_HZ / 1000),
             UI_FLASH_SIZE,
             ui_kb(sram_total),
             ui_kb(psram_total),
             (unsigned)(LCD_PIXEL_CLOCK_HZ / 1000000),
             (unsigned)LCD_DRAW_BUF_COUNT,
             LVGL_VERSION_MAJOR,
             LVGL_VERSION_MINOR,
             LVGL_VERSION_PATCH);

    lv_obj_t *label = lv_label_create(page);
    lv_label_set_text(label, details);
    lv_label_set_long_mode(label, LV_LABEL_LONG_MODE_WRAP);
    lv_obj_set_pos(label, 8, 28);
    lv_obj_set_width(label, LCD_H_RES - 22);
    lv_obj_set_style_text_color(label, lv_color_hex(UI_BLACK), 0);
    lv_obj_set_style_text_font(label, &lv_font_montserrat_12, 0);
    lv_obj_set_style_text_line_space(label, 1, 0);

    s_ui.value = lv_label_create(page);
    lv_obj_add_flag(s_ui.value, LV_OBJ_FLAG_HIDDEN);
    s_ui.sub = lv_label_create(page);
    lv_obj_add_flag(s_ui.sub, LV_OBJ_FLAG_HIDDEN);
    s_ui.hint = lv_label_create(page);
    lv_obj_add_flag(s_ui.hint, LV_OBJ_FLAG_HIDDEN);
    s_ui.bar = NULL;
    s_ui.chart = NULL;
    s_ui.chart_head = NULL;
    s_ui.chart_series = NULL;
}

static void ui_build_page_content(lv_obj_t *page)
{
    char idx[10];

    s_ui.title = ui_label(page, s_page_names[s_ui.page_id], 7, UI_BLACK, &lv_font_montserrat_14, LV_TEXT_ALIGN_LEFT);
    snprintf(idx, sizeof(idx), "%02u/%02u", (unsigned)s_ui.page_id + 1, (unsigned)UI_PAGE_COUNT);
    s_ui.status = ui_label(page, idx, 7, UI_BROWN, &lv_font_montserrat_10, LV_TEXT_ALIGN_RIGHT);

    s_ui.accent = lv_obj_create(page);
    lv_obj_remove_style_all(s_ui.accent);
    lv_obj_set_size(s_ui.accent, 13, 13);
    lv_obj_set_pos(s_ui.accent, 132, 25);
    lv_obj_set_style_radius(s_ui.accent, LV_RADIUS_CIRCLE, 0);
    lv_obj_set_style_bg_color(s_ui.accent, lv_color_hex(UI_RED), 0);
    lv_obj_set_style_bg_opa(s_ui.accent, LV_OPA_COVER, 0);

    if (s_ui.page_id == UI_PAGE_ABOUT) {
        ui_build_about_page(page);
        return;
    }

    s_ui.value = ui_label(page, "--", 38, UI_BLACK, &lv_font_montserrat_14, LV_TEXT_ALIGN_CENTER);
    s_ui.sub = ui_label(page, "--", 63, UI_BROWN, &lv_font_montserrat_10, LV_TEXT_ALIGN_CENTER);
    if (ui_page_has_history(s_ui.page_id)) {
        const uint32_t color = ui_history_color_for_page(s_ui.page_id);
        s_ui.bar = NULL;
        s_ui.chart = ui_history_chart(page,
                                      ui_history_for_page(s_ui.page_id),
                                      color);
        s_ui.chart_head = ui_history_head_dot(page, color);
    }
    else {
        s_ui.chart = NULL;
        s_ui.chart_head = NULL;
        s_ui.chart_series = NULL;
        s_ui.bar = ui_bar(page, 0);
    }
    s_ui.hint = ui_label(page, "L/R page", 106, UI_BLACK, &lv_font_montserrat_10, LV_TEXT_ALIGN_CENTER);
}

static void ui_anim_x(lv_obj_t *obj, int32_t start, int32_t end, lv_anim_completed_cb_t completed_cb)
{
    lv_anim_t a;
    lv_anim_init(&a);
    lv_anim_set_var(&a, obj);
    lv_anim_set_exec_cb(&a, (lv_anim_exec_xcb_t)lv_obj_set_x);
    lv_anim_set_values(&a, start, end);
    lv_anim_set_time(&a, 150);
    if (completed_cb) {
        lv_anim_set_completed_cb(&a, completed_cb);
    }
    lv_anim_start(&a);
}

static void ui_show_page(ui_page_t page, int dir)
{
    lv_obj_t *old = s_ui.page;
    const int start_x = dir == 0 ? 0 : (dir > 0 ? LCD_H_RES : -LCD_H_RES);

    s_ui.page_id = page;
    s_ui.page = ui_make_page(start_x);
    s_ui.title = NULL;
    s_ui.value = NULL;
    s_ui.sub = NULL;
    s_ui.bar = NULL;
    s_ui.chart = NULL;
    s_ui.chart_head = NULL;
    s_ui.chart_series = NULL;
    s_ui.chart_history_version = UINT32_MAX;
    s_ui.status = NULL;
    s_ui.hint = NULL;
    s_ui.accent = NULL;
    ui_build_page_content(s_ui.page);
    ui_refresh();

    if (old) {
        if (dir == 0) {
            lv_obj_delete(old);
        }
        else {
            ui_anim_x(old, 0, dir > 0 ? -LCD_H_RES : LCD_H_RES, lv_obj_delete_anim_completed_cb);
        }
    }
    if (dir != 0) {
        ui_anim_x(s_ui.page, start_x, 0, NULL);
    }
}

static void ui_refresh(void)
{
    if (!s_ui.value || !s_ui.sub || !s_ui.hint) {
        return;
    }

    switch (s_ui.page_id) {
    case UI_PAGE_LIGHT:
        lv_label_set_text_fmt(s_ui.value, "%d%%", pct_from_raw(s_board.light_raw));
        if (s_board.adc_ready) {
            lv_label_set_text_fmt(s_ui.sub, "GPIO36  RAW %04d", s_board.light_raw);
        }
        else {
            lv_label_set_text_fmt(s_ui.sub, "ADC FAIL  %s", short_err(s_board.last_adc_err));
        }
        ui_set_hint("A sample   L/R");
        ui_refresh_history_chart();
        break;
    case UI_PAGE_THERM: {
        const bool therm_changed = s_ui.chart_history_version != s_therm_history_version;
        if (therm_changed || !s_board.adc_ready || s_board.last_adc_err != ESP_OK) {
            lv_label_set_text_fmt(s_ui.value, "%d%%", pct_from_raw(s_board.temp_raw));
            if (s_board.adc_ready) {
                lv_label_set_text_fmt(s_ui.sub, "GPIO39  RAW %04d", s_board.temp_raw);
            }
            else {
                lv_label_set_text_fmt(s_ui.sub, "ADC FAIL  %s", short_err(s_board.last_adc_err));
            }
        }
        ui_set_hint("A sample   L/R");
        ui_refresh_history_chart();
        break;
    }
    case UI_PAGE_MOTION:
        lv_label_set_text(s_ui.value, s_board.mpu_present ? s_board.gesture : "ABSENT");
        if (s_board.mpu_present) {
            lv_label_set_text_fmt(s_ui.sub, "P%+.1f  R%+.1f  0x%02X", s_board.pitch, s_board.roll, s_board.mpu_whoami);
        }
        else {
            lv_label_set_text_fmt(s_ui.sub, "MPU 0x68  %s", short_err(s_board.last_mpu_err));
        }
        ui_set_hint("A rescan   L/R");
        ui_set_bar(s_board.mpu_present ? 100 : 0);
        break;
    case UI_PAGE_LED1:
        lv_label_set_text(s_ui.value, s_board.led1_on ? "ON" : "OFF");
        lv_label_set_text(s_ui.sub, s_board.gd32_present ? "GD32 0x40  REG A0" : "GD32 0x40 ABSENT");
        ui_set_hint("A toggle   B off");
        ui_set_bar(s_board.led1_on ? 100 : 0);
        break;
    case UI_PAGE_LED2:
        lv_label_set_text(s_ui.value, s_board.led2_on ? "ON" : "OFF");
        lv_label_set_text(s_ui.sub, s_board.gd32_present ? "GD32 0x40  REG A1" : "GD32 0x40 ABSENT");
        ui_set_hint("A toggle   B off");
        ui_set_bar(s_board.led2_on ? 100 : 0);
        break;
    case UI_PAGE_BUZZER:
        lv_label_set_text_fmt(s_ui.value, "%lu Hz", (unsigned long)s_buzzer_freq_hz);
        lv_label_set_text(s_ui.sub, s_board.buzzer_ready ? "GPIO14 PWM" : "PWM INIT FAIL");
        ui_set_hint("U/D Hz  A beep  B stop");
        ui_set_bar((int)((s_buzzer_freq_hz - 440) * 100 / (1760 - 440)));
        break;
    case UI_PAGE_MOTOR1:
    case UI_PAGE_MOTOR2: {
        const uint8_t motor = s_ui.page_id == UI_PAGE_MOTOR1 ? 0 : 1;
        lv_label_set_text_fmt(s_ui.value, "%s %03u", s_board.motor_running[motor] ? "VOUT" : "PWM", s_board.motor_speed[motor]);
        if (s_board.gd32_present) {
            lv_label_set_text_fmt(s_ui.sub, "REG %s  DIR %u", motor == 0 ? "0E" : "06", s_board.motor_dir[motor] ? 1 : 0);
        }
        else {
            lv_label_set_text(s_ui.sub, "GD32 0x40 ABSENT");
        }
        ui_set_hint(s_board.motor_running[motor] ? "U/D PWM  A off  B stop" : "U/D PWM  A out  B dir");
        ui_set_bar((int)s_board.motor_speed[motor] * 100 / 255);
        break;
    }
    case UI_PAGE_SD:
        lv_label_set_text(s_ui.value, s_board.sd_mounted ? "MOUNTED" : "NO CARD");
        if (s_board.sd_mounted) {
            lv_label_set_text_fmt(s_ui.sub, "%s  %luMB", s_board.sd_name, (unsigned long)s_board.sd_mb);
        }
        else {
            lv_label_set_text_fmt(s_ui.sub, "GPIO22 CS  %s", short_err(s_board.last_sd_err));
        }
        ui_set_hint(s_board.sd_mounted ? "B unmount  L/R" : "A rescan   L/R");
        ui_set_bar(s_board.sd_mounted ? 100 : 0);
        break;
    case UI_PAGE_GPIO25:
        lv_label_set_text_fmt(s_ui.value, "%s %03u", s_board.ext_out[0] ? "PWM" : "OFF", s_board.ext_pwm[0]);
        lv_label_set_text(s_ui.sub, s_board.ext_pwm_ready ? "GPIO25 LEDC" : "PWM INIT FAIL");
        ui_set_hint("U/D duty  A toggle  B off");
        ui_set_bar((int)s_board.ext_pwm[0] * 100 / EXT_PWM_DUTY_MAX);
        break;
    case UI_PAGE_GPIO26:
        lv_label_set_text_fmt(s_ui.value, "%s %03u", s_board.ext_out[1] ? "PWM" : "OFF", s_board.ext_pwm[1]);
        lv_label_set_text(s_ui.sub, s_board.ext_pwm_ready ? "GPIO26 LEDC" : "PWM INIT FAIL");
        ui_set_hint("U/D duty  A toggle  B off");
        ui_set_bar((int)s_board.ext_pwm[1] * 100 / EXT_PWM_DUTY_MAX);
        break;
    case UI_PAGE_ADC32:
        lv_label_set_text_fmt(s_ui.value, "%d%%", pct_from_raw(s_board.ext_raw[0]));
        if (s_board.adc_ready) {
            lv_label_set_text_fmt(s_ui.sub, "GPIO32 RAW %04d", s_board.ext_raw[0]);
        }
        else {
            lv_label_set_text_fmt(s_ui.sub, "ADC FAIL  %s", short_err(s_board.last_adc_err));
        }
        ui_set_hint("A sample   L/R");
        ui_set_bar(pct_from_raw(s_board.ext_raw[0]));
        break;
    case UI_PAGE_ADC33:
        lv_label_set_text_fmt(s_ui.value, "%d%%", pct_from_raw(s_board.ext_raw[1]));
        if (s_board.adc_ready) {
            lv_label_set_text_fmt(s_ui.sub, "GPIO33 RAW %04d", s_board.ext_raw[1]);
        }
        else {
            lv_label_set_text_fmt(s_ui.sub, "ADC FAIL  %s", short_err(s_board.last_adc_err));
        }
        ui_set_hint("A sample   L/R");
        ui_set_bar(pct_from_raw(s_board.ext_raw[1]));
        break;
    case UI_PAGE_SYSTEM:
        lv_label_set_text(s_ui.value, s_board.i2c_ready ? "I2C OK" : "I2C --");
        lv_label_set_text_fmt(s_ui.sub,
                              "G %s  M %s",
                              s_board.gd32_present ? "OK" : short_err(s_board.last_gd32_err),
                              s_board.mpu_present ? "OK" : short_err(s_board.last_mpu_err));
        ui_set_hint("A rescan   L/R");
        ui_set_bar(s_board.i2c_ready ? 100 : 0);
        break;
    case UI_PAGE_ABOUT:
        break;
    default:
        break;
    }
}

static void ui_motor_stop(uint8_t motor)
{
    esp_err_t err = gd32_motor_set(motor, s_board.motor_dir[motor], 0);
    if (err == ESP_OK) {
        s_board.motor_running[motor] = false;
        set_action(motor == 0 ? "Motor1 stopped" : "Motor2 stopped");
    }
    else {
        set_action("Motor cmd fail");
    }
}

static void ui_motor_toggle(uint8_t motor)
{
    if (s_board.motor_running[motor]) {
        ui_motor_stop(motor);
        return;
    }
    if (s_board.motor_speed[motor] == 0) {
        s_board.last_gd32_err = ESP_ERR_INVALID_ARG;
        set_action("PWM is zero");
        return;
    }

    esp_err_t err = gd32_motor_set(motor, s_board.motor_dir[motor], s_board.motor_speed[motor]);
    if (err == ESP_OK) {
        s_board.motor_running[motor] = true;
        set_action(motor == 0 ? "Motor1 output" : "Motor2 output");
    }
    else {
        set_action("Motor cmd fail");
    }
}

static esp_err_t ui_ext_toggle(uint8_t index)
{
    if (s_board.ext_out[index]) {
        esp_err_t err = ext_output_set(index, false);
        set_action(err == ESP_OK ? (index == 0 ? "GPIO25 off" : "GPIO26 off") : "PWM cmd fail");
        return err;
    }

    if (s_board.ext_pwm[index] == 0) {
        set_action("Duty is zero");
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = ext_output_set(index, true);
    set_action(err == ESP_OK ? (index == 0 ? "GPIO25 PWM" : "GPIO26 PWM") : "PWM cmd fail");
    return err;
}

static void ui_action(void)
{
    esp_err_t err = ESP_OK;

    switch (s_ui.page_id) {
    case UI_PAGE_LIGHT:
    case UI_PAGE_THERM:
    case UI_PAGE_ADC32:
    case UI_PAGE_ADC33:
        err = adc_read_sensors();
        set_action(err == ESP_OK ? "Sampled" : "ADC read fail");
        break;
    case UI_PAGE_MOTION:
        mpu_probe_and_init(true);
        err = s_board.mpu_present ? ESP_OK : s_board.last_mpu_err;
        set_action(s_board.mpu_present ? "MPU ready" : "MPU absent");
        break;
    case UI_PAGE_LED1:
        err = gd32_write_reg(GD32_LED1_REG, s_board.led1_on ? 0 : 1);
        if (err == ESP_OK) {
            s_board.led1_on = !s_board.led1_on;
        }
        set_action(err == ESP_OK ? "LED1 toggled" : "LED cmd fail");
        break;
    case UI_PAGE_LED2:
        err = gd32_write_reg(GD32_LED2_REG, s_board.led2_on ? 0 : 1);
        if (err == ESP_OK) {
            s_board.led2_on = !s_board.led2_on;
        }
        set_action(err == ESP_OK ? "LED2 toggled" : "LED cmd fail");
        break;
    case UI_PAGE_BUZZER:
        buzzer_beep(s_buzzer_freq_hz, 140);
        set_action(s_board.buzzer_ready ? "Beep" : "Buzzer init fail");
        break;
    case UI_PAGE_MOTOR1:
        ui_motor_toggle(0);
        err = s_board.last_gd32_err;
        break;
    case UI_PAGE_MOTOR2:
        ui_motor_toggle(1);
        err = s_board.last_gd32_err;
        break;
    case UI_PAGE_SD:
        sd_try_mount();
        err = s_board.sd_mounted ? ESP_OK : s_board.last_sd_err;
        set_action(s_board.sd_mounted ? "SD mounted" : "No SD card");
        break;
    case UI_PAGE_GPIO25:
        err = ui_ext_toggle(0);
        break;
    case UI_PAGE_GPIO26:
        err = ui_ext_toggle(1);
        break;
    case UI_PAGE_SYSTEM:
        i2c_probe_devices(true);
        err = s_board.i2c_ready ? ESP_OK : ESP_ERR_INVALID_STATE;
        set_action(s_board.i2c_ready ? "Rescanned" : "I2C init fail");
        break;
    default:
        break;
    }

    if (err == ESP_OK && s_ui.page_id != UI_PAGE_BUZZER) {
        buzzer_beep(660, 35);
    }
    ui_refresh();
}

static void ui_cancel(void)
{
    switch (s_ui.page_id) {
    case UI_PAGE_LED1:
        if (s_board.led1_on) {
            esp_err_t err = gd32_write_reg(GD32_LED1_REG, 0);
            if (err == ESP_OK) {
                s_board.led1_on = false;
            }
            set_action(err == ESP_OK ? "LED1 off" : "LED cmd fail");
        }
        else {
            set_action("LED1 off");
        }
        break;
    case UI_PAGE_LED2:
        if (s_board.led2_on) {
            esp_err_t err = gd32_write_reg(GD32_LED2_REG, 0);
            if (err == ESP_OK) {
                s_board.led2_on = false;
            }
            set_action(err == ESP_OK ? "LED2 off" : "LED cmd fail");
        }
        else {
            set_action("LED2 off");
        }
        break;
    case UI_PAGE_BUZZER:
        buzzer_stop();
        set_action("Buzzer stop");
        break;
    case UI_PAGE_SD:
        sd_unmount();
        break;
    case UI_PAGE_GPIO25:
        {
            esp_err_t err = ext_output_set(0, false);
            set_action(err == ESP_OK ? "GPIO25 off" : "PWM cmd fail");
        }
        break;
    case UI_PAGE_GPIO26:
        {
            esp_err_t err = ext_output_set(1, false);
            set_action(err == ESP_OK ? "GPIO26 off" : "PWM cmd fail");
        }
        break;
    case UI_PAGE_MOTOR1:
        if (s_board.motor_running[0]) {
            ui_motor_stop(0);
        }
        else {
            s_board.motor_dir[0] = !s_board.motor_dir[0];
            set_action("Motor1 dir");
        }
        break;
    case UI_PAGE_MOTOR2:
        if (s_board.motor_running[1]) {
            ui_motor_stop(1);
        }
        else {
            s_board.motor_dir[1] = !s_board.motor_dir[1];
            set_action("Motor2 dir");
        }
        break;
    default:
        buzzer_stop();
        set_action("Canceled");
        break;
    }
    ui_refresh();
}

static void ui_adjust(int step)
{
    switch (s_ui.page_id) {
    case UI_PAGE_BUZZER: {
        int freq = (int)s_buzzer_freq_hz + step * 110;
        s_buzzer_freq_hz = MAX(440, MIN(freq, 1760));
        set_action("Pitch set");
        break;
    }
    case UI_PAGE_MOTOR1:
    case UI_PAGE_MOTOR2: {
        const uint8_t motor = s_ui.page_id == UI_PAGE_MOTOR1 ? 0 : 1;
        int speed = s_board.motor_speed[motor] + step * 10;
        s_board.motor_speed[motor] = MAX(0, MIN(speed, 255));
        if (s_board.motor_running[motor]) {
            esp_err_t err = gd32_motor_set(motor, s_board.motor_dir[motor], s_board.motor_speed[motor]);
            set_action(err == ESP_OK ? "Power set" : "Motor cmd fail");
        }
        else {
            set_action("Power set");
        }
        break;
    }
    case UI_PAGE_GPIO25:
    case UI_PAGE_GPIO26: {
        const uint8_t index = s_ui.page_id == UI_PAGE_GPIO25 ? 0 : 1;
        int duty = s_board.ext_pwm[index] + step * 16;
        s_board.ext_pwm[index] = MAX(0, MIN(duty, EXT_PWM_DUTY_MAX));
        if (!s_board.ext_pwm_ready) {
            set_action("PWM init fail");
        }
        else if (s_board.ext_out[index]) {
            esp_err_t err = ext_output_set(index, true);
            set_action(err == ESP_OK ? (s_board.ext_out[index] ? "Duty set" : "Duty zero") : "PWM cmd fail");
        }
        else {
            set_action("Duty set");
        }
        break;
    }
    default:
        return;
    }
    ui_refresh();
}

static void ui_scroll_about(int step)
{
    if (!s_ui.page) {
        return;
    }

    const int32_t scroll_step = 26;
    const int32_t scroll_y = lv_obj_get_scroll_y(s_ui.page) + step * scroll_step;
    lv_obj_scroll_to_y(s_ui.page, MAX(0, scroll_y), LV_ANIM_ON);
}

static void ui_key_event_cb(lv_event_t *e)
{
    if (lv_event_get_code(e) != LV_EVENT_KEY) {
        return;
    }

    const uint32_t key = lv_event_get_key(e);

    if (key == LV_KEY_LEFT || key == LV_KEY_RIGHT) {
        int next = (int)s_ui.page_id + (key == LV_KEY_RIGHT ? 1 : -1);
        if (next < 0) {
            next = UI_PAGE_COUNT - 1;
        }
        if (next >= UI_PAGE_COUNT) {
            next = 0;
        }
        ui_show_page((ui_page_t)next, key == LV_KEY_RIGHT ? 1 : -1);
    }
    else if (key == LV_KEY_UP) {
        if (s_ui.page_id == UI_PAGE_ABOUT) {
            ui_scroll_about(-1);
        }
        else {
            ui_adjust(1);
        }
    }
    else if (key == LV_KEY_DOWN) {
        if (s_ui.page_id == UI_PAGE_ABOUT) {
            ui_scroll_about(1);
        }
        else {
            ui_adjust(-1);
        }
    }
    else if (key == LV_KEY_ENTER) {
        ui_action();
    }
    else if (key == LV_KEY_ESC) {
        ui_cancel();
    }
}

static void ui_create(lv_group_t *group)
{
    s_ui.group = group;
    s_ui.page_id = UI_PAGE_LIGHT;
    s_ui.screen = lv_obj_create(lv_screen_active());
    lv_obj_remove_style_all(s_ui.screen);
    lv_obj_set_size(s_ui.screen, LCD_H_RES, LCD_V_RES);
    lv_obj_set_style_bg_color(s_ui.screen, lv_color_hex(UI_YELLOW), 0);
    lv_obj_set_style_bg_opa(s_ui.screen, LV_OPA_COVER, 0);
    lv_obj_clear_flag(s_ui.screen, LV_OBJ_FLAG_SCROLLABLE);
    lv_obj_add_flag(s_ui.screen, LV_OBJ_FLAG_CLICKABLE);
    lv_group_add_obj(group, s_ui.screen);
    lv_group_focus_obj(s_ui.screen);
    lv_obj_add_event_cb(s_ui.screen, ui_key_event_cb, LV_EVENT_KEY, NULL);
    ui_show_page(UI_PAGE_LIGHT, 0);
}

static lv_group_t *lvgl_input_init(lv_display_t *display)
{
    lv_group_t *group = lv_group_create();
    assert(group);
    lv_group_set_default(group);

    lv_indev_t *indev = lv_indev_create();
    assert(indev);
    lv_indev_set_type(indev, LV_INDEV_TYPE_KEYPAD);
    lv_indev_set_display(indev, display);
    lv_indev_set_group(indev, group);
    lv_indev_set_read_cb(indev, keypad_read_cb);
    lv_indev_set_long_press_time(indev, 360);
    lv_indev_set_long_press_repeat_time(indev, 130);

    return group;
}

static void lvgl_task(void *arg)
{
    lv_group_t *group = (lv_group_t *)arg;
    uint32_t last_update_ms = 0;

    ESP_LOGI(TAG, "Start Xiaomiao hardware dashboard");
    ui_create(group);
    s_lcd_first_flush_done = false;
    lv_refr_now(NULL);
    for (uint8_t i = 0; i < 100 && !s_lcd_first_flush_done; ++i) {
        vTaskDelay(pdMS_TO_TICKS(1));
    }
    lcd_display_on();

    while (true) {
        hardware_process_timers();
        if (lv_tick_elaps(last_update_ms) >= UI_REFRESH_PERIOD_MS) {
            last_update_ms = lv_tick_get();
            hardware_update();
            ui_refresh();
        }

        uint32_t delay_ms = lv_timer_handler();
        delay_ms = MAX(delay_ms, LVGL_TASK_MIN_DELAY_MS);
        delay_ms = MIN(delay_ms, LVGL_TASK_MAX_DELAY_MS);
        usleep(delay_ms * 1000);
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "Xiaomiao LVGL 9.5 dashboard boot");

    sensor_history_init();
    buttons_init();

    esp_lcd_panel_io_handle_t io_handle = lcd_init();
    hardware_init();

    lv_init();
    lv_display_t *display = lvgl_display_init(io_handle);
    lv_group_t *group = lvgl_input_init(display);

    esp_lcd_panel_io_callbacks_t callbacks = {
        .on_color_trans_done = lcd_flush_ready_cb,
    };
    ESP_ERROR_CHECK(esp_lcd_panel_io_register_event_callbacks(io_handle, &callbacks, display));

    const esp_timer_create_args_t tick_timer_args = {
        .callback = lvgl_tick_cb,
        .name = "lvgl_tick",
    };
    esp_timer_handle_t tick_timer = NULL;
    ESP_ERROR_CHECK(esp_timer_create(&tick_timer_args, &tick_timer));
    ESP_ERROR_CHECK(esp_timer_start_periodic(tick_timer, LVGL_TICK_PERIOD_MS * 1000));

    BaseType_t ret = xTaskCreate(lvgl_task,
                                 "lvgl",
                                 LVGL_TASK_STACK_SIZE,
                                 group,
                                 LVGL_TASK_PRIORITY,
                                 NULL);
    ESP_ERROR_CHECK(ret == pdPASS ? ESP_OK : ESP_FAIL);
}
