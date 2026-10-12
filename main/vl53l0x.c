#include "vl53l0x.h"

#include "app_config.h"

#include "driver/gpio.h"
#include "driver/i2c_master.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "vl53l0x";

enum {
    REG_SYSRANGE_START = 0x00,
    REG_SYSTEM_INTERRUPT_CONFIG = 0x0A,
    REG_SYSTEM_INTERRUPT_CLEAR = 0x0B,
    REG_RESULT_INTERRUPT_STATUS = 0x13,
    REG_RESULT_RANGE_STATUS = 0x14,
    REG_MSRC_CONFIG = 0x60,
    REG_MSRC_TIMEOUT = 0x46,
    REG_FINAL_RANGE_MIN_COUNT = 0x44,
    REG_PRE_RANGE_VCSEL = 0x50,
    REG_PRE_RANGE_TIMEOUT = 0x51,
    REG_FINAL_RANGE_VCSEL = 0x70,
    REG_FINAL_RANGE_TIMEOUT = 0x71,
    REG_SYSTEM_SEQUENCE = 0x01,
    REG_DYNAMIC_SPAD_NUM = 0x4E,
    REG_DYNAMIC_SPAD_OFFSET = 0x4F,
    REG_GPIO_MUX = 0x84,
    REG_POWER_FORCE = 0x80,
    REG_SPAD_ENABLES = 0xB0,
    REG_REF_EN_START = 0xB6,
    REG_VHV_CONFIG = 0x89,
    REG_MODEL_ID = 0xC0,
};

/* ST's recommended default tuning for this sensor. */
static const uint8_t TUNING[] = {
    0xFF, 0x01, 0x00, 0x00, 0xFF, 0x00, 0x09, 0x00, 0x10, 0x00, 0x11, 0x00, 0x24, 0x01, 0x25, 0xFF, 0x75, 0x00,
    0xFF, 0x01, 0x4E, 0x2C, 0x48, 0x00, 0x30, 0x20, 0xFF, 0x00, 0x30, 0x09, 0x54, 0x00, 0x31, 0x04, 0x32, 0x03,
    0x40, 0x83, 0x46, 0x25, 0x60, 0x00, 0x27, 0x00, 0x50, 0x06, 0x51, 0x00, 0x52, 0x96, 0x56, 0x08, 0x57, 0x30,
    0x61, 0x00, 0x62, 0x00, 0x64, 0x00, 0x65, 0x00, 0x66, 0xA0, 0xFF, 0x01, 0x22, 0x32, 0x47, 0x14, 0x49, 0xFF,
    0x4A, 0x00, 0xFF, 0x00, 0x7A, 0x0A, 0x7B, 0x00, 0x78, 0x21, 0xFF, 0x01, 0x23, 0x34, 0x42, 0x00, 0x44, 0xFF,
    0x45, 0x26, 0x46, 0x05, 0x40, 0x40, 0x0E, 0x06, 0x20, 0x1A, 0x43, 0x40, 0xFF, 0x00, 0x34, 0x03, 0x35, 0x44,
    0xFF, 0x01, 0x31, 0x04, 0x4B, 0x09, 0x4C, 0x05, 0x4D, 0x04, 0xFF, 0x00, 0x44, 0x00, 0x45, 0x20, 0x47, 0x08,
    0x48, 0x28, 0x67, 0x00, 0x70, 0x04, 0x71, 0x01, 0x72, 0xFE, 0x76, 0x00, 0x77, 0x00, 0xFF, 0x01, 0x0D, 0x01,
    0xFF, 0x00, 0x80, 0x01, 0x01, 0xF8, 0xFF, 0x01, 0x8E, 0x01, 0x00, 0x01, 0xFF, 0x00, 0x80, 0x00,
};

static i2c_master_bus_handle_t s_bus;
static i2c_master_dev_handle_t s_dev;
static uint8_t s_stop_variable;

static esp_err_t write_reg(uint8_t reg, uint8_t value)
{
    uint8_t buf[2] = {reg, value};
    return i2c_master_transmit(s_dev, buf, sizeof(buf), 100);
}

static esp_err_t read_reg(uint8_t reg, uint8_t *value)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, value, 1, 100);
}

static esp_err_t write_regs(uint8_t reg, const uint8_t *data, size_t len)
{
    uint8_t buf[8];
    if (len + 1 > sizeof(buf)) {
        return ESP_ERR_INVALID_SIZE;
    }
    buf[0] = reg;
    memcpy(&buf[1], data, len);
    return i2c_master_transmit(s_dev, buf, len + 1, 100);
}

static esp_err_t read_regs(uint8_t reg, uint8_t *data, size_t len)
{
    return i2c_master_transmit_receive(s_dev, &reg, 1, data, len, 100);
}

static esp_err_t read_reg16(uint8_t reg, uint16_t *value)
{
    uint8_t raw[2];
    ESP_RETURN_ON_ERROR(read_regs(reg, raw, sizeof(raw)), TAG, "reg16");
    *value = (uint16_t)((raw[0] << 8) | raw[1]);
    return ESP_OK;
}

static esp_err_t write_reg16(uint8_t reg, uint16_t value)
{
    uint8_t raw[2] = {(uint8_t)(value >> 8), (uint8_t)value};
    return write_regs(reg, raw, sizeof(raw));
}

/* ST's timeout registers pack a count as (LSB << MSB). */
static uint16_t decode_timeout(uint16_t reg_val)
{
    return (uint16_t)(((reg_val & 0x00FF) << ((reg_val & 0xFF00) >> 8)) + 1);
}

static uint16_t encode_timeout(uint32_t timeout_mclks)
{
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;
    if (timeout_mclks == 0) {
        return 0;
    }
    ls_byte = timeout_mclks - 1;
    while ((ls_byte & 0xFFFFFF00u) != 0) {
        ls_byte >>= 1;
        ms_byte++;
    }
    return (uint16_t)((ms_byte << 8) | (ls_byte & 0xFF));
}

static uint32_t macro_period_ns(uint8_t vcsel_period_pclks)
{
    return (((uint32_t)2304 * vcsel_period_pclks * 1655) + 500) / 1000;
}

static uint32_t timeout_mclks_to_us(uint16_t timeout_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = macro_period_ns(vcsel_period_pclks);
    return ((timeout_mclks * macro_ns) + 500) / 1000;
}

static uint32_t timeout_us_to_mclks(uint32_t timeout_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_ns = macro_period_ns(vcsel_period_pclks);
    return ((timeout_us * 1000) + (macro_ns / 2)) / macro_ns;
}

static uint8_t decode_vcsel(uint8_t reg_val)
{
    return (uint8_t)((reg_val + 1) << 1);
}

/*
 * Shrink the final-range step so one shot lasts VL53L0X_TIMING_BUDGET_US.
 * 20 ms is the shortest budget that still leaves time for the default steps.
 * VCSEL periods stay at the chip default: shortening them saves a little
 * current but drops the maximum distance below a typical tank.
 */
static esp_err_t set_timing_budget_us(uint32_t budget_us)
{
    const uint32_t start_overhead = 1910;
    const uint32_t end_overhead = 960;
    const uint32_t msrc_overhead = 660;
    const uint32_t tcc_overhead = 590;
    const uint32_t dss_overhead = 690;
    const uint32_t pre_range_overhead = 660;
    const uint32_t final_range_overhead = 550;

    if (budget_us < 20000) {
        budget_us = 20000;
    }

    uint8_t sequence = 0;
    uint8_t msrc = 0;
    uint8_t pre_vcsel_reg = 0;
    uint8_t final_vcsel_reg = 0;
    uint16_t pre_timeout_reg = 0;
    ESP_RETURN_ON_ERROR(read_reg(REG_SYSTEM_SEQUENCE, &sequence), TAG, "budget");
    ESP_RETURN_ON_ERROR(read_reg(REG_MSRC_TIMEOUT, &msrc), TAG, "budget");
    ESP_RETURN_ON_ERROR(read_reg(REG_PRE_RANGE_VCSEL, &pre_vcsel_reg), TAG, "budget");
    ESP_RETURN_ON_ERROR(read_reg(REG_FINAL_RANGE_VCSEL, &final_vcsel_reg), TAG, "budget");
    ESP_RETURN_ON_ERROR(read_reg16(REG_PRE_RANGE_TIMEOUT, &pre_timeout_reg), TAG, "budget");

    bool tcc = (sequence >> 4) & 0x1;
    bool dss = (sequence >> 3) & 0x1;
    bool msrc_en = (sequence >> 2) & 0x1;
    bool pre_range = (sequence >> 6) & 0x1;
    bool final_range = (sequence >> 7) & 0x1;

    uint8_t pre_vcsel = decode_vcsel(pre_vcsel_reg);
    uint8_t final_vcsel = decode_vcsel(final_vcsel_reg);
    uint32_t msrc_us = timeout_mclks_to_us((uint16_t)msrc + 1, pre_vcsel);
    uint16_t pre_mclks = decode_timeout(pre_timeout_reg);
    uint32_t pre_us = timeout_mclks_to_us(pre_mclks, pre_vcsel);

    uint32_t used = start_overhead + end_overhead;
    if (tcc) {
        used += msrc_us + tcc_overhead;
    }
    if (dss) {
        used += 2 * (msrc_us + dss_overhead);
    } else if (msrc_en) {
        used += msrc_us + msrc_overhead;
    }
    if (pre_range) {
        used += pre_us + pre_range_overhead;
    }
    if (!final_range) {
        return ESP_OK;
    }
    used += final_range_overhead;
    if (used >= budget_us) {
        ESP_LOGW(TAG, "Timing budget %lu us is too small for the enabled steps", (unsigned long)budget_us);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t final_mclks = timeout_us_to_mclks(budget_us - used, final_vcsel);
    if (pre_range) {
        final_mclks += pre_mclks;
    }
    ESP_RETURN_ON_ERROR(write_reg16(REG_FINAL_RANGE_TIMEOUT, encode_timeout(final_mclks)), TAG, "budget");
    ESP_LOGI(TAG, "Timing budget %lu us", (unsigned long)budget_us);
    return ESP_OK;
}

static bool timed_out(int64_t start_us, int timeout_ms)
{
    return (esp_timer_get_time() - start_us) > ((int64_t)timeout_ms * 1000);
}

static esp_err_t load_tuning(void)
{
    for (size_t i = 0; i + 1 < sizeof(TUNING); i += 2) {
        ESP_RETURN_ON_ERROR(write_reg(TUNING[i], TUNING[i + 1]), TAG, "tuning write failed");
    }
    return ESP_OK;
}

static esp_err_t get_spad_info(uint8_t *count, bool *aperture)
{
    uint8_t tmp = 0;
    ESP_RETURN_ON_ERROR(write_reg(0x80, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x00), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x06), TAG, "spad");
    ESP_RETURN_ON_ERROR(read_reg(0x83, &tmp), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x83, tmp | 0x04), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x07), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x81, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x80, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x94, 0x6b), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x83, 0x00), TAG, "spad");

    int64_t start = esp_timer_get_time();
    do {
        ESP_RETURN_ON_ERROR(read_reg(0x83, &tmp), TAG, "spad poll");
        if (timed_out(start, 500)) {
            return ESP_ERR_TIMEOUT;
        }
    } while (tmp == 0x00);

    ESP_RETURN_ON_ERROR(write_reg(0x83, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(read_reg(0x92, &tmp), TAG, "spad");
    *count = tmp & 0x7f;
    *aperture = (tmp >> 7) & 0x01;

    ESP_RETURN_ON_ERROR(write_reg(0x81, 0x00), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x06), TAG, "spad");
    ESP_RETURN_ON_ERROR(read_reg(0x83, &tmp), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x83, tmp & (uint8_t)~0x04), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x01), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x00), TAG, "spad");
    ESP_RETURN_ON_ERROR(write_reg(REG_POWER_FORCE, 0x00), TAG, "spad");
    return ESP_OK;
}

static esp_err_t apply_spad_map(uint8_t count, bool aperture)
{
    uint8_t map[6];
    ESP_RETURN_ON_ERROR(read_regs(REG_SPAD_ENABLES, map, sizeof(map)), TAG, "spad map");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x01), TAG, "spad map");
    ESP_RETURN_ON_ERROR(write_reg(REG_DYNAMIC_SPAD_OFFSET, 0x00), TAG, "spad map");
    ESP_RETURN_ON_ERROR(write_reg(REG_DYNAMIC_SPAD_NUM, 0x2C), TAG, "spad map");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x00), TAG, "spad map");
    ESP_RETURN_ON_ERROR(write_reg(REG_REF_EN_START, 0xB4), TAG, "spad map");

    uint8_t first = aperture ? 12 : 0;
    uint8_t enabled = 0;
    for (uint8_t i = 0; i < 48; i++) {
        if (i < first || enabled == count) {
            map[i / 8] &= (uint8_t)~(1u << (i % 8));
        } else if ((map[i / 8] >> (i % 8)) & 0x1) {
            enabled++;
        }
    }
    return write_regs(REG_SPAD_ENABLES, map, sizeof(map));
}

static esp_err_t ref_calibration(uint8_t vhv_init)
{
    uint8_t status = 0;
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSRANGE_START, (uint8_t)(0x01 | vhv_init)), TAG, "ref cal");
    int64_t start = esp_timer_get_time();
    do {
        ESP_RETURN_ON_ERROR(read_reg(REG_RESULT_INTERRUPT_STATUS, &status), TAG, "ref cal");
        if (timed_out(start, 500)) {
            return ESP_ERR_TIMEOUT;
        }
    } while ((status & 0x07) == 0);
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "ref cal");
    return write_reg(REG_SYSRANGE_START, 0x00);
}

static esp_err_t sensor_init(void)
{
    uint8_t model = 0;
    uint8_t tmp = 0;
    ESP_RETURN_ON_ERROR(read_reg(REG_MODEL_ID, &model), TAG, "model id");
    if (model != 0xEE) {
        ESP_LOGE(TAG, "Unexpected model id 0x%02x", model);
        return ESP_ERR_NOT_FOUND;
    }

    ESP_RETURN_ON_ERROR(read_reg(REG_VHV_CONFIG, &tmp), TAG, "2v8");
    ESP_RETURN_ON_ERROR(write_reg(REG_VHV_CONFIG, tmp | 0x01), TAG, "2v8");
    ESP_RETURN_ON_ERROR(write_reg(0x88, 0x00), TAG, "i2c mode");

    ESP_RETURN_ON_ERROR(write_reg(REG_POWER_FORCE, 0x01), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x01), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x00), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(read_reg(0x91, &s_stop_variable), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x01), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x00), TAG, "stop variable");
    ESP_RETURN_ON_ERROR(write_reg(REG_POWER_FORCE, 0x00), TAG, "stop variable");

    ESP_RETURN_ON_ERROR(read_reg(REG_MSRC_CONFIG, &tmp), TAG, "msrc");
    ESP_RETURN_ON_ERROR(write_reg(REG_MSRC_CONFIG, tmp | 0x12), TAG, "msrc");
    /* 0.25 MCPS in Q9.7. */
    uint8_t limit[2] = {0x00, 0x20};
    ESP_RETURN_ON_ERROR(write_regs(REG_FINAL_RANGE_MIN_COUNT, limit, sizeof(limit)), TAG, "rate limit");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_SEQUENCE, 0xFF), TAG, "sequence");

    uint8_t spad_count = 0;
    bool aperture = false;
    ESP_RETURN_ON_ERROR(get_spad_info(&spad_count, &aperture), TAG, "spad info");
    ESP_RETURN_ON_ERROR(apply_spad_map(spad_count, aperture), TAG, "spad map");
    ESP_RETURN_ON_ERROR(load_tuning(), TAG, "tuning");

    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_INTERRUPT_CONFIG, 0x04), TAG, "gpio");
    ESP_RETURN_ON_ERROR(read_reg(REG_GPIO_MUX, &tmp), TAG, "gpio");
    ESP_RETURN_ON_ERROR(write_reg(REG_GPIO_MUX, tmp & (uint8_t)~0x10), TAG, "gpio");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "gpio");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_SEQUENCE, 0xE8), TAG, "sequence");

    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_SEQUENCE, 0x01), TAG, "vhv cal");
    ESP_RETURN_ON_ERROR(ref_calibration(0x40), TAG, "vhv cal");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_SEQUENCE, 0x02), TAG, "phase cal");
    ESP_RETURN_ON_ERROR(ref_calibration(0x00), TAG, "phase cal");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_SEQUENCE, 0xE8), TAG, "sequence restore");
    return set_timing_budget_us(VL53L0X_TIMING_BUDGET_US);
}

static esp_err_t read_single(uint16_t *distance_mm)
{
    uint8_t tmp = 0;
    ESP_RETURN_ON_ERROR(write_reg(REG_POWER_FORCE, 0x01), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x01), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x00), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(0x91, s_stop_variable), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(0x00, 0x01), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(0xFF, 0x00), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(REG_POWER_FORCE, 0x00), TAG, "range");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSRANGE_START, 0x01), TAG, "range");

    int64_t start = esp_timer_get_time();
    do {
        ESP_RETURN_ON_ERROR(read_reg(REG_SYSRANGE_START, &tmp), TAG, "range start");
        if (timed_out(start, 50)) {
            return ESP_ERR_TIMEOUT;
        }
    } while (tmp & 0x01);

    start = esp_timer_get_time();
    do {
        ESP_RETURN_ON_ERROR(read_reg(REG_RESULT_INTERRUPT_STATUS, &tmp), TAG, "range ready");
        if (timed_out(start, (VL53L0X_TIMING_BUDGET_US / 1000) + 30)) {
            return ESP_ERR_TIMEOUT;
        }
    } while ((tmp & 0x07) == 0);

    uint8_t raw[2] = {0};
    ESP_RETURN_ON_ERROR(read_regs(REG_RESULT_RANGE_STATUS + 10, raw, sizeof(raw)), TAG, "range result");
    ESP_RETURN_ON_ERROR(write_reg(REG_SYSTEM_INTERRUPT_CLEAR, 0x01), TAG, "range clear");
    *distance_mm = (uint16_t)((raw[0] << 8) | raw[1]);
    return ESP_OK;
}

static void pin_output(gpio_num_t pin, int level)
{
    gpio_hold_dis(pin);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    gpio_set_level(pin, level);
}

static void pin_hold(gpio_num_t pin, int level)
{
    pin_output(pin, level);
    esp_err_t err = gpio_hold_en(pin);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPIO %d will not stay off in deep sleep (%s)", (int)pin, esp_err_to_name(err));
    }
}

/* Input with the pull-up on, then held. The pin does not drive the line,
 * so a module pull-up to 3.3 V draws nothing while the level stays high. */
static void pin_release_high(gpio_num_t pin)
{
    gpio_hold_dis(pin);
    gpio_config_t cfg = {
        .pin_bit_mask = 1ULL << pin,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&cfg);
    esp_err_t err = gpio_hold_en(pin);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "GPIO %d will not stay high in deep sleep (%s)", (int)pin, esp_err_to_name(err));
    }
}

static void bus_close(void)
{
    if (s_dev) {
        i2c_master_bus_rm_device(s_dev);
        s_dev = NULL;
    }
    if (s_bus) {
        i2c_del_master_bus(s_bus);
        s_bus = NULL;
    }
}

static esp_err_t bus_open(void)
{
    if (s_dev) {
        return ESP_OK;
    }
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port = I2C_NUM_0,
        .sda_io_num = I2C_SDA_GPIO,
        .scl_io_num = I2C_SCL_GPIO,
        .clk_source = I2C_CLK_SRC_DEFAULT,
        .glitch_ignore_cnt = 7,
        .flags.enable_internal_pullup = true,
    };
    ESP_RETURN_ON_ERROR(i2c_new_master_bus(&bus_cfg, &s_bus), TAG, "i2c bus");
    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = VL53L0X_I2C_ADDR,
        .scl_speed_hz = 400000,
    };
    return i2c_master_bus_add_device(s_bus, &dev_cfg, &s_dev);
}

void vl53l0x_shutdown(void)
{
    bus_close();
    /* Leave SDA and SCL undriven. The pull-ups hold them high so they
     * do not drain the module while XSHUT is low. */
    pin_release_high(I2C_SDA_GPIO);
    pin_release_high(I2C_SCL_GPIO);
    pin_hold(VL53L0X_XSHUT_GPIO, 0);
    pin_hold(VL53L0X_POWER_GPIO, VL53L0X_POWER_ON_LEVEL ? 0 : 1);
}

esp_err_t vl53l0x_begin(void)
{
    gpio_hold_dis(VL53L0X_POWER_GPIO);
    gpio_hold_dis(VL53L0X_XSHUT_GPIO);
    gpio_hold_dis(I2C_SDA_GPIO);
    gpio_hold_dis(I2C_SCL_GPIO);

    /* XSHUT stays low until VIN is valid. The datasheet requires that order. */
    pin_output(VL53L0X_XSHUT_GPIO, 0);
    gpio_config_t idle_i2c = {
        .pin_bit_mask = (1ULL << I2C_SDA_GPIO) | (1ULL << I2C_SCL_GPIO),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&idle_i2c);
    pin_output(VL53L0X_POWER_GPIO, VL53L0X_POWER_ON_LEVEL);
    vTaskDelay(pdMS_TO_TICKS(5));

    esp_err_t err = bus_open();
    if (err == ESP_OK) {
        gpio_set_level(VL53L0X_XSHUT_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(2));
        err = sensor_init();
    }
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Sensor init failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t vl53l0x_read_once(uint16_t *distance_mm)
{
    esp_err_t err = read_single(distance_mm);
    if (err != ESP_OK) {
        ESP_LOGW(TAG, "Measurement failed: %s", esp_err_to_name(err));
    }
    return err;
}
