#include "battery.h"

#include "app_config.h"

#include "driver/gpio.h"
#include "esp_adc/adc_cali.h"
#include "esp_adc/adc_cali_scheme.h"
#include "esp_adc/adc_oneshot.h"
#include "esp_check.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "battery";

static adc_channel_t battery_channel(void)
{
    switch (BATTERY_ADC_GPIO) {
    case 1:
        return ADC_CHANNEL_0;
    case 2:
        return ADC_CHANNEL_1;
    case 3:
        return ADC_CHANNEL_2;
    case 4:
        return ADC_CHANNEL_3;
    case 5:
        return ADC_CHANNEL_4;
    default:
        return ADC_CHANNEL_0;
    }
}

static void sink_set(bool measuring)
{
    if (BATTERY_SINK_GPIO < 0) {
        return;
    }
    if (measuring) {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << BATTERY_SINK_GPIO,
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
        gpio_set_level(BATTERY_SINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(2));
    } else {
        gpio_config_t cfg = {
            .pin_bit_mask = 1ULL << BATTERY_SINK_GPIO,
            .mode = GPIO_MODE_INPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        gpio_config(&cfg);
    }
}

esp_err_t battery_read(uint16_t *millivolts, uint8_t *percent)
{
    adc_channel_t channel = battery_channel();
    adc_oneshot_unit_handle_t unit = NULL;
    adc_cali_handle_t cali = NULL;
    bool calibrated = false;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = ADC_UNIT_1,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };
    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &unit), TAG, "adc unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = ADC_ATTEN_DB_12,
        .bitwidth = ADC_BITWIDTH_12,
    };
    esp_err_t err = adc_oneshot_config_channel(unit, channel, &chan_cfg);

#if ADC_CALI_SCHEME_CURVE_FITTING_SUPPORTED
    if (err == ESP_OK) {
        adc_cali_curve_fitting_config_t cal_cfg = {
            .unit_id = ADC_UNIT_1,
            .atten = ADC_ATTEN_DB_12,
            .bitwidth = ADC_BITWIDTH_12,
        };
        if (adc_cali_create_scheme_curve_fitting(&cal_cfg, &cali) == ESP_OK) {
            calibrated = true;
        }
    }
#endif

    sink_set(true);

    int32_t sum_mv = 0;
    const int samples = 8;
    for (int i = 0; err == ESP_OK && i < samples; i++) {
        int raw = 0;
        err = adc_oneshot_read(unit, channel, &raw);
        if (err != ESP_OK) {
            break;
        }
        int mv = 0;
        if (calibrated) {
            err = adc_cali_raw_to_voltage(cali, raw, &mv);
        } else {
            mv = (raw * 3300) / 4095;
        }
        sum_mv += mv;
    }

    sink_set(false);

    if (calibrated) {
        adc_cali_delete_scheme_curve_fitting(cali);
    }
    adc_oneshot_del_unit(unit);

    if (err != ESP_OK) {
        ESP_LOGW(TAG, "ADC read failed: %s", esp_err_to_name(err));
        return err;
    }

    int32_t pin_mv = sum_mv / samples;
    int32_t pack_mv = pin_mv * (BATTERY_DIVIDER_HIGH + BATTERY_DIVIDER_LOW) / BATTERY_DIVIDER_LOW;
    if (pack_mv < 0) {
        pack_mv = 0;
    }
    if (pack_mv > 65535) {
        pack_mv = 65535;
    }

    int pct;
    if (pack_mv <= BATTERY_EMPTY_MV) {
        pct = 0;
    } else if (pack_mv >= BATTERY_FULL_MV) {
        pct = 100;
    } else {
        pct = (int)((pack_mv - BATTERY_EMPTY_MV) * 100 / (BATTERY_FULL_MV - BATTERY_EMPTY_MV));
    }

    *millivolts = (uint16_t)pack_mv;
    *percent = (uint8_t)pct;
    ESP_LOGI(TAG, "Battery %d mV (%d%%)", (int)pack_mv, pct);
    return ESP_OK;
}
