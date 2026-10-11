#include "app_config.h"
#include "battery.h"
#include "vl53l0x.h"
#include "water_level.h"

#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_sleep.h"
#include "esp_timer.h"
#include "esp_zigbee.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "nvs_flash.h"
#include "soc/soc_caps.h"

#include <string.h>

static const char *TAG = "waterlevel";

#define RTC_MAGIC 0x574C5631u

typedef struct {
    uint32_t magic;
    uint32_t sec_since_report;
    uint32_t sec_until_join_retry;
    int32_t last_reported_mm;
    uint8_t valid_report;
} rtc_state_t;

static RTC_DATA_ATTR rtc_state_t s_rtc;

static esp_timer_handle_t s_sleep_timer;
static esp_timer_handle_t s_report_timer;
static esp_timer_handle_t s_steer_timer;
static bool s_factory_reset;
static bool s_report_due;
static int s_steer_attempts;

static int32_t iabs32(int32_t value)
{
    return value < 0 ? -value : value;
}

static void enter_deep_sleep(void)
{
    vl53l0x_shutdown();

    gpio_config_t button = {
        .pin_bit_mask = 1ULL << FACTORY_RESET_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button);

    int64_t sleep_us = (int64_t)MEASURE_INTERVAL_SEC * 1000000LL - esp_timer_get_time();
    if (sleep_us < 1000000) {
        sleep_us = 1000000;
    }
    esp_sleep_enable_timer_wakeup((uint64_t)sleep_us);

#if SOC_GPIO_SUPPORT_DEEPSLEEP_WAKEUP
    esp_err_t wake_err = esp_deep_sleep_enable_gpio_wakeup(1ULL << FACTORY_RESET_GPIO, ESP_GPIO_WAKEUP_GPIO_LOW);
    if (wake_err != ESP_OK) {
        ESP_LOGW(TAG, "Button will not wake the chip from deep sleep (%s)", esp_err_to_name(wake_err));
    }
#endif

    ESP_LOGI(TAG, "Deep sleep for %lld s", (long long)(sleep_us / 1000000));
    vTaskDelay(pdMS_TO_TICKS(30));
    esp_deep_sleep_start();
}

static void schedule_sleep(uint32_t seconds)
{
    esp_timer_stop(s_sleep_timer);
    esp_timer_start_once(s_sleep_timer, (uint64_t)seconds * 1000000ULL);
}

static void on_zigbee_activity(void)
{
    /* A write or query arrived. Stay up long enough for another one. */
    schedule_sleep(AWAKE_AFTER_REPORT_SEC);
}

static void report_timer_cb(void *arg)
{
    (void)arg;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    bool reported = false;
    if (s_report_due) {
        reported = water_send_level_report();
    }
    water_send_battery_report();
    esp_zigbee_lock_release();

    if (reported) {
        s_rtc.last_reported_mm = water_sample()->depth_mm;
        s_rtc.valid_report = 1;
        s_rtc.sec_since_report = 0;
        s_report_due = false;
    }
}

static void sleep_timer_cb(void *arg)
{
    (void)arg;
    enter_deep_sleep();
}

static void steer_timer_cb(void *arg)
{
    (void)arg;
    esp_zigbee_lock_acquire(portMAX_DELAY);
    ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
    esp_zigbee_lock_release();
}

static void start_reporting(bool first_join)
{
    water_network_set_joined(true);
    s_rtc.sec_until_join_retry = 0;
    esp_timer_stop(s_report_timer);
    esp_timer_start_once(s_report_timer, 1500 * 1000);
    schedule_sleep(first_join ? AWAKE_AFTER_JOIN_SEC : AWAKE_AFTER_REPORT_SEC);
}

static bool zigbee_signal_handler(const ezb_app_signal_t *signal)
{
    ezb_app_signal_type_t type = ezb_app_signal_get_type(signal);

    switch (type) {
    case EZB_ZDO_SIGNAL_SKIP_STARTUP:
        ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
        break;

    case EZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case EZB_BDB_SIGNAL_DEVICE_REBOOT: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        if (status != EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGW(TAG, "Stack start failed (0x%02x), retrying", status);
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_INITIALIZATION);
            break;
        }
        if (s_factory_reset && !ezb_bdb_is_factory_new()) {
            ESP_LOGW(TAG, "Factory reset");
            water_network_set_joined(false);
            s_rtc.magic = 0;
            esp_zigbee_factory_reset();
        }
        if (ezb_bdb_is_factory_new()) {
            ESP_LOGI(TAG, "Pairing. Permit join on the coordinator.");
            s_steer_attempts = 0;
            ezb_bdb_start_top_level_commissioning(EZB_BDB_MODE_NETWORK_STEERING);
        } else {
            ESP_LOGI(TAG, "Rejoined, short address 0x%04x", ezb_nwk_get_short_address());
            start_reporting(false);
        }
        break;
    }

    case EZB_BDB_SIGNAL_STEERING: {
        ezb_bdb_comm_status_t status = *(ezb_bdb_comm_status_t *)ezb_app_signal_get_params(signal);
        if (status == EZB_BDB_STATUS_SUCCESS) {
            ESP_LOGI(TAG, "Joined PAN 0x%04x on channel %d", ezb_nwk_get_panid(), ezb_nwk_get_current_channel());
            start_reporting(true);
        } else if (s_steer_attempts++ < 3) {
            ESP_LOGW(TAG, "Join failed (0x%02x), retry %d", status, s_steer_attempts);
            esp_timer_stop(s_steer_timer);
            esp_timer_start_once(s_steer_timer, 5 * 1000000ULL);
        } else {
            ESP_LOGW(TAG, "Join failed. Next attempt after the report interval.");
            s_rtc.sec_until_join_retry = REPORT_INTERVAL_SEC;
            schedule_sleep(1);
        }
        break;
    }

    case EZB_ZDO_SIGNAL_LEAVE:
        ESP_LOGI(TAG, "Left the network");
        water_network_set_joined(false);
        s_rtc.sec_until_join_retry = 0;
        break;

    default:
        ESP_LOGI(TAG, "Zigbee signal %s", ezb_app_signal_to_string(type));
        break;
    }
    return true;
}

static void zigbee_task(void *arg)
{
    (void)arg;
    esp_zigbee_config_t config = {
        .device_config =
            {
                .device_type = EZB_NWK_DEVICE_TYPE_END_DEVICE,
                .install_code_policy = false,
                .zed_config =
                    {
                        .ed_timeout = EZB_NWK_ED_TIMEOUT_256MIN,
                        .keep_alive = 1000,
                    },
            },
        .platform_config =
            {
                .storage_partition_name = "nvs",
                .radio_config.radio_mode = ESP_ZIGBEE_RADIO_MODE_NATIVE,
            },
    };

    ESP_ERROR_CHECK(esp_zigbee_init(&config));
    ezb_aps_secur_enable_distributed_security(false);
    ESP_ERROR_CHECK(ezb_bdb_set_primary_channel_set(ZIGBEE_CHANNEL_MASK));
    ESP_ERROR_CHECK(ezb_bdb_set_secondary_channel_set(ZIGBEE_CHANNEL_MASK));
    ezb_bdb_set_scan_duration(3);
    ESP_ERROR_CHECK(ezb_app_signal_add_handler(zigbee_signal_handler));
    ezb_nwk_set_rx_on_when_idle(false);
    ESP_ERROR_CHECK(water_zigbee_device_create());
    ESP_ERROR_CHECK(esp_zigbee_start(false));
    esp_zigbee_launch_mainloop();
    vTaskDelete(NULL);
}

static void rtc_tick(void)
{
    if (s_rtc.magic != RTC_MAGIC) {
        memset(&s_rtc, 0, sizeof(s_rtc));
        s_rtc.magic = RTC_MAGIC;
        return;
    }
    s_rtc.sec_since_report += MEASURE_INTERVAL_SEC;
    if (s_rtc.sec_until_join_retry > MEASURE_INTERVAL_SEC) {
        s_rtc.sec_until_join_retry -= MEASURE_INTERVAL_SEC;
    } else {
        s_rtc.sec_until_join_retry = 0;
    }
}

static bool button_requests_reset(void)
{
    gpio_config_t button = {
        .pin_bit_mask = 1ULL << FACTORY_RESET_GPIO,
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    gpio_config(&button);

    uint32_t causes = esp_sleep_get_wakeup_causes();
    bool woke_from_button = (causes & BIT(ESP_SLEEP_WAKEUP_GPIO)) != 0;
    bool held = gpio_get_level(FACTORY_RESET_GPIO) == 0;
    if (woke_from_button) {
        ESP_LOGI(TAG, "Woke from the reset button");
    } else if (causes & BIT(ESP_SLEEP_WAKEUP_TIMER)) {
        ESP_LOGI(TAG, "Woke from the measurement timer");
    }
    return woke_from_button || held;
}

static int32_t median_mm(uint16_t *values, int count)
{
    for (int i = 0; i < count; i++) {
        for (int j = i + 1; j < count; j++) {
            if (values[j] < values[i]) {
                uint16_t swap = values[i];
                values[i] = values[j];
                values[j] = swap;
            }
        }
    }
    return values[count / 2];
}

static void measure(void)
{
    uint16_t battery_mv = 0;
    uint8_t battery_percent = 0;
    if (battery_read(&battery_mv, &battery_percent) != ESP_OK) {
        battery_mv = 0;
        battery_percent = 0;
    }

    bool distance_valid = false;
    int32_t distance_mm = 0;
    if (vl53l0x_begin() == ESP_OK) {
        uint16_t samples[3];
        int count = 0;
        for (int i = 0; i < 3; i++) {
            uint16_t mm = 0;
            if (vl53l0x_read_once(&mm) == ESP_OK && mm < VL53L0X_INVALID_MM) {
                samples[count++] = mm;
            }
        }
        if (count > 0) {
            distance_mm = median_mm(samples, count);
            distance_valid = true;
        }
    }
    vl53l0x_shutdown();
    water_set_reading(distance_valid, distance_mm, battery_percent, battery_mv);
}

static bool report_due(void)
{
    const water_sample_t *sample = water_sample();
    if (s_rtc.sec_since_report >= REPORT_INTERVAL_SEC) {
        return true;
    }
    if (!sample->distance_valid) {
        return false;
    }
    if (!s_rtc.valid_report) {
        return true;
    }
    float delta_cm = (float)iabs32(sample->depth_mm - s_rtc.last_reported_mm) / 10.0f;
    return delta_cm > LEVEL_CHANGE_THRESHOLD_CM;
}

static void init_nvs(void)
{
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    ESP_ERROR_CHECK(err);
}

static void create_timers(void)
{
    const esp_timer_create_args_t sleep_args = {
        .callback = sleep_timer_cb,
        .name = "sleep",
    };
    const esp_timer_create_args_t report_args = {
        .callback = report_timer_cb,
        .name = "report",
    };
    const esp_timer_create_args_t steer_args = {
        .callback = steer_timer_cb,
        .name = "steer",
    };
    ESP_ERROR_CHECK(esp_timer_create(&sleep_args, &s_sleep_timer));
    ESP_ERROR_CHECK(esp_timer_create(&report_args, &s_report_timer));
    ESP_ERROR_CHECK(esp_timer_create(&steer_args, &s_steer_timer));
}

void app_main(void)
{
    init_nvs();
    water_settings_init();
    rtc_tick();
    s_factory_reset = button_requests_reset();
    measure();

    bool joined = water_network_joined();
    bool join_due = !joined && s_rtc.sec_until_join_retry == 0;
    s_report_due = report_due() || join_due;
    bool radio = s_factory_reset || join_due || (joined && s_report_due);

    const water_sample_t *sample = water_sample();
    ESP_LOGI(TAG, "Depth %ld mm, report %s, joined %s, radio %s", (long)sample->depth_mm, s_report_due ? "yes" : "no",
             joined ? "yes" : "no", radio ? "on" : "off");

    if (!radio) {
        enter_deep_sleep();
    }

    create_timers();
    water_set_activity_callback(on_zigbee_activity);
    schedule_sleep(90);
    xTaskCreate(zigbee_task, "zigbee", 12288, NULL, 5, NULL);
}
