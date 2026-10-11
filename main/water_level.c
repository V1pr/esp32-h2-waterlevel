#include "water_level.h"

#include "app_config.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "water";

#define NVS_NAMESPACE "water"

/* Bacchus custom attributes. zigbee2mqtt reads these without a manufacturer code. */
#define ATTR_TANK_HEIGHT 0xF005
#define ATTR_FILLING 0xF006
#define ATTR_MEASURE_PERIOD 0xF007
#define ATTR_INVERT 0xF008

#define ACCESS_STATE (EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_REPORTING)
#define ACCESS_SETTING (EZB_ZCL_ATTR_ACCESS_READ | EZB_ZCL_ATTR_ACCESS_WRITE)

typedef struct {
    uint16_t height_cm;
    uint8_t alarm_min;
    uint8_t alarm_max;
    uint8_t invert;
    uint16_t period_min;
    uint8_t joined;
} settings_t;

static settings_t s_cfg;
static water_sample_t s_sample;
static int32_t s_distance_mm;
static bool s_distance_valid;
static uint8_t s_batt_pct_zcl;
static uint8_t s_batt_volt_zcl;
static water_activity_cb_t s_activity;
static bool s_applying_write;

static float s_present_mm;
static uint8_t s_out_of_service;
static uint8_t s_status_flags;
static float s_alarm_min;
static float s_alarm_max;
static float s_tank_height_mm;
static float s_filling;
static uint16_t s_period_min;
static uint8_t s_invert;
static int16_t s_temperature;
static uint8_t s_manufacturer[1 + sizeof(ZIGBEE_MANUFACTURER_TEXT)];
static uint8_t s_model[1 + sizeof(ZIGBEE_MODEL_TEXT)];

static void save_settings(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u16(handle, "height", s_cfg.height_cm);
    nvs_set_u8(handle, "maxpct", s_cfg.alarm_max);
    nvs_set_u8(handle, "minpct", s_cfg.alarm_min);
    nvs_set_u8(handle, "invert", s_cfg.invert);
    nvs_set_u16(handle, "period", s_cfg.period_min);
    nvs_set_u8(handle, "joined", s_cfg.joined);
    nvs_commit(handle);
    nvs_close(handle);
}

static uint8_t clamp_percent(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 100) {
        return 100;
    }
    return (uint8_t)value;
}

static uint16_t clamp_height_cm(int value)
{
    if (value < 0) {
        return 0;
    }
    if (value > 450) {
        return 450;
    }
    return (uint16_t)value;
}

static int round_to_int(float value)
{
    if (value >= 0.0f) {
        return (int)(value + 0.5f);
    }
    return (int)(value - 0.5f);
}

static void sync_zcl_values(void)
{
    s_present_mm = s_sample.distance_valid ? (float)s_sample.depth_mm : 0.0f;
    s_out_of_service = s_sample.distance_valid ? 0 : 1;
    s_status_flags = s_out_of_service ? EZB_ZCL_ANALOG_INPUT_STATUS_FLAGS_OUT_OF_SERVICE : 0;
    s_filling = (float)s_sample.percent;
    s_tank_height_mm = (float)s_cfg.height_cm * 10.0f;
    s_alarm_min = (float)s_cfg.alarm_min;
    s_alarm_max = (float)s_cfg.alarm_max;
    s_period_min = s_cfg.period_min;
    s_invert = s_cfg.invert ? 1 : 0;
}

static void recompute(void)
{
    memset(&s_sample, 0, sizeof(s_sample));
    s_sample.distance_valid = s_distance_valid;
    s_sample.distance_mm = s_distance_mm;
    s_sample.battery_percent = s_batt_pct_zcl / 2;
    s_sample.battery_mv = (uint16_t)s_batt_volt_zcl * 100;

    int32_t depth_mm = 0;
    if (s_distance_valid) {
        depth_mm = (int32_t)s_cfg.height_cm * 10 - s_distance_mm;
        if (depth_mm < 0) {
            depth_mm = 0;
        }
    }
    s_sample.depth_mm = depth_mm;

    int32_t full_mm = (int32_t)s_cfg.height_cm * 10;
    if (full_mm < 1) {
        full_mm = 1;
    }
    int percent = (int)(depth_mm * 100 / full_mm);
    if (percent > 100) {
        percent = 100;
    }
    if (!s_distance_valid) {
        percent = 0;
    }
    s_sample.percent = (uint8_t)percent;
    sync_zcl_values();
}

static bool nvs_u16(nvs_handle_t handle, const char *key, uint16_t *dest)
{
    uint16_t value = 0;
    if (nvs_get_u16(handle, key, &value) != ESP_OK) {
        return false;
    }
    *dest = value;
    return true;
}

static bool nvs_u8(nvs_handle_t handle, const char *key, uint8_t *dest)
{
    uint8_t value = 0;
    if (nvs_get_u8(handle, key, &value) != ESP_OK) {
        return false;
    }
    *dest = value;
    return true;
}

void water_settings_init(void)
{
    s_cfg.height_cm = DEFAULT_TANK_HEIGHT_CM;
    s_cfg.alarm_max = DEFAULT_ALARM_MAX_PERCENT;
    s_cfg.alarm_min = DEFAULT_ALARM_MIN_PERCENT;
    s_cfg.invert = 0;
    s_cfg.period_min = (uint16_t)(REPORT_INTERVAL_SEC / 60u);
    s_cfg.joined = 0;
    s_temperature = 0;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        if (!nvs_u16(handle, "height", &s_cfg.height_cm)) {
            /* Earlier firmware stored the sensor height as "install". */
            nvs_u16(handle, "install", &s_cfg.height_cm);
        }
        if (!nvs_u8(handle, "maxpct", &s_cfg.alarm_max)) {
            nvs_u8(handle, "maxset", &s_cfg.alarm_max);
        }
        if (!nvs_u8(handle, "minpct", &s_cfg.alarm_min)) {
            nvs_u8(handle, "minset", &s_cfg.alarm_min);
        }
        nvs_u8(handle, "invert", &s_cfg.invert);
        nvs_u16(handle, "period", &s_cfg.period_min);
        nvs_u8(handle, "joined", &s_cfg.joined);
        nvs_close(handle);
    }
    s_cfg.height_cm = clamp_height_cm(s_cfg.height_cm);
    s_cfg.alarm_min = clamp_percent(s_cfg.alarm_min);
    s_cfg.alarm_max = clamp_percent(s_cfg.alarm_max);
    if (s_cfg.period_min > 3600) {
        s_cfg.period_min = 3600;
    }
    sync_zcl_values();
    ESP_LOGI(TAG, "Tank height %u cm, alarm %u-%u%%", s_cfg.height_cm, s_cfg.alarm_min, s_cfg.alarm_max);
}

bool water_network_joined(void)
{
    return s_cfg.joined != 0;
}

void water_network_set_joined(bool joined)
{
    s_cfg.joined = joined ? 1 : 0;
    save_settings();
}

void water_set_activity_callback(water_activity_cb_t cb)
{
    s_activity = cb;
}

void water_set_reading(bool distance_valid, int32_t distance_mm, uint8_t battery_percent, uint16_t battery_mv)
{
    s_distance_valid = distance_valid;
    s_distance_mm = distance_mm;
    if (battery_percent > 100) {
        battery_percent = 100;
    }
    s_batt_pct_zcl = (uint8_t)(battery_percent * 2);
    s_batt_volt_zcl = (uint8_t)(battery_mv / 100);
    recompute();
    ESP_LOGI(TAG, "Distance %ld mm, depth %ld mm (%u%%)", (long)s_sample.distance_mm, (long)s_sample.depth_mm,
             s_sample.percent);
}

const water_sample_t *water_sample(void)
{
    return &s_sample;
}

static void set_server_attr(uint16_t cluster, uint16_t attr, const void *value)
{
    ezb_zcl_attr_desc_t desc = ezb_zcl_get_attr_desc(ZIGBEE_ENDPOINT, cluster, EZB_ZCL_CLUSTER_SERVER, attr,
                                                     EZB_ZCL_STD_MANUF_CODE);
    if (desc) {
        ezb_zcl_attr_desc_set_value(desc, value);
    }
}

static bool report_attr(uint16_t cluster, uint16_t attr)
{
    ezb_zcl_report_attr_cmd_t report = {
        .cmd_ctrl =
            {
                .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                .dst_addr.addr_mode = EZB_ADDR_MODE_SHORT,
                .dst_addr.u.short_addr = 0x0000,
                .src_ep = ZIGBEE_ENDPOINT,
                .dst_ep = 1,
                .cluster_id = cluster,
            },
        .payload.attr_id = attr,
    };
    ezb_err_t ret = ezb_zcl_report_attr_cmd_req(&report);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Report cluster 0x%04x attr 0x%04x failed (0x%04x)", cluster, attr, ret);
        return false;
    }
    return true;
}

bool water_send_level_report(void)
{
    sync_zcl_values();
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &s_present_mm);
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_OUT_OF_SERVICE_ID, &s_out_of_service);
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_STATUS_FLAGS_ID, &s_status_flags);
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ATTR_FILLING, &s_filling);

    ESP_LOGI(TAG, "Reporting %.1f cm, %u%%", s_present_mm / 10.0f, s_sample.percent);
    bool level = report_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID);
    bool filling = report_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ATTR_FILLING);
    bool service = report_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_OUT_OF_SERVICE_ID);
    return level && filling && service;
}

void water_send_battery_report(void)
{
    set_server_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                    &s_batt_pct_zcl);
    set_server_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &s_batt_volt_zcl);
    report_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID);
    report_attr(EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID);
    ESP_LOGI(TAG, "Battery report %u%%, %u mV", s_sample.battery_percent, s_sample.battery_mv);
}

static void push_setting(uint16_t cluster, uint16_t attr, const void *value)
{
    s_applying_write = true;
    set_server_attr(cluster, attr, value);
    s_applying_write = false;
}

static void on_attr_write(ezb_zcl_set_attr_value_message_t *message)
{
    if (!message || !message->in.attribute.data.value || s_applying_write) {
        return;
    }
    if (s_activity) {
        s_activity();
    }

    uint16_t cluster = message->info.cluster_id;
    uint16_t attr = message->in.attribute.id;
    const void *raw = message->in.attribute.data.value;
    bool stored = false;

    if (cluster == EZB_ZCL_CLUSTER_ID_ANALOG_INPUT && attr == ATTR_TANK_HEIGHT &&
        message->in.attribute.data.size >= sizeof(float)) {
        float written = 0.0f;
        memcpy(&written, raw, sizeof(written));
        s_cfg.height_cm = clamp_height_cm(round_to_int(written / 10.0f));
        stored = true;
        ESP_LOGI(TAG, "Tank height %u cm", s_cfg.height_cm);
    } else if (cluster == EZB_ZCL_CLUSTER_ID_ANALOG_INPUT &&
               (attr == EZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID ||
                attr == EZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID) &&
               message->in.attribute.data.size >= sizeof(float)) {
        float written = 0.0f;
        memcpy(&written, raw, sizeof(written));
        uint8_t percent = clamp_percent(round_to_int(written));
        if (attr == EZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID) {
            s_cfg.alarm_min = percent;
        } else {
            s_cfg.alarm_max = percent;
        }
        stored = true;
        ESP_LOGI(TAG, "Alarm thresholds %u-%u%%", s_cfg.alarm_min, s_cfg.alarm_max);
    } else if (cluster == EZB_ZCL_CLUSTER_ID_ANALOG_INPUT && attr == ATTR_MEASURE_PERIOD &&
               message->in.attribute.data.size >= sizeof(uint16_t)) {
        uint16_t minutes = 0;
        memcpy(&minutes, raw, sizeof(minutes));
        if (minutes > 3600) {
            minutes = 3600;
        }
        s_cfg.period_min = minutes;
        stored = true;
        ESP_LOGI(TAG, "Stored measurement period %u min", s_cfg.period_min);
    } else if (cluster == EZB_ZCL_CLUSTER_ID_ON_OFF && attr == ATTR_INVERT &&
               message->in.attribute.data.size >= sizeof(uint8_t)) {
        s_cfg.invert = (*(const uint8_t *)raw) ? 1 : 0;
        stored = true;
        ESP_LOGI(TAG, "Invert threshold %u", s_cfg.invert);
    }

    if (!stored) {
        return;
    }
    save_settings();
    recompute();
    push_setting(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ATTR_TANK_HEIGHT, &s_tank_height_mm);
    push_setting(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, &s_alarm_min);
    push_setting(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, &s_alarm_max);
    push_setting(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ATTR_MEASURE_PERIOD, &s_period_min);
    push_setting(EZB_ZCL_CLUSTER_ID_ON_OFF, ATTR_INVERT, &s_invert);
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, &s_present_mm);
    set_server_attr(EZB_ZCL_CLUSTER_ID_ANALOG_INPUT, ATTR_FILLING, &s_filling);
}

static void zcl_core_action(ezb_zcl_core_action_callback_id_t callback_id, void *message)
{
    if (callback_id == EZB_ZCL_CORE_SET_ATTR_VALUE_CB_ID) {
        on_attr_write((ezb_zcl_set_attr_value_message_t *)message);
    }
}

static void put_zcl_string(uint8_t *dst, const char *text)
{
    size_t length = strlen(text);
    dst[0] = (uint8_t)length;
    memcpy(&dst[1], text, length);
}

static void force_access(ezb_zcl_cluster_desc_t cluster, uint16_t attr_id, uint8_t access)
{
    ezb_zcl_attr_desc_t attr = ezb_zcl_cluster_get_attr_desc(cluster, attr_id, EZB_ZCL_STD_MANUF_CODE);
    if (attr) {
        ezb_zcl_attr_desc_set_access(attr, access);
    }
}

esp_err_t water_zigbee_device_create(void)
{
    sync_zcl_values();
    put_zcl_string(s_manufacturer, ZIGBEE_MANUFACTURER_TEXT);
    put_zcl_string(s_model, ZIGBEE_MODEL_TEXT);

    ezb_af_device_desc_t dev = ezb_af_create_device_desc();

    ezb_zcl_basic_cluster_server_config_t basic_cfg = {
        .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };
    ezb_zcl_cluster_desc_t basic = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, s_manufacturer);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, s_model);

    ezb_zcl_cluster_desc_t power = ezb_zcl_power_config_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_power_config_cluster_desc_add_attr(power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &s_batt_volt_zcl);
    ezb_zcl_power_config_cluster_desc_add_attr(power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                               &s_batt_pct_zcl);

    ezb_zcl_analog_input_cluster_server_config_t analog_cfg = {
        .out_of_service = s_out_of_service != 0,
        .present_value = s_present_mm,
        .status_flags = s_status_flags,
    };
    ezb_zcl_cluster_desc_t analog = ezb_zcl_analog_input_create_cluster_desc(&analog_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_ERROR_CHECK(ezb_zcl_analog_input_cluster_desc_add_attr(analog, EZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID,
                                                               &s_alarm_min));
    ESP_ERROR_CHECK(ezb_zcl_analog_input_cluster_desc_add_attr(analog, EZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID,
                                                               &s_alarm_max));
    ESP_ERROR_CHECK(ezb_zcl_cluster_desc_add_manuf_attr(analog, ATTR_TANK_HEIGHT, EZB_ZCL_ATTR_TYPE_SINGLE, ACCESS_SETTING,
                                                        EZB_ZCL_STD_MANUF_CODE, &s_tank_height_mm));
    ESP_ERROR_CHECK(ezb_zcl_cluster_desc_add_manuf_attr(analog, ATTR_FILLING, EZB_ZCL_ATTR_TYPE_SINGLE, ACCESS_STATE,
                                                        EZB_ZCL_STD_MANUF_CODE, &s_filling));
    ESP_ERROR_CHECK(ezb_zcl_cluster_desc_add_manuf_attr(analog, ATTR_MEASURE_PERIOD, EZB_ZCL_ATTR_TYPE_UINT16,
                                                        ACCESS_SETTING, EZB_ZCL_STD_MANUF_CODE, &s_period_min));
    force_access(analog, EZB_ZCL_ATTR_ANALOG_INPUT_PRESENT_VALUE_ID, ACCESS_STATE);
    force_access(analog, EZB_ZCL_ATTR_ANALOG_INPUT_OUT_OF_SERVICE_ID, ACCESS_STATE);
    force_access(analog, EZB_ZCL_ATTR_ANALOG_INPUT_MIN_PRESENT_VALUE_ID, ACCESS_SETTING);
    force_access(analog, EZB_ZCL_ATTR_ANALOG_INPUT_MAX_PRESENT_VALUE_ID, ACCESS_SETTING);

    ezb_zcl_temperature_measurement_cluster_server_config_t temp_cfg = {
        .measured_value = s_temperature,
        .min_measured_value = (int16_t)0x8000,
        .max_measured_value = (int16_t)0x8000,
    };
    ezb_zcl_cluster_desc_t temperature =
        ezb_zcl_temperature_measurement_create_cluster_desc(&temp_cfg, EZB_ZCL_CLUSTER_SERVER);

    ezb_zcl_on_off_cluster_server_config_t on_off_cfg = {
        .on_off = false,
    };
    ezb_zcl_cluster_desc_t on_off = ezb_zcl_on_off_create_cluster_desc(&on_off_cfg, EZB_ZCL_CLUSTER_SERVER);
    ESP_ERROR_CHECK(ezb_zcl_cluster_desc_add_manuf_attr(on_off, ATTR_INVERT, EZB_ZCL_ATTR_TYPE_BOOL, ACCESS_SETTING,
                                                        EZB_ZCL_STD_MANUF_CODE, &s_invert));

    ezb_af_ep_config_t ep_cfg = {
        .ep_id = ZIGBEE_ENDPOINT,
        .app_profile_id = EZB_AF_HA_PROFILE_ID,
        .app_device_id = ZIGBEE_DEVICE_ID,
        .app_device_version = 0,
    };
    ezb_af_ep_desc_t ep = ezb_af_create_endpoint_desc(&ep_cfg);
    ESP_RETURN_ON_FALSE(ep, ESP_FAIL, TAG, "endpoint");
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, basic));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, power));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, analog));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, temperature));
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, on_off));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));
    ezb_zcl_core_action_handler_register(zcl_core_action);
    return ESP_OK;
}
