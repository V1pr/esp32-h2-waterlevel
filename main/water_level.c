#include "water_level.h"

#include "app_config.h"

#include "esp_check.h"
#include "esp_log.h"
#include "esp_zigbee.h"
#include "nvs.h"

#include <string.h>

static const char *TAG = "water";

#define NVS_NAMESPACE "water"
#define TUYA_CLUSTER_ID 0xEF00
#define TUYA_BOOL 0x01
#define TUYA_VALUE 0x02
#define TUYA_ENUM 0x04
#define CMD_DATA_REQUEST 0x00
#define CMD_DATA_RESPONSE 0x01
#define CMD_DATA_REPORT 0x02
#define CMD_DATA_QUERY 0x03
#define CMD_SEND_DATA 0x04
#define CMD_MCU_VERSION_REQ 0x10
#define CMD_MCU_VERSION_RSP 0x11

typedef struct {
    uint16_t installation_cm;
    uint16_t depth_max_cm;
    uint8_t max_set;
    uint8_t min_set;
    uint8_t silent;
    uint8_t alarm_min;
    uint8_t joined;
} settings_t;

static settings_t s_cfg;
static water_sample_t s_sample;
static int32_t s_distance_mm;
static bool s_distance_valid;
static uint8_t s_batt_pct_zcl;
static uint8_t s_batt_volt_zcl;
static uint16_t s_seq;
static water_activity_cb_t s_activity;
static uint8_t s_frames[3][48];

static void save_settings(void)
{
    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READWRITE, &handle) != ESP_OK) {
        return;
    }
    nvs_set_u16(handle, "install", s_cfg.installation_cm);
    nvs_set_u16(handle, "maxdepth", s_cfg.depth_max_cm);
    nvs_set_u8(handle, "maxset", s_cfg.max_set);
    nvs_set_u8(handle, "minset", s_cfg.min_set);
    nvs_set_u8(handle, "silent", s_cfg.silent);
    nvs_set_u8(handle, "alarm", s_cfg.alarm_min);
    nvs_set_u8(handle, "joined", s_cfg.joined);
    nvs_commit(handle);
    nvs_close(handle);
}

static uint8_t clamp_u8(uint32_t value, uint8_t max_value)
{
    if (value > max_value) {
        return max_value;
    }
    return (uint8_t)value;
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
        depth_mm = (int32_t)s_cfg.installation_cm * 10 - s_distance_mm;
        if (depth_mm < 0) {
            depth_mm = 0;
        }
    }
    s_sample.depth_mm = depth_mm;

    int32_t max_mm = (int32_t)s_cfg.depth_max_cm * 10;
    if (max_mm < 1) {
        max_mm = 1;
    }
    int percent = (int)(depth_mm * 100 / max_mm);
    if (percent > 100) {
        percent = 100;
    }
    if (!s_distance_valid) {
        percent = 0;
    }
    s_sample.percent = (uint8_t)percent;

    if (s_sample.percent <= s_cfg.min_set) {
        s_sample.liquid_state = 1;
    } else if (s_sample.percent >= s_cfg.max_set) {
        s_sample.liquid_state = 2;
    } else {
        s_sample.liquid_state = 0;
    }
}

void water_settings_init(void)
{
    s_cfg.installation_cm = DEFAULT_INSTALLATION_HEIGHT_CM;
    s_cfg.depth_max_cm = DEFAULT_LIQUID_DEPTH_MAX_CM;
    s_cfg.max_set = DEFAULT_MAX_SET_PERCENT;
    s_cfg.min_set = DEFAULT_MIN_SET_PERCENT;
    s_cfg.silent = 0;
    s_cfg.alarm_min = DEFAULT_ALARM_MINUTES;
    s_cfg.joined = 0;

    nvs_handle_t handle;
    if (nvs_open(NVS_NAMESPACE, NVS_READONLY, &handle) == ESP_OK) {
        nvs_get_u16(handle, "install", &s_cfg.installation_cm);
        nvs_get_u16(handle, "maxdepth", &s_cfg.depth_max_cm);
        nvs_get_u8(handle, "maxset", &s_cfg.max_set);
        nvs_get_u8(handle, "minset", &s_cfg.min_set);
        nvs_get_u8(handle, "silent", &s_cfg.silent);
        nvs_get_u8(handle, "alarm", &s_cfg.alarm_min);
        nvs_get_u8(handle, "joined", &s_cfg.joined);
        nvs_close(handle);
    }
    ESP_LOGI(TAG, "Tank install %u cm, max depth %u cm, low %u%%, high %u%%", s_cfg.installation_cm,
             s_cfg.depth_max_cm, s_cfg.min_set, s_cfg.max_set);
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
    ESP_LOGI(TAG, "Distance %ld mm, depth %ld mm (%u%%, state %u)", (long)s_sample.distance_mm,
             (long)s_sample.depth_mm, s_sample.percent, s_sample.liquid_state);
}

const water_sample_t *water_sample(void)
{
    return &s_sample;
}

static size_t put_header(uint8_t *buf, size_t n, uint8_t dp, uint8_t type, uint8_t len)
{
    buf[n++] = dp;
    buf[n++] = type;
    buf[n++] = 0x00;
    buf[n++] = len;
    return n;
}

static size_t put_value(uint8_t *buf, size_t n, uint8_t dp, uint32_t value)
{
    n = put_header(buf, n, dp, TUYA_VALUE, 4);
    buf[n++] = (uint8_t)(value >> 24);
    buf[n++] = (uint8_t)(value >> 16);
    buf[n++] = (uint8_t)(value >> 8);
    buf[n++] = (uint8_t)value;
    return n;
}

static size_t put_enum(uint8_t *buf, size_t n, uint8_t dp, uint8_t value)
{
    n = put_header(buf, n, dp, TUYA_ENUM, 1);
    buf[n++] = value;
    return n;
}

static size_t put_bool(uint8_t *buf, size_t n, uint8_t dp, uint8_t value)
{
    n = put_header(buf, n, dp, TUYA_BOOL, 1);
    buf[n++] = value ? 1 : 0;
    return n;
}

static uint32_t depth_centimeters(void)
{
    /* Nearest centimeter. zigbee2mqtt divides this by 100 and shows meters. */
    if (!s_sample.distance_valid) {
        return 0;
    }
    int32_t cm = (s_sample.depth_mm + 5) / 10;
    if (cm < 0) {
        cm = 0;
    }
    return (uint32_t)cm;
}

static esp_err_t send_frame(uint8_t cmd, uint16_t seq, const uint8_t *body, size_t body_len, uint8_t *storage)
{
    storage[0] = (uint8_t)(seq & 0xff);
    storage[1] = (uint8_t)((seq >> 8) & 0xff);
    memcpy(&storage[2], body, body_len);

    ezb_zcl_custom_cluster_cmd_t req = {
        .cmd_ctrl =
            {
                .dst_addr =
                    {
                        .addr_mode = EZB_ADDR_MODE_SHORT,
                        .u.short_addr = 0x0000,
                    },
                .src_ep = ZIGBEE_ENDPOINT,
                .dst_ep = 1,
                .cluster_id = TUYA_CLUSTER_ID,
                .fc =
                    {
                        .direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                        .dis_default_rsp = (cmd == CMD_DATA_RESPONSE) ? 1 : 0,
                    },
            },
        .cmd_id = cmd,
        .data_length = (uint16_t)(body_len + 2),
        .data = storage,
    };
    ezb_err_t ret = ezb_zcl_custom_cluster_cmd_req(&req);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Tuya command 0x%02x failed (0x%04x)", cmd, ret);
        return ESP_FAIL;
    }
    return ESP_OK;
}

static bool send_all(uint8_t cmd, uint16_t seq)
{
    uint8_t body[40];
    size_t n = 0;
    bool ok = true;

    n = put_enum(body, 0, DP_LIQUID_STATE, s_sample.liquid_state);
    n = put_value(body, n, DP_LIQUID_DEPTH, depth_centimeters());
    n = put_value(body, n, DP_LIQUID_LEVEL_PERCENT, s_sample.percent);
    ok = send_frame(cmd, seq, body, n, s_frames[0]) == ESP_OK && ok;

    n = put_value(body, 0, DP_MAX_SET, s_cfg.max_set);
    n = put_value(body, n, DP_MIN_SET, s_cfg.min_set);
    n = put_value(body, n, DP_INSTALLATION_HEIGHT, s_cfg.installation_cm);
    n = put_value(body, n, DP_LIQUID_DEPTH_MAX, s_cfg.depth_max_cm);
    ok = send_frame(cmd, (uint16_t)(seq + 1), body, n, s_frames[1]) == ESP_OK && ok;

    n = put_bool(body, 0, DP_SILENT_MODE, s_cfg.silent);
    n = put_value(body, n, DP_ALARM_TIME, s_cfg.alarm_min);
    ok = send_frame(cmd, (uint16_t)(seq + 2), body, n, s_frames[2]) == ESP_OK && ok;

    s_seq = (uint16_t)(seq + 3);
    return ok;
}

bool water_send_tuya_report(void)
{
    ESP_LOGI(TAG, "Reporting depth %lu cm, %u%%", (unsigned long)depth_centimeters(), s_sample.percent);
    return send_all(CMD_DATA_REPORT, s_seq);
}

void water_send_battery_report(void)
{
    ezb_zcl_attr_desc_t percent = ezb_zcl_get_attr_desc(ZIGBEE_ENDPOINT, EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_CLUSTER_SERVER,
                                                        EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                                        EZB_ZCL_STD_MANUF_CODE);
    ezb_zcl_attr_desc_t voltage = ezb_zcl_get_attr_desc(ZIGBEE_ENDPOINT, EZB_ZCL_CLUSTER_ID_POWER_CONFIG, EZB_ZCL_CLUSTER_SERVER,
                                                        EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, EZB_ZCL_STD_MANUF_CODE);
    if (percent) {
        ezb_zcl_attr_desc_set_value(percent, &s_batt_pct_zcl);
    }
    if (voltage) {
        ezb_zcl_attr_desc_set_value(voltage, &s_batt_volt_zcl);
    }

    ezb_zcl_report_attr_cmd_t report = {
        .cmd_ctrl =
            {
                .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                .dst_addr.addr_mode = EZB_ADDR_MODE_SHORT,
                .dst_addr.u.short_addr = 0x0000,
                .src_ep = ZIGBEE_ENDPOINT,
                .dst_ep = 1,
                .cluster_id = EZB_ZCL_CLUSTER_ID_POWER_CONFIG,
            },
        .payload.attr_id = EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
    };
    ezb_err_t ret = ezb_zcl_report_attr_cmd_req(&report);
    if (ret != EZB_ERR_NONE) {
        ESP_LOGW(TAG, "Battery report failed (0x%04x)", ret);
    } else {
        ESP_LOGI(TAG, "Battery report %u%%", s_sample.battery_percent);
    }
}

static uint32_t read_be32(const uint8_t *data)
{
    return ((uint32_t)data[0] << 24) | ((uint32_t)data[1] << 16) | ((uint32_t)data[2] << 8) | data[3];
}

static void apply_dp(uint8_t dp, uint8_t datatype, const uint8_t *data, uint16_t len)
{
    if (datatype == TUYA_VALUE && len == 4) {
        uint32_t value = read_be32(data);
        switch (dp) {
        case DP_MAX_SET:
            s_cfg.max_set = clamp_u8(value, 100);
            break;
        case DP_MIN_SET:
            s_cfg.min_set = clamp_u8(value, 100);
            break;
        case DP_INSTALLATION_HEIGHT:
            s_cfg.installation_cm = value > 500 ? 500 : (uint16_t)value;
            break;
        case DP_LIQUID_DEPTH_MAX:
            s_cfg.depth_max_cm = value > 500 ? 500 : (uint16_t)value;
            break;
        case DP_ALARM_TIME:
            if (value < 1) {
                value = 1;
            }
            s_cfg.alarm_min = clamp_u8(value, 60);
            break;
        default:
            ESP_LOGI(TAG, "Ignoring value dp %u = %lu", dp, (unsigned long)value);
            return;
        }
        ESP_LOGI(TAG, "Set dp %u = %lu", dp, (unsigned long)value);
    } else if (datatype == TUYA_BOOL && len >= 1 && dp == DP_SILENT_MODE) {
        s_cfg.silent = data[0] ? 1 : 0;
        ESP_LOGI(TAG, "Silent mode %u", s_cfg.silent);
    } else {
        ESP_LOGI(TAG, "Ignoring dp %u type %u len %u", dp, datatype, len);
    }
}

static void parse_datapoints(const uint8_t *payload, uint16_t len)
{
    size_t i = 2;
    while (i + 4 <= len) {
        uint8_t dp = payload[i];
        uint8_t type = payload[i + 1];
        uint16_t dlen = (uint16_t)((payload[i + 2] << 8) | payload[i + 3]);
        i += 4;
        if (i + dlen > len) {
            break;
        }
        apply_dp(dp, type, &payload[i], dlen);
        i += dlen;
    }
    save_settings();
    recompute();
}

static ezb_zcl_status_t on_tuya_command(const ezb_zcl_cmd_hdr_t *header, const uint8_t *payload, uint16_t payload_length)
{
    if (!header || header->cluster_id != TUYA_CLUSTER_ID) {
        return EZB_ZCL_STATUS_UNSUP_CMD;
    }
    if (s_activity) {
        s_activity();
    }

    ESP_LOGI(TAG, "Tuya command 0x%02x, %u bytes", header->cmd_id, payload_length);

    if (header->cmd_id == CMD_MCU_VERSION_REQ) {
        uint16_t seq = 0;
        if (payload && payload_length >= 2) {
            seq = (uint16_t)(payload[0] | (payload[1] << 8));
        }
        uint8_t body[1] = {0x01};
        /* Version response is seq + one version byte, not a datapoint list. */
        uint8_t frame[3] = {(uint8_t)(seq & 0xff), (uint8_t)((seq >> 8) & 0xff), body[0]};
        ezb_zcl_custom_cluster_cmd_t req = {
            .cmd_ctrl =
                {
                    .dst_addr = header->src_addr,
                    .src_ep = header->dst_ep,
                    .dst_ep = header->src_ep ? header->src_ep : 1,
                    .cluster_id = TUYA_CLUSTER_ID,
                    .fc.direction = EZB_ZCL_CMD_DIRECTION_TO_CLI,
                    .fc.dis_default_rsp = 1,
                },
            .cmd_id = CMD_MCU_VERSION_RSP,
            .data_length = sizeof(frame),
            .data = frame,
        };
        static uint8_t version_frame[3];
        memcpy(version_frame, frame, sizeof(frame));
        req.data = version_frame;
        ezb_zcl_custom_cluster_cmd_req(&req);
        return EZB_ZCL_STATUS_SUCCESS;
    }

    if (header->cmd_id == CMD_DATA_QUERY) {
        send_all(CMD_DATA_RESPONSE, s_seq);
        return EZB_ZCL_STATUS_SUCCESS;
    }

    if (header->cmd_id == CMD_DATA_REQUEST || header->cmd_id == CMD_SEND_DATA) {
        if (!payload || payload_length < 2) {
            return EZB_ZCL_STATUS_INVALID_VALUE;
        }
        uint16_t seq = (uint16_t)(payload[0] | (payload[1] << 8));
        parse_datapoints(payload, payload_length);
        send_all(CMD_DATA_RESPONSE, seq);
        return EZB_ZCL_STATUS_SUCCESS;
    }

    return EZB_ZCL_STATUS_UNSUP_CMD;
}

static void tuya_cluster_init(uint8_t ep_id)
{
    (void)ep_id;
    ezb_zcl_custom_cluster_handlers_t handlers = {
        .cluster_id = TUYA_CLUSTER_ID,
        .cluster_role = EZB_ZCL_CLUSTER_SERVER,
        .process_cmd_cb = on_tuya_command,
    };
    ezb_zcl_custom_cluster_handlers_register(&handlers);
}

esp_err_t water_zigbee_device_create(void)
{
    ezb_af_device_desc_t dev = ezb_af_create_device_desc();

    ezb_zcl_basic_cluster_server_config_t basic_cfg = {
        .zcl_version = EZB_ZCL_BASIC_ZCL_VERSION_DEFAULT_VALUE,
        .power_source = EZB_ZCL_BASIC_POWER_SOURCE_BATTERY,
    };
    ezb_zcl_cluster_desc_t basic = ezb_zcl_basic_create_cluster_desc(&basic_cfg, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, (void *)ZIGBEE_MANUFACTURER);
    ezb_zcl_basic_cluster_desc_add_attr(basic, EZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, (void *)ZIGBEE_MODEL);

    ezb_zcl_cluster_desc_t power = ezb_zcl_power_config_create_cluster_desc(NULL, EZB_ZCL_CLUSTER_SERVER);
    ezb_zcl_power_config_cluster_desc_add_attr(power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_VOLTAGE_ID, &s_batt_volt_zcl);
    ezb_zcl_power_config_cluster_desc_add_attr(power, EZB_ZCL_ATTR_POWER_CONFIG_BATTERY_PERCENTAGE_REMAINING_ID,
                                               &s_batt_pct_zcl);

    ezb_zcl_custom_cluster_config_t tuya_cfg = {
        .cluster_id = TUYA_CLUSTER_ID,
        .init_func = tuya_cluster_init,
    };
    ezb_zcl_cluster_desc_t tuya = ezb_zcl_custom_create_cluster_desc(&tuya_cfg, EZB_ZCL_CLUSTER_SERVER);

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
    ESP_ERROR_CHECK(ezb_af_endpoint_add_cluster_desc(ep, tuya));
    ESP_ERROR_CHECK(ezb_af_device_add_endpoint_desc(dev, ep));
    ESP_ERROR_CHECK(ezb_af_device_desc_register(dev));
    return ESP_OK;
}
