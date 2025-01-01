/*
 * SPDX-FileCopyrightText: 2021-2024 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: CC0-1.0
 *
 * Zigbee HA_on_off_light Example
 *
 * This example code is in the Public Domain (or CC0 licensed, at your option.)
 *
 * Unless required by applicable law or agreed to in writing, this
 * software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
 * CONDITIONS OF ANY KIND, either express or implied.
 */

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_check.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "nvs_flash.h"
#include "ha/esp_zigbee_ha_standard.h"
#include "esp_zb_light.h"
#include "blink_led.h"

static TaskHandle_t blink_led_task_handle = NULL;
static TaskHandle_t check_incoming_messages_task_handle = NULL;

#define OCCUPANCY_SENSOR_ENDPOINT 1
#define ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID 0x0107

#if !defined ZB_ED_ROLE
#error Define ZB_ED_ROLE in idf.py menuconfig to compile light (End Device) source code.
#endif

char manufname[] = {9, 'E', 's', 'p', 'r', 'e', 's', 's', 'i', 'f'};
char modelid[] = {14, 'E', 'S', 'P', '3', '2', 'H', '2', '.', 'S', 'e', 'n', 's', 'o', 'r'};

static int64_t last_message_time = 0;

static const char *TAG = "ESP_ZB_TRAFFIC_LIGHT";
/********************* Define functions **************************/
static void bdb_start_top_level_commissioning_cb(uint8_t mode_mask)
{
    ESP_ERROR_CHECK(esp_zb_bdb_start_top_level_commissioning(mode_mask));
}

static void check_incoming_messages_task(void *arg)
{
    while (true) {
        int64_t now = esp_timer_get_time();

        // 10s timeout
        if ((now - last_message_time) > 10000000) {
            ESP_LOGW(TAG, "No occupancy reports for >10s, retry network steering...");
            esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

            light_driver_reset();
            xTaskCreate(blink_led_init, "blink_led_task", 4096, NULL, tskIDLE_PRIORITY, &blink_led_task_handle);

            vTaskDelete(check_incoming_messages_task_handle);
        }
        vTaskDelay(pdMS_TO_TICKS(5000)); // 5s
    }
}

void esp_zb_app_signal_handler(esp_zb_app_signal_t *signal_struct)
{
    uint32_t *p_sg_p       = signal_struct->p_app_signal;
    esp_err_t err_status = signal_struct->esp_err_status;
    esp_zb_app_signal_type_t sig_type = *p_sg_p;
    switch (sig_type) {
    case ESP_ZB_ZDO_SIGNAL_SKIP_STARTUP:
        ESP_LOGI(TAG, "Zigbee stack initialized");
        esp_zb_secur_network_min_join_lqi_set(0);
        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_INITIALIZATION);
        break;
    case ESP_ZB_BDB_SIGNAL_DEVICE_FIRST_START:
    case ESP_ZB_BDB_SIGNAL_DEVICE_REBOOT:
        if (err_status == ESP_OK) {
            ESP_LOGI(TAG, "Device started up in %s factory-reset mode", esp_zb_bdb_is_factory_new() ? "" : "non");
            if (esp_zb_bdb_is_factory_new()) {
                ESP_LOGI(TAG, "Start network steering");
            } else {
                ESP_LOGI(TAG, "Device rebooted, attempting reconnection...");
            }
        } else {
            /* commissioning failed */
            ESP_LOGW(TAG, "Failed to initialize Zigbee stack (status: %s)", esp_err_to_name(err_status));
        }

        esp_zb_bdb_start_top_level_commissioning(ESP_ZB_BDB_MODE_NETWORK_STEERING);

        break;
    case ESP_ZB_BDB_SIGNAL_STEERING:
        if (err_status == ESP_OK) {
            esp_zb_ieee_addr_t extended_pan_id;
            esp_zb_get_extended_pan_id(extended_pan_id);
            ESP_LOGI(TAG, "Joined network successfully (Extended PAN ID: %02x:%02x:%02x:%02x:%02x:%02x:%02x:%02x, PAN ID: 0x%04hx, Channel:%d, Short Address: 0x%04hx)",
                     extended_pan_id[7], extended_pan_id[6], extended_pan_id[5], extended_pan_id[4],
                     extended_pan_id[3], extended_pan_id[2], extended_pan_id[1], extended_pan_id[0],
                     esp_zb_get_pan_id(), esp_zb_get_current_channel(), esp_zb_get_short_address());

            if (blink_led_task_handle != NULL) {
                vTaskDelete(blink_led_task_handle);
                blink_led_task_handle = NULL;
                light_driver_reset();

                xTaskCreate(check_incoming_messages_task, "check_incoming_messages", 4096, NULL, tskIDLE_PRIORITY, &check_incoming_messages_task_handle);
            }
        } else {
            ESP_LOGI(TAG, "Network steering was not successful (status: %s)", esp_err_to_name(err_status));
            esp_zb_scheduler_alarm((esp_zb_callback_t)bdb_start_top_level_commissioning_cb, ESP_ZB_BDB_MODE_NETWORK_STEERING, 1000);
        }
        break;
    default:
        ESP_LOGI(TAG, "ZDO signal: %s (0x%x), status: %s", esp_zb_zdo_signal_to_string(sig_type), sig_type,
                 esp_err_to_name(err_status));
        break;
    }
}

static esp_err_t zb_attribute_reporting_handler(esp_zb_zcl_report_attr_message_t *message)
{
    esp_err_t ret = ESP_OK;

    ESP_RETURN_ON_FALSE(message, ESP_FAIL, TAG, "Empty message");
    ESP_RETURN_ON_FALSE(message->status == ESP_ZB_ZCL_STATUS_SUCCESS, ESP_ERR_INVALID_ARG, TAG, "Received message: error status(%d)",
                        message->status);
    ESP_LOGD(TAG, "Received report from address(0x%x) src endpoint(%d) to dst endpoint(%d) cluster(0x%x), attribute(0x%x) datatype(0x%x) datasize(0x%x)",
             message->src_address.u.short_addr, message->src_endpoint,
             message->dst_endpoint, message->cluster, message->attribute.id, message->attribute.data.type, message->attribute.data.size);

    if (message->dst_endpoint == OCCUPANCY_SENSOR_ENDPOINT
        && message->cluster == ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING
        && message->attribute.id == ESP_ZB_ZCL_ATTR_OCCUPANCY_SENSING_OCCUPANCY_ID
        && message->attribute.data.type == ESP_ZB_ZCL_ATTR_TYPE_8BITMAP) {
            bool occupied = (bool)(*(uint8_t *)message->attribute.data.value);

            ESP_LOGI(TAG, "Got occupancy attribute value: 0x%x", *(uint8_t *)message->attribute.data.value);
            light_driver_set_free(!occupied);

            last_message_time = esp_timer_get_time();
    }

    return ret;
}

static esp_err_t zb_action_handler(esp_zb_core_action_callback_id_t callback_id, const void *message)
{
    esp_err_t ret = ESP_OK;
    switch (callback_id) {
    case ESP_ZB_CORE_REPORT_ATTR_CB_ID:
        ret = zb_attribute_reporting_handler((esp_zb_zcl_report_attr_message_t *)message);
        break;
    default:
        ESP_LOGW(TAG, "Receive Zigbee action(0x%x) callback", callback_id);
        break;
    }
    return ret;
}

static void esp_zb_task(void *pvParameters)
{
    /* initialize Zigbee stack */
    esp_zb_cfg_t zb_nwk_cfg = ESP_ZB_ZED_CONFIG();
    esp_zb_init(&zb_nwk_cfg);

    esp_zb_attribute_list_t *esp_zb_basic_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_BASIC);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MANUFACTURER_NAME_ID, &manufname[0]);
    esp_zb_basic_cluster_add_attr(esp_zb_basic_cluster, ESP_ZB_ZCL_ATTR_BASIC_MODEL_IDENTIFIER_ID, &modelid[0]);

    esp_zb_attribute_list_t *esp_zb_occupancy_cluster = esp_zb_zcl_attr_list_create(ESP_ZB_ZCL_CLUSTER_ID_OCCUPANCY_SENSING);

    esp_zb_cluster_list_t *esp_zb_cluster_list = esp_zb_zcl_cluster_list_create();
    esp_zb_cluster_list_add_basic_cluster(esp_zb_cluster_list, esp_zb_basic_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);
    esp_zb_cluster_list_add_occupancy_sensing_cluster(esp_zb_cluster_list, esp_zb_occupancy_cluster, ESP_ZB_ZCL_CLUSTER_CLIENT_ROLE);

    esp_zb_ep_list_t *esp_zb_ep_list = esp_zb_ep_list_create();
    esp_zb_endpoint_config_t endpoint_config = {
        .endpoint = OCCUPANCY_SENSOR_ENDPOINT,
        .app_profile_id = ESP_ZB_AF_HA_PROFILE_ID,
        .app_device_id = ESP_ZB_HA_OCCUPANCY_SENSOR_DEVICE_ID,
    };
    esp_zb_ep_list_add_ep(esp_zb_ep_list, esp_zb_cluster_list, endpoint_config);

    esp_zb_device_register(esp_zb_ep_list);

    esp_zb_core_action_handler_register(zb_action_handler);
    esp_zb_set_primary_network_channel_set(ESP_ZB_PRIMARY_CHANNEL_MASK);
    ESP_ERROR_CHECK(esp_zb_start(false));
    esp_zb_stack_main_loop();
}

void app_main(void)
{
    esp_zb_platform_config_t config = {
        .radio_config = ESP_ZB_DEFAULT_RADIO_CONFIG(),
        .host_config = ESP_ZB_DEFAULT_HOST_CONFIG(),
    };
    ESP_ERROR_CHECK(nvs_flash_init());
    ESP_ERROR_CHECK(esp_zb_platform_config(&config));
    light_driver_init();

    xTaskCreate(blink_led_init, "blink_led_task", 4096, NULL, tskIDLE_PRIORITY, &blink_led_task_handle);

    xTaskCreate(esp_zb_task, "Zigbee_main", 8192, NULL, 5, NULL);
}
