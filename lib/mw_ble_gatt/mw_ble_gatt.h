#ifdef __cplusplus
extern "C" {
#endif
#ifndef mw_ble_gatt_h
#define mw_ble_gatt_h

#include "esp_bt.h"

#include "esp_gap_ble_api.h"
#include "esp_gatts_api.h"
#include "esp_bt_defs.h"
#include "esp_bt_main.h"
#include "esp_gatt_common_api.h"
#include <string.h>
#include "driver/gpio.h"

///Declare the static function

#define GATTS_SERVICE_UUID_TEST_A   0x00FF
#define GATTS_CHAR_UUID_TEST_A      0xFF01
#define GATTS_DESCR_UUID_TEST_A     0x3333
#define GATTS_NUM_HANDLE_TEST_A     4


#define TEST_DEVICE_NAME            "ESP_MW_CONFIG_WIFI"
#define TEST_MANUFACTURER_DATA_LEN  17

#define GATTS_DEMO_CHAR_VAL_LEN_MAX 0x40

#define PREPARE_BUF_MAX_SIZE 1024


#define PROFILE_NUM 1
#define PROFILE_A_APP_ID 0

struct gatts_profile_inst {
    esp_gatts_cb_t gatts_cb;
    uint16_t gatts_if;
    uint16_t app_id;
    uint16_t conn_id;
    uint16_t service_handle;
    esp_gatt_srvc_id_t service_id;
    uint16_t char_handle;
    esp_bt_uuid_t char_uuid;
    esp_gatt_perm_t perm;
    esp_gatt_char_prop_t property;
    uint16_t descr_handle;
    esp_bt_uuid_t descr_uuid;
};


#define GPIO_OUTPUT_IO_0    2//12
// #define GPIO_OUTPUT_IO_1    2
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0)/*||(1ULL<<GPIO_OUTPUT_IO_1)*/)

#define GPIO_INPUT_IO_0     0//14
#define GPIO_INPUT_PIN_SEL  (1ULL<<GPIO_INPUT_IO_0) 
#define ESP_INTR_FLAG_DEFAULT 10
//hold to change mode 
#define HOLD_PERIOD 1000
#define HOLD_SWITCH_APP 5000
// typedef struct {
//     uint8_t                 *prepare_buf;
//     int                     prepare_len;
// } prepare_type_env_t;

// static prepare_type_env_t a_prepare_write_env;

/**
 * 
 * @brief initial ble to config wifi
 * @brief initial wifi
 * 
 */

esp_err_t mw_init_ble_config_wifi();
/**
 * 
 * @brief initial wifi
 * 
 */
esp_err_t mw_init_wifi();
/**
 * 
 * @brief switch to another app in default partition
 * 
 */
void mw_switch_app();

#endif

#ifdef __cplusplus
}
#endif