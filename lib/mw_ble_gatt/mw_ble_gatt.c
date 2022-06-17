#include "esp_system.h"
#include "freertos/FreeRTOS.h"

// #include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "mw_ble_gatt.h"
#include "mw_wifi.h"
#include "esp_ota_ops.h"
#include "esp_partition.h"
// #include "mw_gpio.h"
#include "freertos/timers.h"

#define GATTS_TAG "GATTS"
static const char *GPIO_TAG = "GPIO";

// ESP_EVENT_DEFINE_BASE(GPIO_EVENT);

static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param);
// static void task_iteration_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data);
void stop_blink();
void start_blink();
void light_adv_on();
void light_adv_off();
///gpio///
int64_t timestamp = 0 ;
bool event_op = 0;
bool state_wifi = 0 ;
TimerHandle_t timer_instance;
TimerHandle_t counter;
const bool debug_i = 0 ;
///gpio///


static uint8_t char1_str[] = {0x11,0x22,0x33};
static esp_gatt_char_prop_t a_property = 0;
bool is_advertise = 0;
bool is_connect = 0;

static esp_attr_value_t gatts_demo_char1_val =
{
    .attr_max_len = GATTS_DEMO_CHAR_VAL_LEN_MAX,
    .attr_len     = sizeof(char1_str),
    .attr_value   = char1_str,
};

static uint8_t adv_config_done = 0;
#define adv_config_flag      (1 << 0)
#define scan_rsp_config_flag (1 << 1)

#ifdef CONFIG_SET_RAW_ADV_DATA

static uint8_t raw_adv_data[] = {
        0x02, 0x01, 0x06,
        0x02, 0x0a, 0xeb, 0x03, 0x03, 0xab, 0xcd
};
static uint8_t raw_scan_rsp_data[] = {
        0x0f, 0x09, 0x45, 0x53, 0x50, 0x5f, 0x47, 0x41, 0x54, 0x54, 0x53, 0x5f, 0x44,
        0x45, 0x4d, 0x4f
};
#else

static uint8_t adv_service_uuid128[32] = {
    /* LSB <--------------------------------------------------------------------------------> MSB */
    //first uuid, 16bit, [12],[13] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xEE, 0x00, 0x00, 0x00,
    //second uuid, 32bit, [12], [13], [14], [15] is the value
    0xfb, 0x34, 0x9b, 0x5f, 0x80, 0x00, 0x00, 0x80, 0x00, 0x10, 0x00, 0x00, 0xFF, 0x00, 0x00, 0x00,
};

// The length of adv data must be less than 31 bytes
//static uint8_t test_manufacturer[TEST_MANUFACTURER_DATA_LEN] =  {0x12, 0x23, 0x45, 0x56};
//adv data
static esp_ble_adv_data_t adv_data = {
    .set_scan_rsp = false,
    .include_name = true,
    .include_txpower = false,
    .min_interval = 0x0006, //slave connection min interval, Time = min_interval * 1.25 msec
    .max_interval = 0x0010, //slave connection max interval, Time = max_interval * 1.25 msec
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};
// scan response data
static esp_ble_adv_data_t scan_rsp_data = {
    .set_scan_rsp = true,
    .include_name = true,
    .include_txpower = true,
    //.min_interval = 0x0006,
    //.max_interval = 0x0010,
    .appearance = 0x00,
    .manufacturer_len = 0, //TEST_MANUFACTURER_DATA_LEN,
    .p_manufacturer_data =  NULL, //&test_manufacturer[0],
    .service_data_len = 0,
    .p_service_data = NULL,
    .service_uuid_len = sizeof(adv_service_uuid128),
    .p_service_uuid = adv_service_uuid128,
    .flag = (ESP_BLE_ADV_FLAG_GEN_DISC | ESP_BLE_ADV_FLAG_BREDR_NOT_SPT),
};

#endif /* CONFIG_SET_RAW_ADV_DATA */


static esp_ble_adv_params_t adv_params = {
    .adv_int_min        = 0x20,
    .adv_int_max        = 0x40,
    .adv_type           = ADV_TYPE_IND,
    .own_addr_type      = BLE_ADDR_TYPE_PUBLIC,
    //.peer_addr            =
    //.peer_addr_type       =
    .channel_map        = ADV_CHNL_ALL,
    .adv_filter_policy = ADV_FILTER_ALLOW_SCAN_ANY_CON_ANY,
};


/* One gatt-based profile one app_id and one gatts_if, this array will store the gatts_if returned by ESP_GATTS_REG_EVT */
static struct gatts_profile_inst gl_profile_tab[PROFILE_NUM] = {
    [PROFILE_A_APP_ID] = {
        .gatts_cb = gatts_profile_a_event_handler,
        .gatts_if = ESP_GATT_IF_NONE,       /* Not get the gatt_if, so initial is ESP_GATT_IF_NONE */
    },
};

void switch_app(void *pvParameters) {
    const esp_partition_t *update_partition = NULL;
    esp_err_t err ;
    if(debug_i) printf("init _ boot \n");
    update_partition = esp_ota_get_next_update_partition(NULL);
    if(debug_i) printf("Writing to partition subtype %d at offset 0x%x\n",
            update_partition->subtype, update_partition->address);
    assert(update_partition != NULL);
    err = esp_ota_set_boot_partition(update_partition);
    if (err != ESP_OK) {
        ESP_LOGE("TAG", "esp_ota_set_boot_partition failed (%s)!", esp_err_to_name(err));
        // task_fatal_error();
    }
    if(debug_i) printf("fPrepare to restart system!\n");
    vTaskDelay(500 / portTICK_RATE_MS);
    esp_restart();
    vTaskDelete(NULL);
}
void counter_cb(void *timer)
{
    // printf("test \n");
    light_adv_off();
    vTaskDelay(500 / portTICK_RATE_MS);
    light_adv_on();

}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    if(gpio_get_level(gpio_num) == 0)
    {
        timestamp = esp_timer_get_time();
        xTimerStart(counter, 0);
        stop_blink();
        light_adv_off();
    }
    else
    {
        xTimerStop(counter,0);
        // xTimerDelete(counter,0);
        // light_adv_off();

        // TIMER_EVENT evt;
        int16_t dif= esp_timer_get_time()/1000 - timestamp/1000;
        if(dif > HOLD_SWITCH_APP)
        {
            // evt = TASK_TIMER_OVER;
            xTaskCreatePinnedToCore(switch_app, "switch_app", 4096, NULL, 3, NULL, 0);

        }
        else if(dif > HOLD_PERIOD)
        {
            // evt = TASK_TIMER_OVER;
            esp_err_t ret = ESP_OK;
            // stop_blink();]
            if(!is_advertise)
            {
                ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
                light_adv_on();
                is_advertise=1;
            }
            else
            {
                ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
                light_adv_off();
                is_advertise=0;

            }

            if(debug_i) ets_printf( "upper %ld is_advertise %d \n" ,dif,is_advertise);
            if(ret)
            {
                if(debug_i) ets_printf( "err becouse %s",esp_err_to_name(ret) );
            }
        }
        else
        {
            if(is_advertise)
                light_adv_on();
            
        }

    }

}




void blink_cb(void *timer)
{
    state_wifi = !state_wifi;
    // ESP_LOGI(GPIO_TAG," state %d\n",state_wifi);
    gpio_set_level(GPIO_OUTPUT_IO_0, state_wifi);
}


void light_adv_on()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, false);
}

void light_adv_off()
{
    gpio_set_level(GPIO_OUTPUT_IO_0, true);
}


void setup_pin()
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = 0;
    //disable pull-up mode
    io_conf.pull_up_en = 0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    light_adv_off();

    //interrupt of rising edge
    io_conf.intr_type = GPIO_INTR_POSEDGE;
    //bit mask of the pins, use GPIO4/5 here
    io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    //set as input mode
    io_conf.mode = GPIO_MODE_INPUT;
    //enable pull-up mode
    io_conf.pull_up_en = 1;
    gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
    gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

    // install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    // //remove isr handler for gpio number.
    // gpio_isr_handler_remove(GPIO_INPUT_IO_0);
    // //hook isr handler for specific gpio pin again
    // gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

    ESP_LOGI(GPIO_TAG,"Minimum free heap size: %d bytes\n", esp_get_minimum_free_heap_size());

}

void start_blink()
{
    // timer_instance = xTimerCreate("blink_cb", 1000 / portTICK_RATE_MS,
    //                 true, NULL, blink_cb);
    xTimerStart(timer_instance, 0);
    
}

void stop_blink()
{
    light_adv_off();
    xTimerStop(timer_instance,0);
    // xTimerDelete(timer_instance,0);
}



static void gap_event_handler(esp_gap_ble_cb_event_t event, esp_ble_gap_cb_param_t *param)
{
    // ESP_LOGI(GPIO_TAG,"gap_event_handler %d \n " ,event);
    switch (event) {
#ifdef CONFIG_SET_RAW_ADV_DATA
    case ESP_GAP_BLE_ADV_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_RAW_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        if (adv_config_done==0){
            esp_ble_gap_start_advertising(&adv_params);
        }
        break;
#else
    case ESP_GAP_BLE_ADV_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~adv_config_flag);
        // if (adv_config_done == 0){
        //     esp_ble_gap_start_advertising(&adv_params);
        // }
        break;
    case ESP_GAP_BLE_SCAN_RSP_DATA_SET_COMPLETE_EVT:
        adv_config_done &= (~scan_rsp_config_flag);
        // if (adv_config_done == 0){
        //     esp_ble_gap_start_advertising(&adv_params);
        // }
        break;
#endif
    case ESP_GAP_BLE_ADV_START_COMPLETE_EVT:
        //advertising start complete event to indicate advertising start successfully or failed
        if (param->adv_start_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising start failed\n");
        }else {
            if(debug_i) printf("\nstart adv successfully\n");
        }
        light_adv_on();
        is_advertise = 1;
        break;
    case ESP_GAP_BLE_ADV_STOP_COMPLETE_EVT:
        if (param->adv_stop_cmpl.status != ESP_BT_STATUS_SUCCESS) {
            ESP_LOGE(GATTS_TAG, "Advertising stop failed\n");
        } else {
           if(debug_i)  printf("\nStop adv successfully\n");
        }
        is_advertise = 0;
        light_adv_off();

        break;
    case ESP_GAP_BLE_UPDATE_CONN_PARAMS_EVT:
         if(debug_i) printf("\nupdate connection params status = %d, min_int = %d, max_int = %d,conn_int = %d,latency = %d, timeout = %d",
                  param->update_conn_params.status,
                  param->update_conn_params.min_int,
                  param->update_conn_params.max_int,
                  param->update_conn_params.conn_int,
                  param->update_conn_params.latency,
                  param->update_conn_params.timeout);
        break;
    default:
        break;
    }
}



static void gatts_profile_a_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param) {
    switch (event) {
    case ESP_GATTS_REG_EVT:
        if(debug_i) printf("\nREGISTER_APP_EVT, status %d, app_id %d\n", param->reg.status, param->reg.app_id);
        gl_profile_tab[PROFILE_A_APP_ID].service_id.is_primary = true;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.inst_id = 0x00;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].service_id.id.uuid.uuid.uuid16 = GATTS_SERVICE_UUID_TEST_A;

        esp_err_t set_dev_name_ret = esp_ble_gap_set_device_name(TEST_DEVICE_NAME);
        if (set_dev_name_ret){
            ESP_LOGE(GATTS_TAG, "set device name failed, error code = %x", set_dev_name_ret);
        }
#ifdef CONFIG_SET_RAW_ADV_DATA
        esp_err_t raw_adv_ret = esp_ble_gap_config_adv_data_raw(raw_adv_data, sizeof(raw_adv_data));
        if (raw_adv_ret){
            ESP_LOGE(GATTS_TAG, "config raw adv data failed, error code = %x ", raw_adv_ret);
        }
        adv_config_done |= adv_config_flag;
        esp_err_t raw_scan_ret = esp_ble_gap_config_scan_rsp_data_raw(raw_scan_rsp_data, sizeof(raw_scan_rsp_data));
        if (raw_scan_ret){
            ESP_LOGE(GATTS_TAG, "config raw scan rsp data failed, error code = %x", raw_scan_ret);
        }
        adv_config_done |= scan_rsp_config_flag;
#else
        //config adv data
        esp_err_t ret = esp_ble_gap_config_adv_data(&adv_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config adv data failed, error code = %x", ret);
        }
        adv_config_done |= adv_config_flag;
        //config scan response data
        ret = esp_ble_gap_config_adv_data(&scan_rsp_data);
        if (ret){
            ESP_LOGE(GATTS_TAG, "config scan response data failed, error code = %x", ret);
        }
        adv_config_done |= scan_rsp_config_flag;

#endif
        esp_ble_gatts_create_service(gatts_if, &gl_profile_tab[PROFILE_A_APP_ID].service_id, GATTS_NUM_HANDLE_TEST_A);

        break;
    case ESP_GATTS_READ_EVT: {
        if(debug_i) printf("\nGATT_READ_EVT, conn_id %d, trans_id %d, handle %d\n", param->read.conn_id, param->read.trans_id, param->read.handle);
        esp_gatt_rsp_t rsp;
        memset(&rsp, 0, sizeof(esp_gatt_rsp_t));
        rsp.attr_value.handle = param->read.handle;
        rsp.attr_value.len = 4;
        rsp.attr_value.value[0] = 0xde;
        rsp.attr_value.value[1] = 0xed;
        rsp.attr_value.value[2] = 0xbe;
        rsp.attr_value.value[3] = 0xef;
        esp_ble_gatts_send_response(gatts_if, param->read.conn_id, param->read.trans_id,
                                    ESP_GATT_OK, &rsp);
        break;
    }
    case ESP_GATTS_WRITE_EVT: {
        if(debug_i) printf("\nGATT_WRITE_EVT, conn_id %d, trans_id %d, handle %d", param->write.conn_id, param->write.trans_id, param->write.handle);
        if (!param->write.is_prep){
            if(debug_i) printf("\nGATT_WRITE_EVT, value len %d, value :", param->write.len);
            esp_log_buffer_hex(GATTS_TAG, param->write.value, param->write.len);



            uint16_t  data_len = (param)->write.len;
            char data[data_len];
            memcpy(data ,(char *) (param)->write.value , data_len);
            data[data_len] = '\0';
            int len_ssid =(strchr( data, ',' )- &(data[0]) ) ;

            if(len_ssid > 300 || len_ssid<0)
                break;

            char ssid[32];
            memcpy(ssid,data,len_ssid);

            ssid[len_ssid] = '\0';
            char *pwd = &(data[0])+ len_ssid +1 ; 

            if(debug_i) printf("\nstore ssid: %s pwd: %s",ssid, pwd);

            ESP_ERROR_CHECK(store_wifi_config(ssid, pwd));
            // char ssid[32];
            // memcpy(ssid,data,strlen(data));
            // ssid[strlen(data)] = '\0';
            // char *pwd = &(data[0])+ strlen(data) +1 ; 


        }

        break;
    }
    case ESP_GATTS_EXEC_WRITE_EVT:
        // printf("\nESP_GATTS_EXEC_WRITE_EVT");
        // esp_ble_gatts_send_response(gatts_if, param->write.conn_id, param->write.trans_id, ESP_GATT_OK, NULL);
        break;
    case ESP_GATTS_MTU_EVT:
        if(debug_i) printf("\nESP_GATTS_MTU_EVT, MTU %d", param->mtu.mtu);
        break;
    case ESP_GATTS_UNREG_EVT:
        break;
    case ESP_GATTS_CREATE_EVT:
        if(debug_i) printf("\nCREATE_SERVICE_EVT, status %d,  service_handle %d\n", param->create.status, param->create.service_handle);
        gl_profile_tab[PROFILE_A_APP_ID].service_handle = param->create.service_handle;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.len = ESP_UUID_LEN_16;
        gl_profile_tab[PROFILE_A_APP_ID].char_uuid.uuid.uuid16 = GATTS_CHAR_UUID_TEST_A;

        esp_ble_gatts_start_service(gl_profile_tab[PROFILE_A_APP_ID].service_handle);
        a_property =  ESP_GATT_CHAR_PROP_BIT_WRITE_NR | ESP_GATT_CHAR_PROP_BIT_READ /*| ESP_GATT_CHAR_PROP_BIT_NOTIFY*/;
        esp_err_t add_char_ret = esp_ble_gatts_add_char(gl_profile_tab[PROFILE_A_APP_ID].service_handle, &gl_profile_tab[PROFILE_A_APP_ID].char_uuid,
                                                        ESP_GATT_PERM_READ | ESP_GATT_PERM_WRITE,
                                                        a_property,
                                                        &gatts_demo_char1_val, NULL);
        if (add_char_ret){
            ESP_LOGE(GATTS_TAG, "add char failed, error code =%x",add_char_ret);
        }

        break;
    case ESP_GATTS_ADD_INCL_SRVC_EVT:
        break;
    case ESP_GATTS_ADD_CHAR_EVT: {
        break;
    }
    case ESP_GATTS_ADD_CHAR_DESCR_EVT:
        gl_profile_tab[PROFILE_A_APP_ID].descr_handle = param->add_char_descr.attr_handle;
        if(debug_i) printf("\nADD_DESCR_EVT, status %d, attr_handle %d, service_handle %d\n",
                 param->add_char_descr.status, param->add_char_descr.attr_handle, param->add_char_descr.service_handle);
        break;
    case ESP_GATTS_DELETE_EVT:
        break;
    case ESP_GATTS_START_EVT:
        if(debug_i) printf("\nSERVICE_START_EVT, status %d, service_handle %d\n",
                 param->start.status, param->start.service_handle);
        break;
    case ESP_GATTS_STOP_EVT:
        break;
    case ESP_GATTS_CONNECT_EVT: {
        if(is_wifi_start())
            ESP_ERROR_CHECK(deinit_wifi());
        is_advertise = 0;
        is_connect = 1 ;
        light_adv_off();

        esp_ble_conn_update_params_t conn_params = {0};
        memcpy(conn_params.bda, param->connect.remote_bda, sizeof(esp_bd_addr_t));
        /* For the IOS system, please reference the apple official documents about the ble connection parameters restrictions. */
        conn_params.latency = 0;
        conn_params.max_int = 0x20;    // max_int = 0x20*1.25ms = 40ms
        conn_params.min_int = 0x10;    // min_int = 0x10*1.25ms = 20ms
        conn_params.timeout = 400;    // timeout = 400*10ms = 4000ms
        if(debug_i) printf("\nESP_GATTS_CONNECT_EVT, conn_id %d, remote %02x:%02x:%02x:%02x:%02x:%02x:",
                 param->connect.conn_id,
                 param->connect.remote_bda[0], param->connect.remote_bda[1], param->connect.remote_bda[2],
                 param->connect.remote_bda[3], param->connect.remote_bda[4], param->connect.remote_bda[5]);
        gl_profile_tab[PROFILE_A_APP_ID].conn_id = param->connect.conn_id;
        //start sent the update connection parameters to the peer device.
        esp_ble_gap_update_conn_params(&conn_params);
        break;
    }
    case ESP_GATTS_DISCONNECT_EVT:
        if(debug_i) printf("\nESP_GATTS_DISCONNECT_EVT, disconnect reason 0x%x", param->disconnect.reason);
        // esp_ble_gap_start_advertising(&adv_params);
        is_connect = 0 ;
        start_blink();

        bool is_connect_wifi = wifi_start_sta();
        stop_blink();
        if(!is_connect_wifi)
            esp_ble_gap_start_advertising(&adv_params);
        break;
    case ESP_GATTS_CONF_EVT:
        if(debug_i) printf("\nESP_GATTS_CONF_EVT, status %d ", param->conf.status);
        if (param->conf.status != ESP_GATT_OK){
            esp_log_buffer_hex(GATTS_TAG, param->conf.value, param->conf.len);
        }
        break;
    case ESP_GATTS_OPEN_EVT:
    case ESP_GATTS_CANCEL_OPEN_EVT:
    case ESP_GATTS_CLOSE_EVT:
    case ESP_GATTS_LISTEN_EVT:
    case ESP_GATTS_CONGEST_EVT:
    default:
        break;
    }
}


static void gatts_event_handler(esp_gatts_cb_event_t event, esp_gatt_if_t gatts_if, esp_ble_gatts_cb_param_t *param)
{
    /* If event is register event, store the gatts_if for each profile */
    // ESP_LOGI(GPIO_TAG,"gatts_event_handler %d \n " ,event);
    if (event == ESP_GATTS_REG_EVT) {
        if (param->reg.status == ESP_GATT_OK) {
            gl_profile_tab[param->reg.app_id].gatts_if = gatts_if;
        } else {
            if(debug_i) printf("\nReg app failed, app_id %04x, status %d\n",
                    param->reg.app_id,
                    param->reg.status);
            // ESP_GATT_DUP_REG = 0x90        
            return;
        }
    /* If the gatts_if equal to profile A, call profile A cb handler,
     * so here call each profile's callback */
    }
    do {
        int idx;
        for (idx = 0; idx < PROFILE_NUM; idx++) {
            if (gatts_if == ESP_GATT_IF_NONE || /* ESP_GATT_IF_NONE, not specify a certain gatt_if, need to call every profile cb function */
                    gatts_if == gl_profile_tab[idx].gatts_if) {
                if (gl_profile_tab[idx].gatts_cb) {
                    gl_profile_tab[idx].gatts_cb(event, gatts_if, param);
                }
            }
        }
    } while (0);


}

// static void task_iteration_handler(void *handler_args, esp_event_base_t base, int32_t id, void *event_data)
// {
//   // Two types of data can be passed in to the event handler: the handler specific data and the event-specific data.
//   //
//   // The handler specific data (handler_args) is a pointer to the original data, therefore, the user should ensure that
//   // the memory location it points to is still valid when the handler executes.
//   //
//   // The event-specific data (event_data) is a pointer to a deep copy of the original data, and is managed automatically.
//   TIMER_EVENT iteration = *((TIMER_EVENT *)event_data);
// //   printf( "handling %s:%s from %s, iteration %d\n", base, "ccn_EVENT", "loop", id);
//   switch (iteration)
//   {
//   case TASK_TIMER_OVER:
//         if(is_connect)
//             break;
//         if (adv_config_done == 0){
//             if(!is_advertise)
//                 ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
//             else
//                 ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
//         }
//         break;
//   default:

//         break;
//   }
// }
esp_err_t mw_init_wifi()
{
    esp_err_t ret =ESP_OK;

    counter = xTimerCreate("counter_cb", 1000 / portTICK_RATE_MS,
                    true, NULL, counter_cb);
    timer_instance = xTimerCreate("blink_cb", 1000 / portTICK_RATE_MS,
                    true, NULL, blink_cb);
    // setup_pin();
    bool has_ssid =wifi_init_sta();
    if(has_ssid)
    {
        ESP_LOGI(GPIO_TAG,"has ssid \n");
        start_blink();
        bool is_connect_wifi =wifi_start_sta();
        stop_blink();
        // if(!is_connect_wifi)
        // {
        //     xTaskCreatePinnedToCore(switch_app, "switch_app", 4096, NULL, 3, NULL, 0);
        // }
    }
    else
        ESP_LOGI(GPIO_TAG,"hasn't ssid \n");
    
    return ret;

}

esp_err_t mw_init_ble_config_wifi()
{
    esp_err_t ret =ESP_OK;
    if(debug_i) printf("start timer\n");

    counter = xTimerCreate("counter_cb", 1000 / portTICK_RATE_MS,
                    true, NULL, counter_cb);
    timer_instance = xTimerCreate("blink_cb", 1000 / portTICK_RATE_MS,
                    true, NULL, blink_cb);
    if(debug_i) printf("end timer\n");
    // if(!was_init)
    // printf("esp_bt_controller_mem_release = %s\n", esp_err_to_name(ret));
    esp_bt_controller_status_t ble_state = esp_bt_controller_get_status();
// ESP_BT_CONTROLLER_STATUS_IDLE 
// ESP_BT_CONTROLLER_STATUS_INITED
// ESP_BT_CONTROLLER_STATUS_ENABLED
// ESP_BT_CONTROLLER_STATUS_NUM
    if(ble_state == ESP_BT_CONTROLLER_STATUS_IDLE)
    {
        ESP_ERROR_CHECK(esp_bt_controller_mem_release(ESP_BT_MODE_CLASSIC_BT));
        esp_bt_controller_config_t bt_cfg = BT_CONTROLLER_INIT_CONFIG_DEFAULT();
        ret = esp_bt_controller_init(&bt_cfg);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "%s initialize controller failed: %s\n", __func__, esp_err_to_name(ret));
            return ret;
        }
    }
    if(ble_state == ESP_BT_CONTROLLER_STATUS_IDLE ||ble_state == ESP_BT_CONTROLLER_STATUS_INITED )
    {
        ret = esp_bt_controller_enable(ESP_BT_MODE_BLE);
        if (ret) {
            ESP_LOGE(GATTS_TAG, "%s enable controller failed: %s\n", __func__, esp_err_to_name(ret));
            return ret;
        }
    }
// ESP_BLUEDROID_STATUS_UNINITIALIZED = 0
// Bluetooth not initialized
// ESP_BLUEDROID_STATUS_INITIALIZED
// Bluetooth initialized but not enabled
// ESP_BLUEDROID_STATUS_ENABLED
    esp_bluedroid_status_t bld_state = esp_bluedroid_get_status();
    if(bld_state == ESP_BLUEDROID_STATUS_UNINITIALIZED)
    {
        ret = esp_bluedroid_init();
        if (ret) {
            ESP_LOGE(GATTS_TAG, "%s init bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
            return ret;
        }
    }
    if(bld_state != ESP_BLUEDROID_STATUS_ENABLED)
    {
        ret = esp_bluedroid_enable();
        if (ret) {
            ESP_LOGE(GATTS_TAG, "%s enable bluetooth failed: %s\n", __func__, esp_err_to_name(ret));
            return ret;
        }
    }
    else
        goto UNINIT_BLE;
    

    ret = esp_ble_gatts_register_callback(gatts_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts register error, error code = %x", ret);
        return ret;
    }
    
    ret = esp_ble_gap_register_callback(gap_event_handler);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gap register error, error code = %x", ret);
        return ret;
    }

    ret = esp_ble_gatts_app_register(PROFILE_A_APP_ID);
    if (ret){
        ESP_LOGE(GATTS_TAG, "gatts app register error, error code = %x", ret);
        return ret;
    }

    esp_err_t local_mtu_ret = esp_ble_gatt_set_local_mtu(512-3);
    if (local_mtu_ret){
        ESP_LOGE(GATTS_TAG, "set local  MTU failed, error code = %x", local_mtu_ret);
    }

    UNINIT_BLE:
    {
        setup_pin();
        bool has_ssid =wifi_init_sta();
        if(has_ssid)
        {
            ESP_LOGI(GPIO_TAG,"has ssid \n");
            start_blink();
            bool is_connect_wifi =wifi_start_sta();
            stop_blink();
            if(is_connect_wifi)
            {
                ESP_ERROR_CHECK(esp_ble_gap_stop_advertising());
                light_adv_off();

            }
            else {
                ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
                light_adv_on();

            }
        }
        else {
                ESP_ERROR_CHECK(esp_ble_gap_start_advertising(&adv_params));
                light_adv_on();

            }
    }


    return ret;
}


void mw_switch_app()
{
    xTaskCreatePinnedToCore(switch_app, "switch_app", 4096, NULL, 3, NULL, 0);
    return;
}