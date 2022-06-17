#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "esp_system.h"
// #include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_wifi.h"
#include "esp_event_loop.h"
#include "string.h"

#include "mw_wifi.h"
// #include "mw_ble_gatt.h"
#define DEBUG 0

static const char *TAG = "wifi station";

static EventGroupHandle_t s_wifi_event_group;


bool mw_wifi_start = 0;
static int s_retry_num = 0;
wifi_config_t wifi_config;



static esp_err_t event_handler(void *ctx, system_event_t *event)

{
    if (event->event_id == SYSTEM_EVENT_STA_START) 
    {
        esp_wifi_connect();
        if(DEBUG) printf( "\nconnect to the AP ");
    } 
    else if (event->event_id == SYSTEM_EVENT_STA_DISCONNECTED) {
        if(DEBUG) printf( "\ndisconnect to the AP, reason: %d",event->event_info.disconnected.reason);
        //resson enum wifi_err_reason_t
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_err_t ret=esp_wifi_connect();
            s_retry_num++;
            if(DEBUG) printf( "\nretry to connect to the AP %s",esp_err_to_name(ret));
        } else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        if(DEBUG) printf( "\nconnect to the AP fail");
    } else if (event->event_id == SYSTEM_EVENT_STA_GOT_IP) {
        // ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        // if(DEBUG) printf( "\nGot IP: %s\n",ip4addr_ntoa(&event->event_info.got_ip.ip_info.ip));
                if(DEBUG) printf( "\nGot IP: \n");

        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
      return ESP_OK;
}
	//32
    //64
esp_err_t store_wifi_config(char *ssid,char *pwd)
{
    if(strlen(ssid)> 32)
        return ESP_ERR_INVALID_SIZE;
    if(strlen(pwd)> 64)
        return ESP_ERR_INVALID_SIZE;


    memcpy(wifi_config.sta.ssid,ssid,strlen(ssid));
    wifi_config.sta.ssid[strlen(ssid)]='\0';
    memcpy(wifi_config.sta.password,pwd,strlen(pwd));
    wifi_config.sta.password[strlen(pwd)]='\0';
    wifi_config.sta.threshold.authmode = WIFI_AUTH_WPA_PSK;

    // if(DEBUG) printf( "\nwifi_init_sta finished. ssid %s ",wifi_config.sta.ssid );

    return ESP_OK;
}

bool wifi_init_sta()
{


     // ESP_ERROR_CHECK(esp_netif_init());
    tcpip_adapter_init();

    ESP_ERROR_CHECK(esp_event_loop_init(event_handler, NULL));

    // ESP_ERROR_CHECK(esp_event_loop_create_default());
    // esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));
    esp_err_t ret;
    ret = esp_wifi_get_config(ESP_IF_WIFI_STA, &wifi_config);
    if (ret == ESP_OK)
    {
        if(DEBUG) printf( "\nWifi configuration already stored in flash partition called NVS");
        if(DEBUG) printf( "\n%s" ,wifi_config.sta.ssid);
        if(DEBUG) printf( "\n%s" ,wifi_config.sta.password);
        if(strlen((char *)(wifi_config.sta.ssid))>1)
        {
            return 1;//wifi_start_sta(); 
        }
        else    
        {
            if(DEBUG) printf("Ssid length < 1, ssid format incorrect\n");
            return 0;
        }
    }
    else
    {
        if(DEBUG) printf( "\nWifi configuration not found in flash partition called NVS."); 
        return 0;
    }
        return 0;
        // ESP_ERROR_CHECK(esp_wifi_start());
}


bool wifi_start_sta(void)
{
    s_wifi_event_group = xEventGroupCreate();


    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA) );
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_MIN_MODEM));
    ESP_ERROR_CHECK(esp_wifi_start() );
    mw_wifi_start  = 1;
    s_retry_num = 0 ;
    if(DEBUG) printf( "\nwifi_init_sta finished. ssid %s ",wifi_config.sta.ssid );
        if(DEBUG) printf( "\nnode are connecting to ap SSID:%s password:%s\n",
                    wifi_config.sta.ssid, wifi_config.sta.password);

    /* Waiting until either the connection is established (WIFI_CONNECTED_BIT) or connection failed for the maximum
        * number of re-tries (WIFI_FAIL_BIT). The bits are set by event_handler() (see above) */
    EventBits_t bits = xEventGroupWaitBits(s_wifi_event_group,
            WIFI_CONNECTED_BIT | WIFI_FAIL_BIT,
            pdFALSE,
            pdFALSE,
            portMAX_DELAY);


    /* xEventGroupWaitBits() returns the bits before the call returned, hence we can test which event actually
        * happened. */
    if (bits & WIFI_CONNECTED_BIT) {
        if(DEBUG) printf( "\nconnected to ap SSID:%s password:%s",
                    wifi_config.sta.ssid, wifi_config.sta.password);
        return 1;
    } else if (bits & WIFI_FAIL_BIT) {
        if(DEBUG) printf( "\nFailed to connect to SSID:%s, password:%s",
                    wifi_config.sta.ssid, wifi_config.sta.password);
        return 0;
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
        return 0;
    }


    /* The event will not be processed after unregister */
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    // ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
    return 0;
}
	
esp_err_t deinit_wifi()
{
    mw_wifi_start = 0 ;
    // esp_err_t ret = esp_wifi_stop();
    // if(ret)
    //     return ret;
    // return esp_wifi_deinit();
    return esp_wifi_stop();
}

bool is_wifi_start()
{
    return mw_wifi_start;
}