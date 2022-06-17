#include "mw_ble_gatt.h"
#include "nvs_flash.h"
#include "esp_err.h"

void app_main()
{
	esp_err_t ret = nvs_flash_init();
	if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
  	    ESP_ERROR_CHECK(nvs_flash_erase());
	    ESP_ERROR_CHECK(nvs_flash_init());
    }
    mw_init_ble_config_wifi();
}
