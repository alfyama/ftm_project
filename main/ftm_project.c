#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "nvs_flash.h"
#include "freertos/FreeRTOS.h"
#include "freertos/event_groups.h"
#include "esp_event.h"
#include "esp_log.h"
#include "esp_err.h"
#include "esp_wifi.h"
#include "esp_mac.h"

#define WIFI_SSID      CONFIG_WIFI_SSID
#define WIFI_PASSWORD  CONFIG_WIFI_PASSWORD
#define AP_CHANNEL     CONFIG_AP_CHANNEL

#define STA_MODE       CONFIG_STA_MODE


static bool s_reconnect = true;
static int s_retry_num = 5;

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_lit *disconnect;
    struct arg_end *end;
} wifi_sta_args_t;

typedef struct {
    struct arg_str *ssid;
    struct arg_str *password;
    struct arg_int *channel;
    struct arg_int *bandwidth;
    struct arg_end *end;
} wifi_ap_args_t;

typedef struct {
    /* FTM Initiator */
    struct arg_lit *initiator;
    struct arg_int *frm_count;
    struct arg_int *burst_period;
    struct arg_str *ssid;
    /* FTM Responder */
    struct arg_lit *responder;
    struct arg_lit *enable;
    struct arg_lit *disable;
    struct arg_int *offset;
    struct arg_end *end;
} wifi_ftm_args_t;

static const char *TAG_STA = "STATION";
static const char *TAG_AP = "AP";


static wifi_sta_args_t sta_args;
static wifi_ap_args_t ap_args;
static wifi_ftm_args_t ftm_args;

// RTOS events
static EventGroupHandle_t s_wifi_event_group;
static EventGroupHandle_t s_ftm_event_group;


// Flags 
static const int CONNECTED_BIT = BIT0;
static const int DISCONNECTED_BIT = BIT1;

static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;

#define MAX_CONNECT_RETRY_ATTEMPTS 5
#define ETH_ALEN 6

static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est, s_dist_est;
static bool s_ap_started;
static uint8_t s_ap_channel;
static uint8_t s_ap_bssid[ETH_ALEN];


static void event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: "MACSTR", Channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);

        memcpy(s_ap_bssid, event->bssid, ETH_ALEN);
        s_ap_channel = event->channel;
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_reconnect && ++s_retry_num < MAX_CONNECT_RETRY_ATTEMPTS) {
            ESP_LOGI(TAG_STA, "sta disconnect, retry attempt %d...", s_retry_num);
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG_STA, "sta disconnected");
        }
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_FTM_REPORT) {
        wifi_event_ftm_report_t *event = (wifi_event_ftm_report_t *) event_data;

        s_rtt_est = event->rtt_est;
        s_dist_est = event->dist_est;
        s_ftm_report = event->ftm_report_data;
        s_ftm_report_num_entries = event->ftm_report_num_entries;
        if (event->status == FTM_STATUS_SUCCESS) {
            xEventGroupSetBits(s_ftm_event_group, FTM_REPORT_BIT);
        } else {
            ESP_LOGI(TAG_STA, "FTM procedure with Peer("MACSTR") failed! (Status - %d)",
                     MAC2STR(event->peer_mac), event->status);
            xEventGroupSetBits(s_ftm_event_group, FTM_FAILURE_BIT);
        }
    } else if (event_id == WIFI_EVENT_AP_START) {
        s_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        s_ap_started = false;
    }


}

static void init_wifi(void) {

    static bool initialized = false;    

    if (initialized) {
        return;
    }

    ESP_ERROR_CHECK(esp_netif_init());
    s_wifi_event_group = xEventGroupCreate();
    s_ftm_event_group = xEventGroupCreate();
    ESP_ERROR_CHECK( esp_event_loop_create_default() );
    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &event_handler,
                                                        NULL,
                                                        &instance_any_id));
    
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    initialized = true;
}

void app_main(void)
{
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK(ret);

    init_wifi();



#if STA_MODE 
    /** FTM Initiator (Station mode) **/
    int bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
    wifi_config_t wifi_config = {
        .sta.ssid = WIFI_SSID,
        .sta.password = WIFI_PASSWORD,

    };

    s_reconnect = true;
    s_retry_num = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_connect());



#else 
    /** FTM Responder (AP mode) **/     

    ESP_LOGI(TAG_AP,"Starting AP...");

    wifi_config_t ap_config = {

        .ap.ssid = WIFI_SSID,
        .ap.password = WIFI_PASSWORD,
        .ap.channel = AP_CHANNEL,
        .ap.max_connection = 4,
        .ap.authmode = WIFI_AUTH_WPA2_PSK,
        .ap.ftm_responder = true
    };
   

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));

    // Here https://github.com/espressif/esp-idf/blob/a5b261f699808efdacd287adbded5b718dffd14e/examples/wifi/ftm/README.md it 
    // says that 20MHz gives more accurate results.
    esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, WIFI_BW_HT20);



#endif 

}
