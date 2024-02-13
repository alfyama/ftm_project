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

ESP_EVENT_DEFINE_BASE(END_SCAN_OR_FTM_EVENT);
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

#if STA_MODE
static const int FTM_REPORT_BIT = BIT0;
static const int FTM_FAILURE_BIT = BIT1;

#endif /* if STA_MODE */

#define MAX_CONNECT_RETRY_ATTEMPTS 5
#define ETH_ALEN 6
#define MAX_APS 4

static wifi_ftm_report_entry_t *s_ftm_report;
static uint8_t s_ftm_report_num_entries;
static uint32_t s_rtt_est, s_dist_est;
static bool s_ap_started;
static uint8_t s_ap_channel;
static uint8_t s_ap_bssid[ETH_ALEN];

wifi_ap_record_t *g_ap_list_buffer;
wifi_ap_record_t aps[MAX_APS];


const wifi_bandwidth_t CURRENT_BW = WIFI_BW_HT20;



int ftm(wifi_ap_record_t *ap_record);

static void wifi_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {
    if (event_id == WIFI_EVENT_STA_CONNECTED) {
        wifi_event_sta_connected_t *event = (wifi_event_sta_connected_t *)event_data;

        ESP_LOGI(TAG_STA, "Connected to %s (BSSID: "MACSTR", Channel: %d)", event->ssid,
                 MAC2STR(event->bssid), event->channel);

        memcpy(s_ap_bssid, event->bssid, ETH_ALEN);
        s_ap_channel = event->channel;
        xEventGroupClearBits(s_wifi_event_group, DISCONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, CONNECTED_BIT);

        /* if(ESP_OK != ftm()){ */
        /*     ESP_LOGE(TAG_STA, "FTM failed!"); */
        /* } */

    } else if (event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_reconnect && ++s_retry_num < MAX_CONNECT_RETRY_ATTEMPTS) {
            ESP_LOGI(TAG_STA, "sta disconnect, retry attempt %d...", s_retry_num);
            esp_wifi_connect();
        } else {
            ESP_LOGI(TAG_STA, "sta disconnected");
        }
        xEventGroupClearBits(s_wifi_event_group, CONNECTED_BIT);
        xEventGroupSetBits(s_wifi_event_group, DISCONNECTED_BIT);
    } else if (event_id == WIFI_EVENT_AP_START) {
        s_ap_started = true;
    } else if (event_id == WIFI_EVENT_AP_STOP) {
        s_ap_started = false;
    }
}

static void ftm_event_handler(void *arg, esp_event_base_t event_base, int32_t event_id, void *event_data) {



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

    esp_event_handler_instance_t instance_wifi_id;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        WIFI_EVENT_STA_CONNECTED,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_wifi_id));

    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, 
                                                          WIFI_EVENT_STA_DISCONNECTED,
                                                          &wifi_event_handler,
                                                          NULL,
                                                          &instance_wifi_id));
    
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT, 
                                                        WIFI_EVENT_FTM_REPORT,
                                                        &ftm_event_handler,
                                                        NULL,
                                                        NULL));

    #if !STA_MODE
        // ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_AP, CURRENT_BW));
    #endif

    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_NULL));
    ESP_ERROR_CHECK(esp_wifi_start());
    
    initialized = true;
}

static esp_err_t perform_scan(const char *ssid, uint8_t num_anchors)
{

    ESP_LOGI(TAG_STA, "Performing AP scan.");
    wifi_scan_config_t scan_config = { 0 };
    scan_config.ssid = (uint8_t *) ssid;

    if (ESP_OK != esp_wifi_scan_start(&scan_config, true)) {
        ESP_LOGI(TAG_STA, "Scan failed.");
        return ESP_FAIL;
    }

    uint16_t g_scan_ap_num;
    esp_wifi_scan_get_ap_num(&g_scan_ap_num);

    if(g_scan_ap_num == 0) {
        ESP_LOGI(TAG_STA, "No APs found.");
        return ESP_FAIL;
    }

    g_ap_list_buffer = malloc(g_scan_ap_num * sizeof(wifi_ap_record_t));
    if (g_ap_list_buffer == NULL) {
        ESP_LOGE(TAG_STA, "Failed to malloc buffer to print scan results");
        return ESP_FAIL;
    }

    if (esp_wifi_scan_get_ap_records(&g_scan_ap_num, (wifi_ap_record_t *)g_ap_list_buffer) == ESP_OK) {
        for (uint8_t i = 0; i < g_scan_ap_num; i++) {
            ESP_LOGI(TAG_STA, "[%s][rssi=%d]""%s", g_ap_list_buffer[i].ssid, g_ap_list_buffer[i].rssi,
                        g_ap_list_buffer[i].ftm_responder ? "[FTM Responder]" : "");

            // TODO Just add access points to the buffer that 
            // we have configured, ignore all others 
            if(num_anchors < MAX_APS && g_ap_list_buffer[i].ftm_responder){

                int comp = strcmp((char *)g_ap_list_buffer[i].ssid, WIFI_SSID);
                if(comp == 0){
                    aps[num_anchors] = g_ap_list_buffer[i];
                    num_anchors++;
                }
            }
            
        }
    }

    free(g_ap_list_buffer);
    ESP_LOGI(TAG_STA, "sta scan done");
    return ESP_OK;
}

static int process_aps(uint8_t num_anchors, uint8_t current_anchor) {

    current_anchor += 1;
    if(current_anchor >= num_anchors) {
        current_anchor = 0;
    }

    return ftm(&aps[current_anchor]);
}

int ftm(wifi_ap_record_t *ap_record)
{
    /* int bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0); */
    /* if (!(bits & CONNECTED_BIT)){ */
    /*     ESP_LOGE(TAG_STA, "Not connected. FTM only supported for connected AP."); */
    /*     return ESP_FAIL; */
    /* } */

    ESP_LOGI(TAG_STA, "FTM start...");
        
    wifi_ftm_initiator_cfg_t ftm_cfg = {
        .frm_count = 32,
        .burst_period = 2,
        .channel = s_ap_channel,
    };

    if(ap_record){
        memcpy(ftm_cfg.resp_mac, ap_record->bssid, ETH_ALEN);
    }
    
    else {
        ESP_LOGE(TAG_STA, "NOT AP RECORD");
        ESP_ERROR_CHECK(esp_event_post( END_SCAN_OR_FTM_EVENT, 0, NULL, 0,pdMS_TO_TICKS(100)));
    }

    esp_err_t err = esp_wifi_ftm_initiate_session(&ftm_cfg);
    if (ESP_OK != err) {
        ESP_LOGE(TAG_STA, "Failed to start FTM session with %i", err);
        ESP_ERROR_CHECK(esp_event_post( END_SCAN_OR_FTM_EVENT, 0, NULL, 0,pdMS_TO_TICKS(100)));
        return ESP_FAIL;
    }



    /* ESP_LOGI(TAG_STA, "Requesting FTM session with Frm Count - %d, Burst Period - %dmSec (0: No Preference)", */
    /*                 ftm_cfg.frm_count, ftm_cfg.burst_period*100); */
    /*  */
    /* if(ESP_OK != esp_wifi_ftm_initiate_session(&ftm_cfg)){ */
    /*     ESP_LOGE(TAG_STA, "Failed to start FTM session"); */
    /*     return ESP_FAIL; */
    /* } */

    /* bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT | FTM_FAILURE_BIT, */
    /*                                         pdTRUE, pdFALSE, portMAX_DELAY); */
    /* if (bits & FTM_REPORT_BIT) */
    /* { */
    /*     ESP_LOGI(TAG_STA, */
    /*         "Estimated RTT - %" PRId32 " nSec,\ */
    /*         Estimated Distance - %" PRId32 ".%02" PRId32 " meters", */
    /*         s_rtt_est, s_dist_est / 100, s_dist_est % 100); */
    /*     return ESP_OK; */
    /* } else { */
    /*     return ESP_FAIL; */
    /* } */

    return ESP_OK;
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
    
    const TickType_t xTicksToWaitFTM = 500 / portTICK_PERIOD_MS;
    uint8_t num_anchors, current_anchor;
    num_anchors = 0;
    current_anchor = -1;


    int bits = xEventGroupWaitBits(s_wifi_event_group, CONNECTED_BIT, 0, 1, 0);
    wifi_config_t wifi_config = {
        .sta.ssid = WIFI_SSID,
        .sta.password = WIFI_PASSWORD,
    };

    s_reconnect = true;
    s_retry_num = 0;

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_STA, &wifi_config));
    
    // TODO ssid should be the same as APs ??
    if(perform_scan(WIFI_SSID, num_anchors))    
        ESP_LOGW(TAG_STA, "Scan failed. Connecting anyway.");
    ESP_ERROR_CHECK(esp_wifi_connect());

    for(;;){

        process_aps(num_anchors, current_anchor);
        EventBits_t bits = xEventGroupWaitBits(s_ftm_event_group, FTM_REPORT_BIT|FTM_FAILURE_BIT, pdFALSE, pdFALSE, xTicksToWaitFTM);

        if(bits & FTM_REPORT_BIT) {
            // TODO ftm report 
            
            vTaskDelay(20);
        }
        else if (bits & FTM_FAILURE_BIT) {
              xEventGroupClearBits(s_ftm_event_group, FTM_FAILURE_BIT);

              esp_restart();
        }

        else {
            vTaskDelay(20);
        }
        xEventGroupClearBits(s_ftm_event_group, FTM_REPORT_BIT);
    }


#else 
    /** FTM Responder (AP mode) **/     

    uint8_t mac[6];
    char mac_add[17];

    wifi_bandwidth_t bw;

    ESP_LOGI(TAG_AP,"Starting AP...");

    ESP_ERROR_CHECK(esp_base_mac_addr_get(&mac[0]));
    ESP_LOGI(TAG_AP,"MAC address: %02X:%02X:%02X:%02X:%02X:%02X", mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);

    wifi_config_t ap_config = {

        .ap.ssid = WIFI_SSID,
        .ap.ssid_len = 0,
        .ap.password = WIFI_PASSWORD,
        .ap.channel = AP_CHANNEL,
        .ap.max_connection = 4,
        .ap.authmode = WIFI_AUTH_WPA2_PSK,
        .ap.ftm_responder = true
    };

    /* strlcpy((char *)ap_config.ap.ssid, mac_add, sizeof(ap_config.ap.ssid)); */
    /* strlcpy((char *)ap_config.ap.password, WIFI_PASSWORD, sizeof(ap_config.ap.password)); */
   

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(ESP_IF_WIFI_AP, &ap_config));

    /* while (1)  */
    /* { */
    /*     vTaskDelay(1000); */
    /* } */

    /* ESP_ERROR_CHECK(esp_wifi_get_bandwith(ESP_IF_WIFI_AP, &bw)); */
    

#endif 

    


}
