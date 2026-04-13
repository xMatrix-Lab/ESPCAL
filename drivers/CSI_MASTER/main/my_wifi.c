#if 1


#include "my_wifi.h"
#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_mac.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "driver/gpio.h"
#include "esp_private/wifi.h"

#include "lwip/err.h"
#include "lwip/sys.h"
#include "esp_wifi.h"
#include "lwip/inet.h"
#include "lwip/sockets.h"

#include "my_spi.h"

my_csi_data_t csi_data[8][CSI_STACK_SIZE];
uint8_t wifi_channel;
uint8_t wifi_bw;
esp_netif_t *wifi_netif;

#define EXAMPLE_ESP_WIFI_SSID      "Calibration"
#define EXAMPLE_ESP_WIFI_PASS      "123456789"
#define EXAMPLE_ESP_WIFI_CHANNEL   1
#define EXAMPLE_MAX_STA_CONN       8

#if CONFIG_ESP_GTK_REKEYING_ENABLE
#define EXAMPLE_GTK_REKEY_INTERVAL CONFIG_ESP_GTK_REKEY_INTERVAL
#else
#define EXAMPLE_GTK_REKEY_INTERVAL 0
#endif

static const char *TAG = "wifi softAP";
uint8_t my_mac[6] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55};

static void wifi_event_handler(void* arg, esp_event_base_t event_base,
                                    int32_t event_id, void* event_data)
{
    if (event_id == WIFI_EVENT_AP_STACONNECTED) {
        wifi_event_ap_staconnected_t* event = (wifi_event_ap_staconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" join, AID=%d",
                 MAC2STR(event->mac), event->aid);
    } else if (event_id == WIFI_EVENT_AP_STADISCONNECTED) {
        wifi_event_ap_stadisconnected_t* event = (wifi_event_ap_stadisconnected_t*) event_data;
        ESP_LOGI(TAG, "station "MACSTR" leave, AID=%d, reason=%d",
                 MAC2STR(event->mac), event->aid, event->reason);
    }
}

void wifi_init(void)
{
    gpio_config_t TRIG_config = {
        .pin_bit_mask = (1ULL<<GPIO_TRIG),          //配置引脚
        .mode =GPIO_MODE_OUTPUT,                  //输出模式
        .pull_up_en = GPIO_PULLUP_ENABLE,         //不使能上拉
        .pull_down_en = GPIO_PULLDOWN_DISABLE,    //不使能下拉
        .intr_type = GPIO_INTR_DISABLE            //不使能引脚中断
    };
    gpio_config(&TRIG_config);
    gpio_set_level(GPIO_TRIG, 1);


    // esp_netif_config_t cfg = ESP_NETIF_DEFAULT_WIFI_AP();
    // wifi_netif = esp_netif_create_wifi(WIFI_IF_AP, &cfg);
    // wifi_netif = esp_netif_new(&cfg);
    // ESP_ERROR_CHECK(esp_netif_attach_wifi_ap(wifi_netif));
    // ESP_ERROR_CHECK(esp_wifi_set_default_wifi_ap_handlers());
    //wifi_netif = esp_netif_create_default_wifi_ap(); 

    wifi_init_config_t wifi_cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&wifi_cfg));
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        NULL));
    wifi_config_t wifi_config = {
        .ap = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .ssid_len = strlen(EXAMPLE_ESP_WIFI_SSID),
            .channel = EXAMPLE_ESP_WIFI_CHANNEL,
            .password = EXAMPLE_ESP_WIFI_PASS,
            .max_connection = EXAMPLE_MAX_STA_CONN,
#ifdef CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT
            .authmode = WIFI_AUTH_WPA3_PSK,
            .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
#else /* CONFIG_ESP_WIFI_SOFTAP_SAE_SUPPORT */
            .authmode = WIFI_AUTH_WPA2_PSK,
#endif
            .pmf_cfg = {
                .required = true,
            },
#ifdef CONFIG_ESP_WIFI_BSS_MAX_IDLE_SUPPORT
            .bss_max_idle_cfg = {
                .period = WIFI_AP_DEFAULT_MAX_IDLE_PERIOD,
                .protected_keep_alive = 1,
            },
#endif
            .gtk_rekey_interval = EXAMPLE_GTK_REKEY_INTERVAL,
        },
    };
    if (strlen(EXAMPLE_ESP_WIFI_PASS) == 0) {
        wifi_config.ap.authmode = WIFI_AUTH_OPEN;
    }

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_AP));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_AP, &wifi_config));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(WIFI_IF_AP, WIFI_BW_HT20));
    //ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_ERROR_CHECK(esp_wifi_config_11b_rate(WIFI_IF_AP, true));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_LOGI(TAG, "wifi_init_softap finished. SSID:%s password:%s channel:%d",
             EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS, EXAMPLE_ESP_WIFI_CHANNEL);

    ESP_ERROR_CHECK(esp_wifi_set_mac(ESP_IF_WIFI_AP, my_mac));
    uint8_t mac[6];
    esp_wifi_get_mac(WIFI_IF_AP, mac);
    ESP_LOGI(TAG, "MY AP MAC: %02x:%02x:%02x:%02x:%02x:%02x",
             mac[0], mac[1], mac[2], mac[3], mac[4], mac[5]);
    esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
    esp_wifi_stop();
}


#endif


