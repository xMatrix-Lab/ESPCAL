#include "my_wifi.h"
#include "my_spi.h"
#include "esp_radar.h"
#include "esp_csi_gain_ctrl.h"

static const char *TAG = "csi_slave_wifi";

/* FreeRTOS event group to signal when we are connected*/
static EventGroupHandle_t s_wifi_event_group;
static int s_retry_num = 0;

int s_count = 0;
my_csi_data_t csi_data[CSI_STACK_SIZE];
uint8_t csi_stack_pointer = 0;
uint8_t mac_filter[6] = {0x00, 0x55, 0x66, 0x77, 0x88, 0x88};

static void wifi_event_handler(void* arg, esp_event_base_t event_base, int32_t event_id, void* event_data)
{
    if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_START) {
        esp_wifi_connect();
    } 
    else if (event_base == WIFI_EVENT && event_id == WIFI_EVENT_STA_DISCONNECTED) {
        if (s_retry_num < EXAMPLE_ESP_MAXIMUM_RETRY) {
            esp_wifi_connect();
            s_retry_num++;
            ESP_LOGI(TAG, "retry to connect to the AP");
        } 
        else {
            xEventGroupSetBits(s_wifi_event_group, WIFI_FAIL_BIT);
        }
        ESP_LOGI(TAG,"connect to the AP fail");
    } 
    else if (event_base == IP_EVENT && event_id == IP_EVENT_STA_GOT_IP) {
        ip_event_got_ip_t* event = (ip_event_got_ip_t*) event_data;
        ESP_LOGI(TAG, "got ip:" IPSTR, IP2STR(&event->ip_info.ip));
        s_retry_num = 0;
        xEventGroupSetBits(s_wifi_event_group, WIFI_CONNECTED_BIT);
    }
}

void wifi_init()
{
    // esp_wifi_enable_rx_stbc(1);
    s_wifi_event_group = xEventGroupCreate();

    esp_netif_create_default_wifi_sta();

    wifi_init_config_t cfg = WIFI_INIT_CONFIG_DEFAULT();
    ESP_ERROR_CHECK(esp_wifi_init(&cfg));

    esp_event_handler_instance_t instance_any_id;
    esp_event_handler_instance_t instance_got_ip;
    ESP_ERROR_CHECK(esp_event_handler_instance_register(WIFI_EVENT,
                                                        ESP_EVENT_ANY_ID,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_any_id));
    ESP_ERROR_CHECK(esp_event_handler_instance_register(IP_EVENT,
                                                        IP_EVENT_STA_GOT_IP,
                                                        &wifi_event_handler,
                                                        NULL,
                                                        &instance_got_ip));

    wifi_config_t wifi_config = {
        .sta = {
            .ssid = EXAMPLE_ESP_WIFI_SSID,
            .password = EXAMPLE_ESP_WIFI_PASS,
            /* Setting a password implies station will connect to all security modes including WEP/WPA.
             * However these modes are deprecated and not advisable to be used. Incase your Access point
             * doesn't support WPA2, these mode can be enabled by commenting below line */
	        .threshold.authmode = ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD,
	        .sae_pwe_h2e = WPA3_SAE_PWE_BOTH,
        },
    };

    ESP_ERROR_CHECK(esp_wifi_set_mode(WIFI_MODE_STA));
    ESP_ERROR_CHECK(esp_wifi_set_config(WIFI_IF_STA, &wifi_config) );
    ESP_ERROR_CHECK(esp_wifi_set_storage(WIFI_STORAGE_RAM));
    ESP_ERROR_CHECK(esp_wifi_set_bandwidth(ESP_IF_WIFI_STA, WIFI_BW_HT20));
    ESP_ERROR_CHECK(esp_wifi_set_ps(WIFI_PS_NONE));
    
    //ESP_ERROR_CHECK(esp_wifi_set_mac(WIFI_IF_STA, CONFIG_CSI_SEND_MAC));
    ESP_ERROR_CHECK(esp_wifi_start());
    ESP_ERROR_CHECK(esp_wifi_set_channel(CONFIG_LESS_INTERFERENCE_CHANNEL, WIFI_SECOND_CHAN_NONE));
    ESP_ERROR_CHECK(esp_wifi_set_promiscuous(true));
    ESP_LOGI(TAG, "wifi_init_sta finished.");

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
        ESP_LOGI(TAG, "connected to ap SSID:%s password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else if (bits & WIFI_FAIL_BIT) {
        ESP_LOGI(TAG, "Failed to connect to SSID:%s, password:%s",
                 EXAMPLE_ESP_WIFI_SSID, EXAMPLE_ESP_WIFI_PASS);
    } else {
        ESP_LOGE(TAG, "UNEXPECTED EVENT");
    }

    /* The event will not be processed after unregister */
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(IP_EVENT, IP_EVENT_STA_GOT_IP, instance_got_ip));
    ESP_ERROR_CHECK(esp_event_handler_instance_unregister(WIFI_EVENT, ESP_EVENT_ANY_ID, instance_any_id));
    vEventGroupDelete(s_wifi_event_group);
}

static void wifi_csi_rx_cb(void *ctx, wifi_csi_info_t *info)
{
    if (!info || !info->buf) {
        ESP_LOGW(TAG, "<%s> wifi_csi_cb", esp_err_to_name(ESP_ERR_INVALID_ARG));
        return;
    }

    const wifi_pkt_rx_ctrl_t *rx_ctrl = &info->rx_ctrl;
    wifi_pkt_rx_ctrl_phy_t *phy_info = (wifi_pkt_rx_ctrl_phy_t *)info;

    printf("MAC:"MACSTR", DMAC:"MACSTR", RSSI:%d, timestamp:%d\n", MAC2STR(info->mac), MAC2STR(info->dmac), rx_ctrl->rssi, rx_ctrl->timestamp);
    if(info->mac[0]== mac_filter[0] && info->mac[1]== mac_filter[1] && info->mac[2] == mac_filter[2] && info->mac[3] == mac_filter[3] && info->mac[4] == mac_filter[4] && info->mac[5] == mac_filter[5]){
    }else{
        return;
    }

    s_count++;
    float compensate_gain = 1.0f;
    static uint8_t agc_gain = 0;
    static int8_t fft_gain = 0;
#if CONFIG_GAIN_CONTROL 
    static uint8_t agc_gain_baseline = 0;
    static int8_t fft_gain_baseline = 0;
    esp_csi_gain_ctrl_get_rx_gain(rx_ctrl, &agc_gain, &fft_gain);
    if (s_count < 100) {
        esp_csi_gain_ctrl_record_rx_gain(agc_gain, fft_gain);
    } else if (s_count == 100) {
        esp_csi_gain_ctrl_get_rx_gain_baseline(&agc_gain_baseline, &fft_gain_baseline);
        printf("agc_gain_baseline %d, fft_gain_baseline %d\n", agc_gain_baseline, fft_gain_baseline);
        // esp_csi_gain_ctrl_set_rx_force_gain(agc_gain_baseline, fft_gain_baseline);
    } else {
        ESP_ERROR_CHECK(esp_csi_gain_ctrl_get_gain_compensation(&compensate_gain, agc_gain, fft_gain));
        printf("compensate_gain %f, agc_gain %d, fft_gain %d\n", compensate_gain, agc_gain, fft_gain);
    }


#endif

    if(csi_stack_pointer==CSI_STACK_SIZE){
        return;
    }
    csi_data[csi_stack_pointer].timestamp = rx_ctrl->timestamp;
    memcpy(csi_data[csi_stack_pointer].mac, info->mac, 6);
    memcpy(csi_data[csi_stack_pointer].dmac, info->dmac, 6);
    csi_data[csi_stack_pointer].rssi = rx_ctrl->rssi;
    csi_data[csi_stack_pointer].noise_floor = rx_ctrl->noise_floor;
    csi_data[csi_stack_pointer].rate = rx_ctrl->rate;
    csi_data[csi_stack_pointer].sgi = rx_ctrl->sgi;
    csi_data[csi_stack_pointer].sig_mode = rx_ctrl->sig_mode;
    csi_data[csi_stack_pointer].mcs = rx_ctrl->mcs;
    csi_data[csi_stack_pointer].cwb = rx_ctrl->cwb;
    csi_data[csi_stack_pointer].channel = rx_ctrl->channel;
    csi_data[csi_stack_pointer].secondary_channel = rx_ctrl->secondary_channel;
    csi_data[csi_stack_pointer].rx_state = rx_ctrl->rx_state;
    csi_data[csi_stack_pointer].fft_gain = phy_info->fft_gain;
    csi_data[csi_stack_pointer].agc_gain = phy_info->agc_gain;
    csi_data[csi_stack_pointer].gain = compensate_gain;
    csi_data[csi_stack_pointer].first_word_invalid = info->first_word_invalid;
    csi_data[csi_stack_pointer].len = info->len;
    memset(csi_data[csi_stack_pointer].buf, 0, sizeof(csi_data[csi_stack_pointer].buf));
    memcpy(csi_data[csi_stack_pointer].buf, info->buf, info->len);
    csi_stack_pointer++;

#if 0
    if (!s_count) {
        ESP_LOGI(TAG, "================ CSI RECV ================");
        printf("type,id,mac,rssi,rate,noise_floor,fft_gain,agc_gain,channel,local_timestamp,sig_len,rx_state,len,first_word,data\n");
    }
    printf("CSI_DATA," MACSTR ",%d,%d,%d,%d,%d,%d,%d,%d,%d",
            MAC2STR(info->mac), rx_ctrl->rssi, rx_ctrl->rate,
            rx_ctrl->noise_floor, fft_gain_force_value, agc_gain_force_value,rx_ctrl->channel,
            rx_ctrl->timestamp, rx_ctrl->sig_len, rx_ctrl->rx_state);

    printf(",%d,%d,\"[%d", info->len, info->first_word_invalid, info->buf[0]);
    for (int i = 1; i < info->len; i++) {
        printf(",%d", info->buf[i]);
    }
    printf("]\"\n");
#endif
    
}

void csi_init()
{

    wifi_csi_config_t csi_config = {
        .lltf_en           = true,
        .htltf_en          = true,
        .stbc_htltf2_en    = false,
        .ltf_merge_en      = true,
        .channel_filter_en = false,
        .manu_scale        = false,
        .shift             = 0,
    };

    static wifi_ap_record_t s_ap_info = {0};
    //ESP_ERROR_CHECK(esp_wifi_sta_get_ap_info(&s_ap_info));
    ESP_ERROR_CHECK(esp_wifi_set_csi_config(&csi_config));
    ESP_ERROR_CHECK(esp_wifi_set_csi_rx_cb(wifi_csi_rx_cb, s_ap_info.bssid));
    ESP_ERROR_CHECK(esp_wifi_set_csi(true));
}

esp_err_t wifi_ping_router_start()
{
    static esp_ping_handle_t ping_handle = NULL;

    esp_ping_config_t ping_config = ESP_PING_DEFAULT_CONFIG();
    ping_config.count             = 0;
    ping_config.interval_ms       = 100000 / CONFIG_SEND_FREQUENCY;
    ping_config.task_stack_size   = 3072;
    ping_config.data_size         = 1;

    esp_netif_ip_info_t local_ip;
    esp_netif_get_ip_info(esp_netif_get_handle_from_ifkey("WIFI_STA_DEF"), &local_ip);
    ESP_LOGI(TAG, "got ip:" IPSTR ", gw: " IPSTR, IP2STR(&local_ip.ip), IP2STR(&local_ip.gw));
    ping_config.target_addr.u_addr.ip4.addr = ip4_addr_get_u32(&local_ip.gw);
    ping_config.target_addr.type = ESP_IPADDR_TYPE_V4;

    esp_ping_callbacks_t cbs = { 0 };
    esp_ping_new_session(&ping_config, &cbs, &ping_handle);
    esp_ping_start(ping_handle);

    return ESP_OK;
}

esp_err_t wifi_scan_start()
{
    static esp_netif_t *s_sta_netif = NULL;
    
    // 获取STA网络接口
    s_sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (s_sta_netif == NULL) {
        ESP_LOGE(TAG, "Failed to get STA netif");
        return ESP_FAIL;
    }

    // 配置WiFi扫描参数
    wifi_scan_config_t scan_config = {
        .ssid = NULL,           // 扫描所有SSID
        .bssid = NULL,          // 扫描所有BSSID
        .channel = 0,           // 扫描所有信道
        .show_hidden = true,    // 显示隐藏网络
        .scan_type = WIFI_SCAN_TYPE_ACTIVE, // 主动扫描
        .scan_time = {          // 扫描时间配置
            .active = {
                .min = 100,     // 最小活跃扫描时间(ms)
                .max = 300      // 最大活跃扫描时间(ms)
            }
        }
    };

    ESP_LOGI(TAG, "Starting WiFi scan...");
    
    // 启动扫描
    esp_err_t ret = esp_wifi_scan_start(&scan_config, false);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start WiFi scan: %s", esp_err_to_name(ret));
        return ret;
    }

    ESP_LOGI(TAG, "WiFi scan started successfully");
    return ESP_OK;
}




