#ifndef MY_WIFI_H_
#define MY_WIFI_H_

#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "sdkconfig.h"

#include "esp_flash.h"
#include "esp_system.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_wifi.h"
#include "esp_netif.h"
#include "driver/gpio.h"
#include "esp_mac.h"

#include "ping/ping_sock.h"
#include "ping/ping.h"
#include "lwip/err.h"
#include "lwip/netdb.h"
#include "lwip/sys.h"


#define EXAMPLE_ESP_WIFI_SSID      "mywifi"
#define EXAMPLE_ESP_WIFI_PASS      "123456789"
#define EXAMPLE_ESP_MAXIMUM_RETRY  0
#define ESP_WIFI_SCAN_AUTH_MODE_THRESHOLD WIFI_AUTH_OPEN 
#define DATA_TABLE_SIZE            100

/* The event group allows multiple bits for each event, but we only care about two events:
 * - we are connected to the AP with an IP
 * - we failed to connect after the maximum amount of retries */
#define WIFI_CONNECTED_BIT BIT0
#define WIFI_FAIL_BIT      BIT1

#define CONFIG_LESS_INTERFERENCE_CHANNEL  1  //第一信道
#define CONFIG_FORCE_GAIN                 0  //是否强制增益，开启后会固定增益值为前100次的平均值，可能会导致wifi不稳定，甚至无法连接，建议先关闭观察增益变化情况
#define CONFIG_GAIN_CONTROL               1  //
#define CONFIG_IDF_TARGET_ESP32S3         1
#define CONFIG_SEND_FREQUENCY        100

typedef struct
{
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
    unsigned : 32; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 ||CONFIG_IDF_TARGET_ESP32C6
    unsigned : 16; /**< reserved */
    unsigned fft_gain : 8;
    unsigned agc_gain : 8;
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
#if CONFIG_IDF_TARGET_ESP32S2
      signed : 8;  /**< reserved */
    unsigned : 24; /**< reserved */
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
    unsigned : 32; /**< reserved */
#endif
    unsigned : 32; /**< reserved */
} wifi_pkt_rx_ctrl_phy_t;

typedef struct __attribute__((packed)){
    uint32_t timestamp;         /**< timestamp. The local time when this packet is received. It is precise only if modem sleep or light sleep is not enabled. unit: microsecond */
    uint8_t mac[6];             /**< source MAC address of the CSI data */
    uint8_t dmac[6];            /**< destination MAC address of the CSI data */
    int8_t rssi;                /**< Received Signal Strength Indicator(RSSI) of packet. unit: dBm */
    int8_t noise_floor;         /**< noise floor of Radio Frequency Module(RF). unit: dBm*/
    uint8_t rate;               /**< PHY rate encoding of the packet. Only valid for non HT(11bg) packet */
    uint8_t sgi;                /**< Short Guide Interval(SGI). 0: Long GI; 1: Short GI */
    uint8_t sig_mode;           /**< Protocol of the received packet, 0: non HT(11bg) packet; 1: HT(11n) packet; 3: VHT(11ac) packet */
    uint8_t mcs;                /**< Modulation Coding Scheme. If is HT(11n) packet, shows the modulation, range from 0 to 76(MSC0 ~ MCS76) */
    uint8_t cwb;                /**< Channel Bandwidth of the packet. 0: 20MHz; 1: 40MHz */
    uint8_t channel;            /**< primary channel on which this packet is received */
    uint8_t secondary_channel;  /**< secondary channel on which this packet is received. 0: none; 1: above; 2: below */
    uint8_t rx_state;           /**< state of the packet. 0: no error; others: error numbers which are not public */
    uint8_t fft_gain;
    uint8_t agc_gain;
    float gain;                 /**< user */
    uint8_t first_word_invalid; /**< first four bytes of the CSI data is invalid or not, true indicates the first four bytes is invalid due to hardware limitation */
    uint16_t len;               /**< valid length of CSI data */
    int8_t buf[384];            /**< valid buffer of CSI data */
} my_csi_data_t;

#define CSI_STACK_SIZE  1

extern my_csi_data_t csi_data[CSI_STACK_SIZE];
extern uint8_t csi_stack_pointer;
extern uint8_t mac_filter[6];

void wifi_init();
void csi_init();
esp_err_t wifi_ping_router_start();
esp_err_t wifi_scan_start();


#endif