#ifndef MY_WIFI_H_
#define MY_WIFI_H_

#include "esp_wifi.h"


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

extern my_csi_data_t csi_data[8][CSI_STACK_SIZE];
extern uint8_t my_mac[6];
extern uint8_t wifi_channel;
extern uint8_t wifi_bw;

void wifi_init(void);


#endif