/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include <math.h>
#include <sys/param.h>
#include "esp_heap_caps.h"
#include "esp_system.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_wifi.h"
#include "esp_log.h"
#include "esp_now.h"
#ifdef __cplusplus
extern "C"
{
#endif

#define ADDR_IS_FULL(addr)        (((addr)[0] & (addr)[1] & (addr)[2] & (addr)[3] & (addr)[4] & (addr)[5]) == 0xFF)
#define ADDR_IS_EMPTY(addr)         (((addr)[0] | (addr)[1] | (addr)[2] | (addr)[3] | (addr)[4] | (addr)[5]) == 0x0)

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define WIFI_CSI_PHY_GAIN_ENABLE          1
#endif
#ifndef WIFI_BW_HT40
#define WIFI_BW_HT40 WIFI_BW40
#endif
#ifndef WIFI_BW_HT20
#define WIFI_BW_HT20 WIFI_BW20
#endif
// #if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 2, 1)
// #define WIFI_CSI_SEND_NULL_DATA_ENABLE    1
// #endif

static inline uint32_t esp_radar_malloc_caps(void)
{
#ifdef CONFIG_SPIRAM
    static int8_t s_psram_ok = -1;
    if (s_psram_ok < 0) {
        s_psram_ok = (heap_caps_get_total_size(MALLOC_CAP_SPIRAM) > 0) ? 1 : 0;
    }
    return (s_psram_ok == 1) ? MALLOC_CAP_SPIRAM : MALLOC_CAP_DEFAULT;
#endif
    return MALLOC_CAP_DEFAULT;
}

#define RADAR_MALLOC_RETRY(size) ({                                                                                                        \
    void *ptr = NULL;                                                                                                                  \
    while (size > 0 && !(ptr = heap_caps_malloc(size, esp_radar_malloc_caps()))) {                                                     \
        ESP_LOGW("esp_radar", "<ESP_ERR_NO_MEM> %s:%d Realloc size: %d, ptr: %p, heap free: %d", __func__, __LINE__, (int)size, ptr, esp_get_free_heap_size()); \
        vTaskDelay(pdMS_TO_TICKS(100));                                                                                                \
    }                                                                                                                                  \
    ptr;                                                                                                                               \
})

#define RADAR_FREE(ptr) do { \
    if (ptr) { \
        free(ptr); \
        ptr = NULL; \
    } \
} while(0)

/**
 * @brief WiFi RX format enumeration
 */
typedef enum {
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    RX_FORMAT_11B      = 0,           /**< The reception frame is an 11b MPDU */
    RX_FORMAT_11G      = 1,           /**< The reception frame is an 11g MPDU */
    RX_FORMAT_11A = RX_FORMAT_11G,   /**< The reception frame is an 11a MPDU */
    RX_FORMAT_HT       = 2,           /**< The reception frame is an HT MPDU */
    RX_FORMAT_VHT      = 3,           /**< The reception frame is a VHT MPDU */
    RX_FORMAT_HE_SU    = 4,           /**< The reception frame is a HE SU MPDU */
    RX_FORMAT_HE_MU    = 5,           /**< The reception frame is a HE MU MPDU */
#elif CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    RX_FORMAT_NON_HT   = 0,           /**< The reception frame is a non-HT MPDU (11b/g) */
    RX_FORMAT_HT       = 1,           /**< The reception frame is an HT MPDU */
    RX_FORMAT_VHT      = 2,           /**< The reception frame is a VHT MPDU */
#endif
} wifi_rx_format_t;

/**
 * @brief WiFi signal mode enumeration
 */
typedef enum {
    WIFI_SIGNAL_MODE_NON_HT,  /**< 11b/g */
    WIFI_SIGNAL_MODE_HT,      /**< 11n */
    WIFI_SIGNAL_MODE_HE,      /**< 11ax */
} wifi_signal_mode_t;

/**
 * @brief WiFi channel bandwidth enumeration
 */
typedef enum {
    WIFI_CHANNEL_BANDWIDTH_20MHZ = 0,  /**< 20MHz */
    WIFI_CHANNEL_BANDWIDTH_40MHZ = 1,  /**< 40MHz */
} wifi_channel_bandwidth_t;

/**
 * @brief Wi-Fi radar information structure
 */
typedef struct {
    float waveform_jitter;  /**< Jitter of the radar waveform, used for detecting human movement */
    float waveform_wander;  /**< Wander of the radar waveform, used for detecting human presence */
} wifi_radar_info_t;

/**
 * @brief Radar RX control information structure
 */
typedef struct {
    int8_t rssi;                    /**< Received Signal Strength Indicator */
    uint8_t rate;                   /**< Data rate */
    wifi_rx_format_t rx_format;     /**< Reception frame format */
    wifi_signal_mode_t signal_mode; /**< Signal mode */
    uint8_t mcs;                    /**< Modulation and Coding Scheme */
    uint8_t cwb;                    /**< Channel Width Bandwidth */
    uint8_t stbc;                   /**< Space Time Block Code */
    uint8_t agc_gain;               /**< Automatic Gain Control gain */
    int8_t fft_gain;                /**< FFT gain */
    uint32_t timestamp;             /**< Timestamp */
    int8_t noise_floor;             /**< Noise floor */
    uint8_t channel;                /**< Primary channel */
    uint8_t secondary_channel;      /**< Secondary channel */
} esp_radar_rx_ctrl_info_t;

/**
 * @brief Sub-carrier range structure
 */
typedef struct {
    uint16_t start;  /**< Start index of the sub-carrier range */
    uint16_t stop;   /**< Stop index of the sub-carrier range */
} sub_carrier_range_t;

/**
 * @brief CSI sub-carrier table structure
 */
typedef struct {
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    wifi_second_chan_t second;                    /**< Secondary channel */
#endif
    wifi_signal_mode_t signal_mode;                /**< Signal mode */
    wifi_channel_bandwidth_t channel_bandwidth;    /**< Channel bandwidth */
    bool stbc;                                     /**< Space Time Block Code (STBC). 0: non-STBC packet; 1: STBC packet */
    size_t total_bytes;                            /**< Total bytes of CSI data */
    size_t valid_bytes;                            /**< Valid bytes of CSI data after filtering */
    union {
        struct {
            uint16_t lltf_bytes;                   /**< Legacy Long Training Field bytes */
            uint16_t ht_ltf_bytes;                 /**< HT Long Training Field bytes */
            uint16_t stbc_ht_ltf_bytes;           /**< STBC HT Long Training Field bytes */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
            uint16_t he_ltf_bytes;                 /**< HE Long Training Field bytes */
            uint16_t stbc_he_ltf_bytes;           /**< STBC HE Long Training Field bytes */
#endif
        };
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
        uint16_t sub_carrier_bytes[5];             /**< Sub-carrier bytes array (5 elements) */
#else
        uint16_t sub_carrier_bytes[3];             /**< Sub-carrier bytes array (3 elements) */
#endif
    };
    sub_carrier_range_t lltf[2];                  /**< Legacy Long Training Field sub-carrier ranges */
    sub_carrier_range_t ht_ltf[4];                /**< HT Long Training Field sub-carrier ranges */
    sub_carrier_range_t stbc_ht_ltf[4];           /**< STBC HT Long Training Field sub-carrier ranges */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    sub_carrier_range_t he_ltf[4];                /**< HE Long Training Field sub-carrier ranges */
    sub_carrier_range_t stbc_he_ltf[4];           /**< STBC HE Long Training Field sub-carrier ranges */
#endif
} csi_sub_carrier_table_t;

extern const csi_sub_carrier_table_t sub_carrier_table[];
extern const size_t sub_carrier_table_size;

/**
 * @brief WiFi CSI data type enumeration
 */
typedef enum {
    WIFI_CSI_DATA_TYPE_INT8  = 0,   /**< CSI samples are stored as 8-bit signed integers */
    WIFI_CSI_DATA_TYPE_INT16 = 1,   /**< CSI samples are stored as 16-bit signed integers */
} wifi_csi_data_type_t;

/**
 * @brief Channel state information (CSI) filtered data structure
 */
typedef struct {
    wifi_csi_info_t *info;                      /**< Pointer to CSI information */
    esp_radar_rx_ctrl_info_t rx_ctrl_info;      /**< Received packet radio metadata header of the CSI data */
    uint32_t seq_id;                            /**< Auto-increment sequence ID for frame-level tracking */
    uint8_t mac[6];                             /**< Source MAC address of the CSI data */
    uint8_t dmac[6];                           /**< Destination MAC address of the CSI data */
    float rx_gain_compensation;                 /**< RX gain compensation */
    wifi_csi_data_type_t data_type;             /**< Actual data width used by valid_data */
    uint16_t raw_len;                           /**< Length of the raw CSI data */
    int8_t *raw_data;                           /**< Pointer to the raw CSI data, unfiltered and contains invalid subcarriers */
    uint16_t valid_len;                         /**< Length of the CSI data after filtering */
    uint16_t valid_lltf_len;                    /**< Length of the LL-LTF data after filtering */
    uint16_t valid_ht_ltf_len;                  /**< Length of the HT-LTF data after filtering */
    uint16_t valid_stbc_ht_ltf_len;             /**< Length of the STBC-HT-LTF data after filtering */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    uint16_t valid_he_ltf_len;                   /**< Length of the HE-LTF data after filtering */
    uint16_t valid_stbc_he_ltf_len;              /**< Length of the STBC-HE-LTF data after filtering */
#endif
    union {
        int8_t  valid_data[0];                   /**< CSI data interpreted as 8-bit samples */
        int16_t valid_data_i16[0];               /**< CSI data interpreted as 16-bit samples */
    };
} wifi_csi_filtered_info_t;

/**
 * @brief The RX callback function of Wi-Fi radar data.
 *
 *        Each time Wi-Fi radar data analysis completes, the callback function will be called.
 *
 * @param ctx  Context argument, passed to esp_radar_init() or esp_radar_change_config() when registering callback function.
 * @param info Wi-Fi radar data received. The memory that it points to will be deallocated after callback function returns.
 */
typedef void (*wifi_radar_cb_t)(void *ctx, const wifi_radar_info_t *info);

/**
 * @brief The RX callback function of Wi-Fi CSI data.
 *
 * @param ctx  Context argument, passed to esp_radar_csi_init() when registering callback function.
 * @param info Pointer to the filtered CSI data. The data is preprocessed according to WiFi packet type
 *             (e.g., lltf, htltf1, htltf2, heltf, etc.). The memory that it points to will be deallocated
 *             after callback function returns.
 */
typedef void (*wifi_csi_filtered_cb_t)(void *ctx, const wifi_csi_filtered_info_t *info);

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
/**
 * @brief CSI HE STBC mode enumeration
 */
typedef enum {
    CSI_HE_STBC_MODE_LTF1 = 0,      /**< Acquire the complete HE-LTF1 */
    CSI_HE_STBC_MODE_LTF2 = 1,      /**< Acquire the complete HE-LTF2 */
    CSI_HE_STBC_MODE_AVERAGE = 2,   /**< Acquire average of HE-LTF1 and HE-LTF2 */
} csi_he_stbc_mode_t;
#endif

/**
 * @brief Radar CSI configuration structure
 */
typedef struct {
    wifi_csi_filtered_cb_t csi_filtered_cb;     /**< Register the callback function of Wi-Fi CSI data */
    void *csi_filtered_cb_ctx;                  /**< Context argument, passed to callback function of Wi-Fi CSI */
    bool csi_compensate_en;                      /**< Whether to enable CSI compensation */
    uint8_t filter_mac[6];                       /**< Filter MAC address of the specified device, no filtering: [0xff:0xff:0xff:0xff:0xff:0xff] */
    uint8_t filter_dmac[6];                      /**< Filter destination MAC address of the specified device, no filtering: [0xff:0xff:0xff:0xff:0xff:0xff] */
    bool filter_dmac_flag;                       /**< Whether to enable destination MAC address filtering */
    uint16_t csi_recv_interval;                  /**< The interval of receiving CSI data, unit: ms */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    bool acquire_csi_lltf       : 1;             /**< Enable to acquire L-LTF */
    bool acquire_csi_ht20       : 1;             /**< Enable to acquire HT-LTF when receiving an HT20 PPDU */
    bool acquire_csi_ht40       : 1;             /**< Enable to acquire HT-LTF when receiving an HT40 PPDU */
    bool acquire_csi_vht        : 1;             /**< Enable to acquire VHT-LTF when receiving a VHT20 PPDU */
    bool acquire_csi_su         : 1;             /**< Enable to acquire HE-LTF when receiving an HE20 SU PPDU */
    bool acquire_csi_mu         : 1;             /**< Enable to acquire HE-LTF when receiving an HE20 MU PPDU */
    bool acquire_csi_dcm        : 1;             /**< Enable to acquire HE-LTF when receiving an HE20 DCM applied PPDU */
    bool acquire_csi_beamformed : 1;             /**< Enable to acquire HE-LTF when receiving an HE20 Beamformed applied PPDU */
    csi_he_stbc_mode_t acquire_csi_he_stbc_mode; /**< HE STBC mode for acquiring CSI data */
    uint8_t val_scale_cfg           : 4;         /**< Value scale configuration, range: 0-8 */
    bool dump_ack_en             : 1;             /**< Enable to dump 802.11 ACK frame, default disabled */
#endif
#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
    bool lltf_en;                                /**< Enable to receive legacy long training field (LLTF) data. Default enabled */
    bool htltf_en;                               /**< Enable to receive HT long training field (HTLTF) data. Default enabled */
    bool stbc_htltf2_en;                         /**< Enable to receive space time block code HT long training field (STBC-HTLTF2) data. Default enabled */
    bool ltf_merge_en;                           /**< Enable to generate HTLTF data by averaging LLTF and HT-LTF data when receiving HT packet. Otherwise, use HT-LTF data directly. Default enabled */
    bool channel_filter_en;                      /**< Enable to turn on channel filter to smooth adjacent sub-carrier. Disable it to keep independence of adjacent sub-carrier. Default enabled */
    bool manu_scale;                             /**< Manually scale the CSI data by left shifting or automatically scale the CSI data. If set true, please set the shift bits. false: automatically. true: manually. Default false */
    uint8_t shift;                               /**< Manually left shift bits of the scale of the CSI data. The range of the left shift bits is 0~15 */
    bool dump_ack_en;                            /**< Enable to dump 802.11 ACK frame, default disabled */
#endif
} esp_radar_csi_config_t;

/**
 * @brief Radar WiFi configuration structure
 */
typedef struct {
    wifi_band_mode_t band_mode;      /**< WiFi band mode */
    wifi_protocols_t protocols;       /**< WiFi protocols */
    wifi_bandwidths_t bandwidths;     /**< WiFi bandwidths */
    uint8_t channel;                  /**< Primary channel */
    wifi_second_chan_t second_chan;   /**< Secondary channel */
    uint8_t mac_address[6];           /**< MAC address */
} esp_radar_wifi_config_t;

/**
 * @brief Radar ESP-NOW configuration structure
 */
typedef struct {
    wifi_phy_rate_t rate;            /**< Physical layer rate */
    uint8_t peer_addr[6];             /**< Peer MAC address */
    uint8_t pmk[16];                  /**< Pairwise Master Key (PMK) */
} esp_radar_espnow_config_t;

/**
 * @brief Radar LTF type enumeration
 */
typedef enum {
    RADAR_LTF_TYPE_LLTF = 0,        /**< Use Legacy Long Training Field (L-LTF) */
    RADAR_LTF_TYPE_HTLTF,           /**< Use HT Long Training Field (HT-LTF) */
    RADAR_LTF_TYPE_STBC_HTLTF,      /**< Use STBC HT Long Training Field (STBC-HT-LTF) */
#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
    RADAR_LTF_TYPE_HELTF,           /**< Use HE Long Training Field (HE-LTF) */
    RADAR_LTF_TYPE_STBC_HELTF,      /**< Use STBC HE Long Training Field (STBC-HE-LTF) */
#endif
} esp_radar_ltf_type_t;

/**
 * @brief Radar decoder configuration structure
 */
typedef struct {
    wifi_radar_cb_t wifi_radar_cb;         /**< Register the callback function of Wi-Fi radar data */
    void *wifi_radar_cb_ctx;               /**< Context argument, passed to callback function of Wi-Fi radar */
    esp_radar_ltf_type_t ltf_type;         /**< LTF type to use for radar analysis */
    uint8_t sub_carrier_step_size;         /**< Sub-carrier step size for CSI data sampling, default: 4 */
    uint8_t outliers_threshold;            /**< CSI outliers threshold for filtering, set to 0 to disable, default: 8 */
    struct {                                /**< Algorithm configuration */
        UBaseType_t csi_handle_priority;  /**< The priority of the task that handles the CSI data */
        UBaseType_t csi_combine_priority; /**< The priority of the task that combines the CSI data */
        uint16_t csi_handle_time;         /**< The time of handling CSI data, unit: ms */
        uint8_t dec_window_size;          /**< Detection sliding window size, default: 4 */
    };
} esp_radar_dec_config_t;

#define ESP_RADAR_WIFI_CONFIG_DEFAULT() (esp_radar_wifi_config_t){ \
    .band_mode = WIFI_BAND_MODE_2G_ONLY, \
    .protocols = { \
        .ghz_2g = WIFI_PROTOCOL_11N, \
        .ghz_5g = WIFI_PROTOCOL_11N, \
    }, \
    .bandwidths = { \
        .ghz_2g = WIFI_BW_HT40, \
        .ghz_5g = WIFI_BW_HT40, \
    }, \
    .channel = 11, \
    .second_chan = WIFI_SECOND_CHAN_BELOW, \
    .mac_address = {0x00, 0x00, 0x00, 0x00, 0x00, 0x00}, \
}

#define ESP_RADAR_ESPNOW_CONFIG_DEFAULT() (esp_radar_espnow_config_t){ \
    .rate = WIFI_PHY_RATE_MCS0_LGI, \
    .peer_addr = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .pmk = {'p', 'm', 'k', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '1', '2', '3'}, \
}

#if CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){ \
    .csi_filtered_cb = NULL, \
    .csi_filtered_cb_ctx = NULL, \
    .csi_compensate_en = true, \
    .filter_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac_flag = false, \
    .csi_recv_interval = 10, \
    .acquire_csi_lltf = true, \
    .acquire_csi_ht20 = true, \
    .acquire_csi_ht40 = true, \
    .acquire_csi_vht = true, \
    .acquire_csi_su = true, \
    .acquire_csi_mu = true, \
    .acquire_csi_dcm = true, \
    .acquire_csi_beamformed = true, \
    .acquire_csi_he_stbc_mode = CSI_HE_STBC_MODE_AVERAGE, \
    .val_scale_cfg = 0, \
    .dump_ack_en = false, \
}
#elif CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){ \
    .csi_filtered_cb = NULL, \
    .csi_filtered_cb_ctx = NULL, \
    .csi_compensate_en = true, \
    .filter_mac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff}, \
    .filter_dmac_flag = false, \
    .csi_recv_interval = 10, \
    .lltf_en = true, \
    .htltf_en = true, \
    .stbc_htltf2_en = true, \
    .ltf_merge_en = false, \
    .channel_filter_en = false, \
    .manu_scale = false, \
    .shift = 0, \
    .dump_ack_en = false, \
}
#else
#define ESP_RADAR_CSI_CONFIG_DEFAULT() (esp_radar_csi_config_t){0}
#endif

#define ESP_RADAR_DEC_CONFIG_DEFAULT() (esp_radar_dec_config_t){ \
    .wifi_radar_cb = NULL, \
    .wifi_radar_cb_ctx = NULL, \
    .ltf_type = RADAR_LTF_TYPE_LLTF, \
    .sub_carrier_step_size = 4, \
    .outliers_threshold = 8, \
    .csi_handle_priority = configMAX_PRIORITIES - 1, \
    .csi_combine_priority = configMAX_PRIORITIES - 1, \
    .csi_handle_time = 200, \
    .dec_window_size = 4, \
}
/**
 * @brief Radar configuration structure
 */
typedef struct {
    esp_radar_wifi_config_t wifi_config;   /**< WiFi configuration */
    esp_radar_csi_config_t csi_config;     /**< CSI configuration */
    esp_radar_espnow_config_t espnow_config; /**< ESP-NOW configuration */
    esp_radar_dec_config_t dec_config;     /**< Decoder configuration */
} esp_radar_config_t;

#define ESP_RADAR_CONFIG_DEFAULT() (esp_radar_config_t){ \
    .wifi_config = ESP_RADAR_WIFI_CONFIG_DEFAULT(), \
    .csi_config = ESP_RADAR_CSI_CONFIG_DEFAULT(), \
    .espnow_config = ESP_RADAR_ESPNOW_CONFIG_DEFAULT(), \
    .dec_config = ESP_RADAR_DEC_CONFIG_DEFAULT(), \
}
/**
 * @brief Initialize the radar system with the given configuration
 *
 * This function initializes all radar subsystems including WiFi, ESP-NOW, CSI, and decoder.
 * It must be called before using any other radar functions.
 *
 * @param config Pointer to the radar configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_init(esp_radar_config_t *config);

/**
 * @brief Deinitialize the radar system
 *
 * This function stops CSI reception, frees all allocated resources, and resets the radar system.
 * After calling this function, esp_radar_init() must be called again before using radar functions.
 *
 * @return
 *        - ESP_OK: Success
 */
esp_err_t esp_radar_deinit(void);

/**
 * @brief Start radar data processing
 *
 * This function starts the radar data processing tasks and allocates necessary buffers.
 * It creates two FreeRTOS tasks: one for CSI data detection and one for CSI data preprocessing.
 * The buffer size and window size are calculated based on csi_handle_time and csi_recv_interval.
 *
 * @return
 *        - ESP_OK: Success
 */
esp_err_t esp_radar_start(void);

/**
 * @brief Stop radar data processing
 *
 * This function stops the radar data processing tasks, waits for them to exit,
 * and frees all allocated buffers. The radar system can be restarted by calling esp_radar_start() again.
 *
 * @return
 *        - ESP_OK: Success
 */
esp_err_t esp_radar_stop(void);

/**
 * @brief Initialize WiFi subsystem for radar
 *
 * This function initializes the WiFi subsystem with the specified configuration.
 * It sets up WiFi mode, protocols, bandwidths, channel, and MAC address.
 * The function handles platform-specific limitations (e.g., 5GHz support).
 *
 * @param config Pointer to the WiFi configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_wifi_init(esp_radar_wifi_config_t *config);

/**
 * @brief Reinitialize WiFi subsystem with new configuration
 *
 * This function stops and deinitializes the current WiFi subsystem,
 * then reinitializes it with the new configuration. Useful for changing WiFi settings at runtime.
 *
 * @param config Pointer to the new WiFi configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_wifi_reinit(esp_radar_wifi_config_t *config);

/**
 * @brief Initialize CSI (Channel State Information) subsystem
 *
 * This function configures the CSI subsystem to receive and process WiFi CSI data.
 * It sets up CSI callbacks, filters, and acquisition modes based on the configuration.
 * The configuration is platform-specific (different for ESP32C5/C6/C61 vs ESP32/S2/S3/C3).
 *
 * @param config Pointer to the CSI configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_csi_init(esp_radar_csi_config_t *config);

/**
 * @brief Initialize ESP-NOW subsystem for radar
 *
 * This function initializes ESP-NOW with the specified peer address and PMK (Pairwise Master Key).
 * It sets up the ESP-NOW peer and configures the physical layer rate.
 * Note: WiFi must be initialized first (via esp_radar_wifi_init()).
 *
 * @param config Pointer to the ESP-NOW configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 *        - ESP_ERR_INVALID_STATE: WiFi not initialized
 */
esp_err_t esp_radar_espnow_init(esp_radar_espnow_config_t *config);

/**
 * @brief Initialize radar decoder subsystem
 *
 * This function initializes the radar decoder with the specified configuration.
 * It sets up the decoder callbacks, LTF type, sub-carrier step size, and algorithm parameters.
 *
 * @param config Pointer to the decoder configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_dec_init(esp_radar_dec_config_t *config);

/**
 * @brief Get current radar configuration
 *
 * This function retrieves the current configuration of all radar subsystems
 * (WiFi, CSI, ESP-NOW, and decoder) and copies them to the provided structure.
 *
 * @param config Pointer to the configuration structure where the current config will be stored. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_get_config(esp_radar_config_t *config);

/**
 * @brief Change radar configuration at runtime
 *
 * This function updates the radar configuration while the system is running.
 * If radar is currently running, it will be stopped, configuration updated, and then restarted.
 * Only changed configurations are updated (compared by memcmp).
 *
 * @param config Pointer to the new configuration structure. Must not be NULL.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_INVALID_ARG: Invalid argument
 */
esp_err_t esp_radar_change_config(esp_radar_config_t *config);

/**
 * @brief Start radar calibration/training mode
 *
 * This function starts the calibration process for the radar system.
 * During calibration, the system collects reference data to calculate thresholds
 * for detecting human presence (wander) and movement (jitter).
 * The calibration status is set to RADAR_CALIBRATE_PROGRESS.
 *
 * @return
 *        - ESP_OK: Success
 */
esp_err_t esp_radar_train_start(void);

/**
 * @brief Remove all calibration data
 *
 * This function clears all collected calibration data and resets the calibration status.
 * The calibration data buffers are freed, and statistics are reset.
 * The calibration status is set to RADAR_CALIBRATE_NO.
 *
 * @return
 *        - ESP_OK: Success
 */
esp_err_t esp_radar_train_remove(void);

/**
 * @brief Stop calibration and get calculated thresholds
 *
 * This function stops the calibration process and calculates the thresholds
 * for wander (human presence detection) and jitter (human movement detection).
 * The calibration status is set to RADAR_CALIBRATE_COMPLETE.
 *
 * @param wander_threshold Pointer to store the calculated wander threshold (1 - average correlation).
 *                         Can be NULL if not needed.
 * @param jitter_threshold Pointer to store the calculated jitter threshold (1 - static correlation).
 *                        Can be NULL if not needed.
 * @return
 *        - ESP_OK: Success
 *        - ESP_ERR_NOT_SUPPORTED: Calibration not initialized
 *        - ESP_ERR_INVALID_STATE: No calibration data collected
 */
esp_err_t esp_radar_train_stop(float *wander_threshold, float *jitter_threshold);

#ifdef __cplusplus
}
#endif
