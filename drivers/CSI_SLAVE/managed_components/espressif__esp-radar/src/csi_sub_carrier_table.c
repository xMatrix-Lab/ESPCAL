/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include "esp_wifi_types_generic.h"
#include <stdint.h>
#include <stddef.h>
#include <stdbool.h>
#include "esp_radar.h"

const csi_sub_carrier_table_t sub_carrier_table[] = {
#if CONFIG_IDF_TARGET_ESP32 || CONFIG_IDF_TARGET_ESP32S2 || CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3
    /**< ------------------- secondary channel : none ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: 0~31, -32~-1, 128 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .lltf_bytes        = 104,
        .lltf = {{76, 128}, {2, 54}},   // 1, 11, 0
    },
    /**< HT,     20 MHz, non STBC, LLTF: 0~31, -32~-1, HT-ltf: 0~31, -32~-1, 256 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .lltf = {{76, 128}, {2, 54}},   // 1, 11, 0
        .ht_ltf = {{200, 256}, {130, 186}} // 1, 7, 0
    },
    /**< HT,     20 MHz,     STBC, LLTF: 0~31, -32~-1, HT-ltf: 0~31, -32~-1, STBC-HT-LTF: 0~31, -32~-1, 384 */
    {
        .second            = WIFI_SECOND_CHAN_NONE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .stbc_ht_ltf_bytes = 112,
        .lltf = {{76, 128}, {2, 54}},              // 1, 11, 0
        .ht_ltf = {{200, 256}, {130, 186}},        // 1, 7, 0
        .stbc_ht_ltf = {{258, 314}, {328, 384}}    // 1, 7, 0
    },

    /**< ------------------- secondary channel : below ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: 0~63, 128 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .lltf_bytes        = 104,
        .lltf = {{12, 64}, {66, 118}},         // 6, 1, 5
    },
    /**< HT,     20 MHz, non STBC, LLTF: 0~63, HT-ltf: 0~63, 256 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .lltf   = {{12, 64}, {66, 118}},      // 6, 1, 5
        .ht_ltf = {{132, 188}, {190, 246}},   // 2, 1, 5
    },
    /**< HT,     20 MHz,     STBC, LLTF: 0~63, HT-ltf: 0~62, STBC-HT-LTF: 0~62, 380 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 380,
        .valid_bytes       = 328,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .stbc_ht_ltf_bytes = 112,
        .lltf   = {{12, 64}, {66, 118}},           // 6, 1, 5
        .ht_ltf = {{132, 188}, {190, 246}},        // 2, 1, 5
        .stbc_ht_ltf = {{256, 312}, {314, 370}},   // 0, 1, 5
    },

    /**< HT,     40 MHz, non STBC, LLTF: 0~63, HT-ltf: 0~63, -64~-1, 384 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 224,
        .lltf   = {{12, 64}, {66, 118}},                           // 6, 1, 5
        .ht_ltf = {{268, 324}, {326, 382}, {132, 188}, {190, 246}} // 2, 1, 5   6, 1, 1
    },
    /**< HT,     40 MHz,     STBC, LLTF: 0~63, HT-ltf: 0~60, -60~-1, STBC-HT-LTF: 0~60, -60~-1, 612 */
    {
        .second            = WIFI_SECOND_CHAN_BELOW,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 612,
        .valid_bytes       = 552,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 224,
        .stbc_ht_ltf_bytes = 224,
        .lltf   = {{12, 64}, {66, 118}},                                // 6, 1, 5
        .ht_ltf = {{254, 310}, {312, 368}, {132, 188}, {190, 246}},     // 2, 1, 2   2, 1, 1
        .stbc_ht_ltf = {{496, 552}, {554, 610}, {374, 430}, {432, 488}} // 2, 1, 2   2, 1, 1
    },

    /**< ------------------- secondary channel : above ------------------- **/
    /**< non HT, 20 MHz, non STBC, LLTF: 0~63, 128 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .lltf_bytes        = 104,
        .lltf = {{12, 64}, {66, 118}},         // 6, 1, 5
    },
    /**< HT,     20 MHz, non STBC, LLTF: 0~63, HT-ltf: 0~63, 256 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 216,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .lltf   = {{12, 64}, {66, 118}},      // 6, 1, 5
        .ht_ltf = {{132, 188}, {190, 246}},   // 2, 1, 5
    },
    /**< HT,     20 MHz,     STBC, LLTF: 0~63, HT-ltf: 0~62, STBC-HT-LTF: 0~62, 380 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 380,
        .valid_bytes       = 328,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 112,
        .stbc_ht_ltf_bytes = 112,
        .lltf   = {{12, 64}, {66, 118}},           // 6, 1, 5
        .ht_ltf = {{132, 188}, {190, 246}},        // 2, 1, 5
        .stbc_ht_ltf = {{256, 312}, {314, 370}},   // 0, 1, 5
    },

    /**< HT,     40 MHz, non STBC, LLTF: 0~63, HT-ltf: 0~63, -64~-1, 384 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 384,
        .valid_bytes       = 328,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 224,
        .lltf   = {{12, 64}, {66, 118}},                           // 6, 1, 5
        .ht_ltf = {{268, 324}, {326, 382}, {132, 188}, {190, 246}} // 2, 1, 5   6, 1, 1
    },
    /**< HT,     40 MHz,     STBC, LLTF: 0~63, HT-ltf: 0~60, -60~-1, STBC-HT-LTF: 0~60, -60~-1, 612 */
    {
        .second            = WIFI_SECOND_CHAN_ABOVE,
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 612,
        .valid_bytes       = 552,
        .lltf_bytes        = 104,
        .ht_ltf_bytes      = 224,
        .stbc_ht_ltf_bytes = 224,
        .lltf   = {{12, 64}, {66, 118}},                                // 6, 1, 5
        .ht_ltf = {{254, 310}, {312, 368}, {132, 188}, {190, 246}},     // 2, 1, 2   2, 1, 1
        .stbc_ht_ltf = {{496, 552}, {554, 610}, {374, 430}, {432, 488}} // 2, 1, 2   2, 1, 1
    },

#elif CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C61
    /**< non HT, 20 MHz, non STBC, LLTF: 0~26, -26~-1, 106 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 106,
        .valid_bytes       = 104,
        .lltf_bytes        = 104,
        .lltf = {{0, 52}, {52, 104}},
    },
    /**< HT,     20 MHz, non STBC, HT-ltf: 0~28, -28~-1, 114 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 114,
        .valid_bytes       = 112,
        .ht_ltf_bytes      = 112,
        .ht_ltf = {{0, 56}, {58, 114}},
    },
    /**< HT,     20 MHz,     STBC, HT-ltf: 0~28, -28~-1, STBC-HT-LTF: 0~28, -28~-1, 228 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 228,
        .valid_bytes       = 224,
        .ht_ltf_bytes      = 112,
        .stbc_ht_ltf_bytes = 112,
        .ht_ltf = {{0, 56}, {58, 114}},
        .stbc_ht_ltf = {{114, 170}, {172, 228}},
    },
    /**< HT,     40 MHz, non STBC, HT-ltf: 0~58, -58~-1, 234 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 234,
        .valid_bytes       = 228,
        .ht_ltf_bytes      = 228,
        .ht_ltf = {{0, 114}, {120, 234}},
    },
    /**< HT,     40 MHz,     STBC, HT-ltf: 0~58, -58~-1, STBC-HT-LTF: 0~58, -58~-1, 468 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 468,
        .valid_bytes       = 456,
        .ht_ltf_bytes      = 228,
        .stbc_ht_ltf_bytes = 228,
        .ht_ltf = {{0, 114}, {120, 234}},
        .stbc_ht_ltf = {{234, 348}, {354, 468}},
    },
    /**< HE,     20 MHz, non STBC, HE-LTF: 0~122, -122~-1, 490 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HE,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 490,
        .valid_bytes       = 484,
        .he_ltf_bytes      = 484,
        .he_ltf = {{0, 242}, {248, 490}},
    },
    /**< HE,     20 MHz,     STBC, STBC-HE-LTF: 0~122, -122~-1, 490 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HE,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 490,
        .valid_bytes       = 484,
        .stbc_he_ltf_bytes = 484,
        .stbc_he_ltf = {{0, 242}, {248, 490}},
    },
#elif CONFIG_IDF_TARGET_ESP32C6
    /**< non HT, 20 MHz, non STBC, LLTF: 0~31, -32~-1, 128 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_NON_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 104,
        .lltf_bytes        = 104,
        .lltf = {{12, 64}, {66, 118}}, //
    },
    /**< HT,     20 MHz, non STBC, HT-ltf: 0~31, -32~-1, 128 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 128,
        .valid_bytes       = 112,
        .ht_ltf_bytes      = 112,
        .ht_ltf = {{8, 64}, {66, 122}},
    },
    /**< HT,     20 MHz,     STBC, HT-ltf: 0~31, -32~-1, STBC-HT-LTF: 0~31, -32~-1, 256 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 256,
        .valid_bytes       = 224,
        .ht_ltf_bytes      = 112,
        .stbc_ht_ltf_bytes = 112,
        .ht_ltf = {{8, 64}, {66, 122}},
        .stbc_ht_ltf = {{136, 192}, {194, 250}},
    },
    /**< HT,     40 MHz, non STBC, HT-ltf: 0~63, -64~-1, 256 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = false,
        .total_bytes       = 256,
        .valid_bytes       = 228,
        .ht_ltf_bytes      = 228,
        .ht_ltf = {{12, 126}, {132, 246}},
    },
    /**< HT,     40 MHz,     STBC, HT-ltf: 0~63, -64~-1, STBC-HT-LTF: 0~63, -64~-1, 512 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HT,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_40MHZ,
        .stbc              = true,
        .total_bytes       = 512,
        .valid_bytes       = 456,
        .ht_ltf_bytes      = 228,
        .stbc_ht_ltf_bytes = 228,
        .ht_ltf = {{12, 126}, {132, 246}},
        .stbc_ht_ltf = {{268, 382}, {388, 502}},
    },
    /**< HE,     20 MHz, non STBC, HE-LTF: 0~127, -128~-1, 512 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HE,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = false,
        .total_bytes       = 512,
        .valid_bytes       = 484,
        .he_ltf_bytes      = 484,
        .he_ltf = {{12, 254}, {260, 502}},
    },
    /**< HE,     20 MHz,     STBC, STBC-HE-LTF: 0~127, -128~-1, 512 */
    {
        .signal_mode       = WIFI_SIGNAL_MODE_HE,
        .channel_bandwidth = WIFI_CHANNEL_BANDWIDTH_20MHZ,
        .stbc              = true,
        .total_bytes       = 512,
        .valid_bytes       = 484,
        .stbc_he_ltf_bytes = 484,
        .stbc_he_ltf = {{12, 254}, {260, 502}},
    },
#endif

};

const size_t sub_carrier_table_size = sizeof(sub_carrier_table) / sizeof(sub_carrier_table[0]);
