/*
 * SPDX-FileCopyrightText: 2025 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#include <stdint.h>
#include <stdbool.h>

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

#if CONFIG_IDF_TARGET_ESP32S3 || CONFIG_IDF_TARGET_ESP32C3 || CONFIG_IDF_TARGET_ESP32C5 || CONFIG_IDF_TARGET_ESP32C6 || CONFIG_IDF_TARGET_ESP32C61
#define WIFI_CSI_PHY_GAIN_ENABLE          1
#endif

/**
 * @brief RX gain status
 */
typedef enum {
    RX_GAIN_COLLECT = 0,   /**< Data collection in progress */
    RX_GAIN_READY,         /**< Baseline has been calculated */
    RX_GAIN_FORCE          /**< Gain manually forced */
} rx_gain_status_t;

/**
 * @brief Check whether "automatic RX gain has been forcibly set"
 *
 * @return true   Gain has been forcibly set via esp_csi_gain_ctrl_set_rx_force_gain()
 * @return false  Normal AGC automatic gain is in use
 */
rx_gain_status_t esp_csi_gain_ctrl_get_gain_status(void);

/**
 * @brief Get the RX gain "baseline" values (median)
 *
 * Multiple calls to esp_csi_gain_ctrl_record_rx_gain() are required first.
 *
 * @param[out] agc_gain  Baseline AGC gain
 * @param[out] fft_gain  Baseline FFT gain
 *
 * @return
 *      - ESP_OK: Success
 *      - ESP_ERR_INVALID_ARG: Invalid argument
 *      - ESP_ERR_INVALID_STATE: Baseline not ready yet
 *      - ESP_ERR_NO_MEM: Memory allocation failed
 */
esp_err_t esp_csi_gain_ctrl_get_rx_gain_baseline(uint8_t *agc_gain, int8_t *fft_gain);

/**
 * @brief Record a single RX gain sample for later baseline calculation
 *
 * @param agc_gain Current AGC gain
 * @param fft_gain Current FFT gain
 *
 * @return ESP_OK
 */
esp_err_t esp_csi_gain_ctrl_record_rx_gain(uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief Forcefully set the receive gain (may cause packet loss)
 *
 * If both agc_gain and fft_gain are 0, forced gain is disabled and automatic gain resumes.
 *
 * @param agc_gain AGC gain
 * @param fft_gain FFT gain
 *
 * @return
 *      - ESP_OK
  */
esp_err_t esp_csi_gain_ctrl_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief Reset RX gain baseline statistics
 */
void esp_csi_gain_ctrl_reset_rx_gain_baseline(void);

/**
 * @brief Calculate the compensation factor for the current RX gain
 *
 * @param[out] compensate_gain  Calculated compensation factor
 * @param agc_gain              Current AGC gain
 * @param fft_gain              Current FFT gain
 *
 * @return
 *      - ESP_OK
 *      - ESP_ERR_INVALID_STATE: Baseline not ready or other invalid state
 */
esp_err_t esp_csi_gain_ctrl_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief Extended version of RX gain compensation supporting 8-bit and 16-bit CSI samples
 *
 * @param data               Data buffer to be compensated (in-place)
 * @param size               Buffer length in bytes
 * @param samples_are_16bit  true if each CSI component is 16-bit, false for 8-bit
 * @param compensate_gain    Output compensation factor
 * @param agc_gain           Current AGC gain
 * @param fft_gain           Current FFT gain
 *
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t esp_csi_gain_ctrl_compensate_rx_gain(void *data, uint16_t size, bool samples_are_16bit,
                                               float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);

/**
 * @brief Extract RX gain from CSI packet information
 *
 * @param rx_ctrl       Pointer to RX control information (wifi_pkt_rx_ctrl_t)
 * @param[out] agc_gain Extracted AGC gain
 * @param[out] fft_gain Extracted FFT gain
 */
void esp_csi_gain_ctrl_get_rx_gain(const void *rx_ctrl, uint8_t *agc_gain, int8_t *fft_gain);

#ifdef __cplusplus
}
#endif
