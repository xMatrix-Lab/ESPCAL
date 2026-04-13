/*
 * SPDX-FileCopyrightText: 2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#pragma once

#include "esp_err.h"
#include <stdint.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Compute motion detection result
 *
 * @param cols Number of subcarriers (columns)
 * @param row_0 Number of rows in the first data segment
 * @param data_0 First data segment [row_0][cols]
 * @param row_1 Number of rows in the second data segment
 * @param data_1 Second data segment [row_1][cols]
 * @param output Output result [cols]
 * @return esp_err_t ESP_OK on success
 */
esp_err_t esp_radar_motion_dec_compute(uint32_t cols,
                                       uint32_t row_0, const float data_0[row_0][cols],
                                       uint32_t row_1, const float data_1[row_1][cols],
                                       float output[cols]);

/**
 * @brief Compute waveform wander
 *
 * @param a Vector A
 * @param b Vector B
 * @param len Vector length
 * @return float Waveform wander value [0.0, 1.0], smaller value indicates more similarity (more stable)
 */
float esp_radar_motion_dec_wander_compute(const float *a, const float *b, size_t len);

/**
 * @brief Compute waveform jitter
 *
 * @param current Current result [cols]
 * @param history History data buffer [window_size][cols]
 * @param cols Number of subcarriers
 * @param window_size Window size
 * @param buff_num Current buffer count (used to index history data)
 * @return float Computed jitter value [0.0, 1.0], smaller value indicates more stability, returns 0.0 if data is insufficient
 */
float esp_radar_motion_dec_compute_jitter(float *current,
                                          float **history,
                                          uint16_t cols,
                                          uint8_t window_size,
                                          uint32_t buff_num);

#ifdef __cplusplus
}
#endif
