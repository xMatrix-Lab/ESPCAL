/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_radar_motion_dec.h"

static const char *TAG = "motion_dec_test";

#define TEST_COLS 64
#define TEST_ROW_0 10
#define TEST_ROW_1 10
#define TEST_WINDOW_SIZE 5

/**
 * @brief Test esp_radar_motion_dec_compute function
 */
static void test_motion_dec_compute(void)
{
    ESP_LOGI(TAG, "========== Test esp_radar_motion_dec_compute ==========");

    // Use dynamic allocation to avoid stack overflow
    float (*data_0)[TEST_COLS] = (float (*)[TEST_COLS])malloc(TEST_ROW_0 * TEST_COLS * sizeof(float));
    float (*data_1)[TEST_COLS] = (float (*)[TEST_COLS])malloc(TEST_ROW_1 * TEST_COLS * sizeof(float));
    float *output = (float *)malloc(TEST_COLS * sizeof(float));

    if (!data_0 || !data_1 || !output) {
        ESP_LOGE(TAG, "Failed to allocate memory for test data");
        if (data_0) {
            free(data_0);
        }
        if (data_1) {
            free(data_1);
        }
        if (output) {
            free(output);
        }
        return;
    }

    // Initialize test data - first segment: simple incremental pattern
    for (int i = 0; i < TEST_ROW_0; i++) {
        for (int j = 0; j < TEST_COLS; j++) {
            data_0[i][j] = (float)(i * TEST_COLS + j) * 0.1f;
        }
    }

    // Initialize test data - second segment: slightly different pattern
    for (int i = 0; i < TEST_ROW_1; i++) {
        for (int j = 0; j < TEST_COLS; j++) {
            data_1[i][j] = (float)((i + TEST_ROW_0) * TEST_COLS + j) * 0.1f + 0.5f;
        }
    }

    // Call the function
    esp_err_t ret = esp_radar_motion_dec_compute(TEST_COLS,
                                                 TEST_ROW_0, data_0,
                                                 TEST_ROW_1, data_1,
                                                 output);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "esp_radar_motion_dec_compute: SUCCESS");
        ESP_LOGI(TAG, "Output sample (first 5 values): %.4f, %.4f, %.4f, %.4f, %.4f",
                 output[0], output[1], output[2], output[3], output[4]);
    } else {
        ESP_LOGE(TAG, "esp_radar_motion_dec_compute: FAILED (ret=%d)", ret);
    }

    // Free memory
    free(data_0);
    free(data_1);
    free(output);
}

/**
 * @brief Test esp_radar_motion_dec_wander_compute function
 */
static void test_wander_compute(void)
{
    ESP_LOGI(TAG, "========== Test esp_radar_motion_dec_wander_compute ==========");

    // Create test vectors
    float vector_a[TEST_COLS];
    float vector_b[TEST_COLS];
    float vector_c[TEST_COLS];

    // Initialize vector A: sine wave pattern
    for (int i = 0; i < TEST_COLS; i++) {
        vector_a[i] = (float)sin(2.0 * M_PI * i / TEST_COLS);
    }

    // Initialize vector B: similar to A (slightly offset)
    for (int i = 0; i < TEST_COLS; i++) {
        vector_b[i] = (float)sin(2.0 * M_PI * i / TEST_COLS + 0.1);
    }

    // Initialize vector C: completely different from A (random values)
    for (int i = 0; i < TEST_COLS; i++) {
        vector_c[i] = (float)(rand() % 100) / 10.0f;
    }

    // Test similar vectors
    float wander_ab = esp_radar_motion_dec_wander_compute(vector_a, vector_b, TEST_COLS);
    ESP_LOGI(TAG, "Wander (similar vectors): %.4f (smaller is more similar)", wander_ab);

    // Test dissimilar vectors
    float wander_ac = esp_radar_motion_dec_wander_compute(vector_a, vector_c, TEST_COLS);
    ESP_LOGI(TAG, "Wander (different vectors): %.4f (smaller is more similar)", wander_ac);

    if (wander_ab < wander_ac) {
        ESP_LOGI(TAG, "Result: Similar vectors have smaller wander value - CORRECT");
    } else {
        ESP_LOGW(TAG, "Result: Unexpected - similar vectors have larger wander value");
    }
}

/**
 * @brief Test esp_radar_motion_dec_compute_jitter function
 */
static void test_jitter_compute(void)
{
    ESP_LOGI(TAG, "========== Test esp_radar_motion_dec_compute_jitter ==========");

    // Create history data buffer
    float *history[TEST_WINDOW_SIZE];
    for (int i = 0; i < TEST_WINDOW_SIZE; i++) {
        history[i] = (float *)malloc(TEST_COLS * sizeof(float));
        if (!history[i]) {
            ESP_LOGE(TAG, "Failed to allocate history buffer %d", i);
            return;
        }
    }

    // Create current data
    float current[TEST_COLS];

    // Initialize history data: stable pattern
    for (int i = 0; i < TEST_WINDOW_SIZE; i++) {
        for (int j = 0; j < TEST_COLS; j++) {
            history[i][j] = (float)sin(2.0 * M_PI * j / TEST_COLS) + i * 0.01f;
        }
    }

    // Test 1: current data similar to history data (stable case)
    for (int j = 0; j < TEST_COLS; j++) {
        current[j] = (float)sin(2.0 * M_PI * j / TEST_COLS) + (TEST_WINDOW_SIZE - 1) * 0.01f;
    }

    float jitter_stable = esp_radar_motion_dec_compute_jitter(current, history, TEST_COLS,
                                                              TEST_WINDOW_SIZE, TEST_WINDOW_SIZE);
    ESP_LOGI(TAG, "Jitter (stable case): %.4f (smaller is more stable)", jitter_stable);

    // Test 2: current data differs significantly from history data (motion case)
    for (int j = 0; j < TEST_COLS; j++) {
        current[j] = (float)(rand() % 100) / 10.0f;
    }

    float jitter_motion = esp_radar_motion_dec_compute_jitter(current, history, TEST_COLS,
                                                              TEST_WINDOW_SIZE, TEST_WINDOW_SIZE);
    ESP_LOGI(TAG, "Jitter (motion case): %.4f (smaller is more stable)", jitter_motion);

    // Test 3: insufficient data case
    float jitter_insufficient = esp_radar_motion_dec_compute_jitter(current, history, TEST_COLS,
                                                                    TEST_WINDOW_SIZE, 2);
    ESP_LOGI(TAG, "Jitter (insufficient data): %.4f (should be 0.0)", jitter_insufficient);

    // Free memory
    for (int i = 0; i < TEST_WINDOW_SIZE; i++) {
        free(history[i]);
    }
}

void app_main()
{
    ESP_LOGI(TAG, "ESP Radar Motion Detection Test Application");
    ESP_LOGI(TAG, "===========================================");

    vTaskDelay(pdMS_TO_TICKS(1000));

    // Run all tests
    test_motion_dec_compute();
    vTaskDelay(pdMS_TO_TICKS(500));

    test_wander_compute();
    vTaskDelay(pdMS_TO_TICKS(500));

    test_jitter_compute();
    vTaskDelay(pdMS_TO_TICKS(500));

    ESP_LOGI(TAG, "===========================================");
    ESP_LOGI(TAG, "All tests completed!");
}
