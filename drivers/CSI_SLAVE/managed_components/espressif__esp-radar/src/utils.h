/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#pragma once

#ifdef __cplusplus
extern "C"
{
#endif

bool mac_str2hex(const char *mac_str, uint8_t *mac_hex);
float avg(const float *array, size_t len);
float sum(const float *a, size_t len);
float max(const float *array, size_t len, float percent);
float min(const float *array, size_t len, float percent);
float trimmean(const float *array, size_t len, float percent);

float cov(const float *x, const float *y, size_t len);
float std(const float *a, size_t len);
float dis(const float *a, const float *b, size_t len);
int cmp_float(const void *_a, const void *_b);
float median(const float *a, size_t len);

#ifdef __cplusplus
}
#endif
