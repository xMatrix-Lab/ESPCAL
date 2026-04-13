/*
 * SPDX-FileCopyrightText: 2025-2026 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <errno.h>
#include <string.h>
#include <stdio.h>
#include <string.h>
#include <sys/param.h>
#include <math.h>
#include <stdbool.h>

#include "esp_radar.h"

float avg(const float *a, size_t len)
{
    if (!len) {
        return 0.0;
    }

    float sum = 0;

    for (int i = 0; i < len; i++) {
        sum += a[i];
    }

    return sum / len;
}

float sum(const float *a, size_t len)
{
    float sum = 0;

    for (int i = 0; i < len; i++) {
        sum += a[i];
    }

    return sum;

}

int cmp_float(const void *a, const void *b)
{
    return *(float *)a > *(float *)b ? 1 : -1;
}

float trimmean(const float *array, size_t len, float percent)
{
    float trimmean_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);
    qsort(tmp, len, sizeof(float), cmp_float);
    trimmean_data = avg(tmp, (int)(len * (1 - percent) + 0.5));

    RADAR_FREE(tmp);

    return trimmean_data;
}

float max(const float *array, size_t len, float percent)
{
    float max_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);
    qsort(tmp, len, sizeof(float), cmp_float);
    max_data = tmp[len - (int)(len * percent / 2 + 0.5)];

    RADAR_FREE(tmp);
    return max_data;
}

float min(const float *array, size_t len, float percent)
{
    float min_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);
    qsort(tmp, len, sizeof(float), cmp_float);
    min_data = tmp[(int)(len * percent / 2 + 0.5)];

    RADAR_FREE(tmp);
    return min_data;
}

float cov(const float *x, const float *y, size_t len)
{
    float aver_x, aver_y, sum_x = 0, sum_y = 0;
    float sum = 0;

    for (int i = 0; i < len; i++) {
        sum_x += x[i];
        sum_y += y[i];
    }

    aver_x = sum_x / len;
    aver_y = sum_y / len;

    for (int i = 0; i < len; i++) {
        sum += (x[i] - aver_x) * (y[i] - aver_y);
    }

    return sum / (len - 1);
}

float std(const float *a, size_t len)
{
    float avg_data = avg(a, len);
    float pow_sum  = 0;

    for (int i = 0; i < len; i++) {
        pow_sum += powf(a[i] - avg_data, 2);
    }

    return sqrtf(pow_sum / len);
}

float dis(const float *a, const float *b, size_t len)
{
    float pow_sum[2]  = {0};

    for (int i = 0; i < len; i++) {
        pow_sum[0] += powf((a[i] - b[i]), 2);
        pow_sum[1] += powf((a[i] + b[i]), 2);
    }

    return sqrtf(MIN(pow_sum[0], pow_sum[1])) / len;
}

float median(const float *a, size_t len)
{
    float median_data = 0;
    float *tmp        = RADAR_MALLOC_RETRY(sizeof(float) * len);
    memcpy(tmp, a, sizeof(float) * len);

    qsort(tmp, len, sizeof(float), cmp_float);
    median_data = (len % 2) ? tmp[len / 2] : (tmp[len / 2 - 1] + tmp[len % 2]) / 2;

    RADAR_FREE(tmp);

    return median_data;
}

float trimstd(const float *array, size_t len, float percent)
{
    float trimstd_data = 0;
    float *tmp = RADAR_MALLOC_RETRY(sizeof(float) * len);

    memcpy(tmp, array, sizeof(float) * len);
    qsort(tmp, len, sizeof(float), cmp_float);
    trimstd_data = std(tmp, (int)(len * (1 - percent) + 0.5));

    RADAR_FREE(tmp);

    return trimstd_data;
}
