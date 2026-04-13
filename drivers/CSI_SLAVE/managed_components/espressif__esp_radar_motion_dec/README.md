# ESP Radar Motion Detection [[中文]](./README_cn.md)

### Overview

`esp_radar_motion_dec` is a motion detection algorithm component for the ESP-CSI radar system. It provides signal processing algorithms for analyzing and detecting motion based on Channel State Information (CSI) data.

### Features

- **Feature Extraction**: Computes motion features from multiple CSI data segments, extracting motion-related characteristics
- **Waveform Wander Calculation**: Measures signal stability by computing similarity between signal vectors
- **Waveform Jitter Calculation**: Evaluates motion intensity by analyzing waveform changes within a sliding window

### Supported Chips

- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C3
- ESP32-C5
- ESP32-C6
- ESP32-C61

### Dependencies

- ESP-IDF >= 5.4
- `esp_common` component

### API Reference

#### `esp_radar_motion_dec_compute()`

Computes motion features from two data segments.

```c
esp_err_t esp_radar_motion_dec_compute(uint32_t cols,
                                       uint32_t row_0, const float data_0[row_0][cols],
                                       uint32_t row_1, const float data_1[row_1][cols],
                                       float output[cols]);
```

**Parameters:**
- `cols`: Number of subcarriers (number of columns)
- `row_0`: Number of rows in the first data segment
- `data_0`: First data segment `[row_0][cols]`
- `row_1`: Number of rows in the second data segment
- `data_1`: Second data segment `[row_1][cols]`
- `output`: Output result `[cols]`

**Returns:**
- `ESP_OK` on success
- `ESP_ERR_NO_MEM` on memory allocation failure
- `ESP_FAIL` if computation did not converge

#### `esp_radar_motion_dec_wander_compute()`

Computes waveform wander to measure signal stability.

```c
float esp_radar_motion_dec_wander_compute(const float *a, const float *b, size_t len);
```

**Parameters:**
- `a`: Vector A
- `b`: Vector B
- `len`: Vector length

**Returns:**
- Waveform wander value, range `[0.0, 1.0]`
- Smaller values indicate greater similarity (more stable)

#### `esp_radar_motion_dec_compute_jitter()`

Computes waveform jitter to evaluate motion intensity.

```c
float esp_radar_motion_dec_compute_jitter(float *current,
                                          float **history,
                                          uint16_t cols,
                                          uint8_t window_size,
                                          uint32_t buff_num);
```

**Parameters:**
- `current`: Current result `[cols]`
- `history`: Historical data buffer `[window_size][cols]`
- `cols`: Number of subcarriers
- `window_size`: Sliding window size
- `buff_num`: Current buffer count (used for indexing historical data)

**Returns:**
- Jitter value, range `[0.0, 1.0]`
- Smaller values indicate greater stability
- Returns `0.0` when data is insufficient

### Usage Examples

Test_app