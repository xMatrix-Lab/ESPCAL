# ESP CSI Gain Control [[中文]](./README_cn.md)

### Overview

`esp_csi_gain_ctrl` is a receive gain control component for ESP-CSI radar systems. It provides receive gain management functionality to stabilize CSI data acquisition and processing, ensuring data consistency by compensating for Automatic Gain Control (AGC) and FFT gain variations.

### Features

- **Gain Baseline Calculation**: Collects multiple gain samples and calculates baseline
- **Gain Compensation**: Calculates compensation factors based on the difference between current gain and baseline, used to correct CSI data
- **Force Gain Setting**: Supports manual setting of receive gain (may affect packet transmission)
- **Gain Extraction**: Extracts AGC and FFT gain values from CSI packet information

### Supported Chips

- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C3
- ESP32-C5
- ESP32-C6
- ESP32-C61

### Requirements

- ESP-IDF >= 4.4.1

### API Reference

#### `esp_csi_gain_ctrl_get_gain_status()`

Check receive gain status.

```c
rx_gain_status_t esp_csi_gain_ctrl_get_gain_status(void);
```

**Returns:**
- `RX_GAIN_COLLECT` Data collection in progress
- `RX_GAIN_READY` Baseline calculation completed
- `RX_GAIN_FORCE` Gain has been manually forced

#### `esp_csi_gain_ctrl_record_rx_gain()`

Record a single receive gain sample for subsequent baseline calculation.

```c
esp_err_t esp_csi_gain_ctrl_record_rx_gain(uint8_t agc_gain, int8_t fft_gain);
```

**Parameters:**
- `agc_gain`: Current AGC gain
- `fft_gain`: Current FFT gain

**Returns:**
- `ESP_OK` Success

#### `esp_csi_gain_ctrl_get_rx_gain_baseline()`

Get receive gain baseline values.

```c
esp_err_t esp_csi_gain_ctrl_get_rx_gain_baseline(uint8_t *agc_gain, int8_t *fft_gain);
```

**Parameters:**
- `agc_gain`: Output baseline AGC gain
- `fft_gain`: Output baseline FFT gain

**Returns:**
- `ESP_OK` Success
- `ESP_ERR_INVALID_ARG` Invalid argument
- `ESP_ERR_INVALID_STATE` Baseline not ready
- `ESP_ERR_NO_MEM` Memory allocation failed

#### `esp_csi_gain_ctrl_set_rx_force_gain()`

Force set receive gain (may cause packet loss).

```c
esp_err_t esp_csi_gain_ctrl_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain);
```

**Parameters:**
- `agc_gain`: AGC gain
- `fft_gain`: FFT gain

**Returns:**
- `ESP_OK` Success

**Note:** If both `agc_gain` and `fft_gain` are 0, force gain is disabled and automatic gain is restored.

#### `esp_csi_gain_ctrl_reset_rx_gain_baseline()`

Reset receive gain baseline statistics.

```c
void esp_csi_gain_ctrl_reset_rx_gain_baseline(void);
```

#### `esp_csi_gain_ctrl_get_gain_compensation()`

Calculate compensation factor for current receive gain.

```c
esp_err_t esp_csi_gain_ctrl_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);
```

**Parameters:**
- `compensate_gain`: Output calculated compensation factor
- `agc_gain`: Current AGC gain
- `fft_gain`: Current FFT gain

**Returns:**
- `ESP_OK` Success
- `ESP_ERR_INVALID_STATE` Baseline not ready or other invalid state

#### `esp_csi_gain_ctrl_compensate_rx_gain()`

Extended version of receive gain compensation, supporting 8-bit and 16-bit CSI samples.

```c
esp_err_t esp_csi_gain_ctrl_compensate_rx_gain(void *data, uint16_t size, bool samples_are_16bit,
                                               float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);
```

**Parameters:**
- `data`: Data buffer to be compensated (modified in-place)
- `size`: Buffer length (bytes)
- `samples_are_16bit`: `true` if each CSI component is 16-bit, `false` if 8-bit
- `compensate_gain`: Output compensation factor
- `agc_gain`: Current AGC gain
- `fft_gain`: Current FFT gain

**Returns:**
- `ESP_OK` Success
- Other error codes indicate failure

#### `esp_csi_gain_ctrl_get_rx_gain()`

Extract receive gain from CSI packet information.

```c
void esp_csi_gain_ctrl_get_rx_gain(const void *rx_ctrl, uint8_t *agc_gain, int8_t *fft_gain);
```

**Parameters:**
- `rx_ctrl`: Pointer to RX control information (wifi_pkt_rx_ctrl_t)
- `agc_gain`: Output extracted AGC gain
- `fft_gain`: Output extracted FFT gain

### Usage Example

See test_app in the component
