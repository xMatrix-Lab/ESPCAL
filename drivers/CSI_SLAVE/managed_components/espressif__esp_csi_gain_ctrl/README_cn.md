# ESP CSI 增益控制 [[English]](./README.md)

### 概述

`esp_csi_gain_ctrl` 是用于 ESP-CSI 雷达系统的接收增益控制组件。它提供接收增益管理功能，用于稳定 CSI 数据采集和处理，通过补偿自动增益控制（AGC）和 FFT 增益变化来确保数据一致性。

### 功能特性

- **增益基线计算**：收集多个增益样本并计算基线
- **增益补偿**：根据当前增益与基线的差异计算补偿因子，用于校正 CSI 数据
- **强制增益设置**：支持手动设置接收增益（可能影响数据包传输）
- **增益提取**：从 CSI 数据包信息中提取 AGC 和 FFT 增益值

### 支持的芯片

- ESP32
- ESP32-S2
- ESP32-S3
- ESP32-C3
- ESP32-C5
- ESP32-C6
- ESP32-C61

### 依赖要求

- ESP-IDF >= 4.4.1

### API 参考

#### `esp_csi_gain_ctrl_get_gain_status()`

检查接收增益状态。

```c
rx_gain_status_t esp_csi_gain_ctrl_get_gain_status(void);
```

**返回值：**
- `RX_GAIN_COLLECT` 正在收集数据
- `RX_GAIN_READY` 基线已计算完成
- `RX_GAIN_FORCE` 增益已被手动强制设置

#### `esp_csi_gain_ctrl_record_rx_gain()`

记录单个接收增益样本，用于后续基线计算。

```c
esp_err_t esp_csi_gain_ctrl_record_rx_gain(uint8_t agc_gain, int8_t fft_gain);
```

**参数：**
- `agc_gain`: 当前 AGC 增益
- `fft_gain`: 当前 FFT 增益

**返回值：**
- `ESP_OK` 成功

#### `esp_csi_gain_ctrl_get_rx_gain_baseline()`

获取接收增益基线值。

```c
esp_err_t esp_csi_gain_ctrl_get_rx_gain_baseline(uint8_t *agc_gain, int8_t *fft_gain);
```

**参数：**
- `agc_gain`: 输出基线 AGC 增益
- `fft_gain`: 输出基线 FFT 增益

**返回值：**
- `ESP_OK` 成功
- `ESP_ERR_INVALID_ARG` 参数无效
- `ESP_ERR_INVALID_STATE` 基线尚未准备好
- `ESP_ERR_NO_MEM` 内存分配失败

#### `esp_csi_gain_ctrl_set_rx_force_gain()`

强制设置接收增益（可能导致数据包丢失）。

```c
esp_err_t esp_csi_gain_ctrl_set_rx_force_gain(uint8_t agc_gain, int8_t fft_gain);
```

**参数：**
- `agc_gain`: AGC 增益
- `fft_gain`: FFT 增益

**返回值：**
- `ESP_OK` 成功

**注意：** 如果 `agc_gain` 和 `fft_gain` 都为 0，则禁用强制增益并恢复自动增益。

#### `esp_csi_gain_ctrl_reset_rx_gain_baseline()`

重置接收增益基线统计。

```c
void esp_csi_gain_ctrl_reset_rx_gain_baseline(void);
```

#### `esp_csi_gain_ctrl_get_gain_compensation()`

计算当前接收增益的补偿因子。

```c
esp_err_t esp_csi_gain_ctrl_get_gain_compensation(float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);
```

**参数：**
- `compensate_gain`: 输出计算的补偿因子
- `agc_gain`: 当前 AGC 增益
- `fft_gain`: 当前 FFT 增益

**返回值：**
- `ESP_OK` 成功
- `ESP_ERR_INVALID_STATE` 基线未准备好或其他无效状态

#### `esp_csi_gain_ctrl_compensate_rx_gain()`

扩展版本的接收增益补偿，支持 8 位和 16 位 CSI 样本。

```c
esp_err_t esp_csi_gain_ctrl_compensate_rx_gain(void *data, uint16_t size, bool samples_are_16bit,
                                               float *compensate_gain, uint8_t agc_gain, int8_t fft_gain);
```

**参数：**
- `data`: 待补偿的数据缓冲区（原地修改）
- `size`: 缓冲区长度（字节）
- `samples_are_16bit`: 如果每个 CSI 分量为 16 位则为 `true`，8 位则为 `false`
- `compensate_gain`: 输出补偿因子
- `agc_gain`: 当前 AGC 增益
- `fft_gain`: 当前 FFT 增益

**返回值：**
- `ESP_OK` 成功
- 其他错误码表示失败

#### `esp_csi_gain_ctrl_get_rx_gain()`

从 CSI 数据包信息中提取接收增益。

```c
void esp_csi_gain_ctrl_get_rx_gain(const void *rx_ctrl, uint8_t *agc_gain, int8_t *fft_gain);
```

**参数：**
- `rx_ctrl`: 指向 RX 控制信息的指针（wifi_pkt_rx_ctrl_t）
- `agc_gain`: 输出提取的 AGC 增益
- `fft_gain`: 输出提取的 FFT 增益

### 使用示例

组件内的 test_app
