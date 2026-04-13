# ESP 雷达运动检测 [[English]](./README.md)

### 概述

`esp_radar_motion_dec` 是用于 ESP-CSI 雷达系统的运动检测算法组件。它提供信号处理算法，用于基于信道状态信息（CSI）数据分析和检测运动。

### 功能特性

- **特征提取**：从多段 CSI 数据计算运动特征，提取与运动相关的特性
- **波形漂移计算**：通过计算信号向量之间的相似度来测量信号稳定性
- **波形抖动计算**：通过分析滑动窗口内的波形变化来评估运动强度

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

#### `esp_radar_motion_dec_compute()`

从两个数据段计算运动特征。

```c
esp_err_t esp_radar_motion_dec_compute(uint32_t cols,
                                       uint32_t row_0, const float data_0[row_0][cols],
                                       uint32_t row_1, const float data_1[row_1][cols],
                                       float output[cols]);
```

**参数：**
- `cols`: 子载波数量（列数）
- `row_0`: 第一段数据的行数
- `data_0`: 第一段数据 `[row_0][cols]`
- `row_1`: 第二段数据的行数
- `data_1`: 第二段数据 `[row_1][cols]`
- `output`: 输出结果 `[cols]`

**返回值：**
- `ESP_OK` 成功
- `ESP_ERR_NO_MEM` 内存分配失败
- `ESP_FAIL` 计算未收敛

#### `esp_radar_motion_dec_wander_compute()`

计算波形漂移（wander）以测量信号稳定性。

```c
float esp_radar_motion_dec_wander_compute(const float *a, const float *b, size_t len);
```

**参数：**
- `a`: 向量 A
- `b`: 向量 B
- `len`: 向量长度

**返回值：**
- 波形漂移值，范围 `[0.0, 1.0]`
- 值越小表示越相似（越稳定）

#### `esp_radar_motion_dec_compute_jitter()`

计算波形抖动（jitter）以评估运动强度。

```c
float esp_radar_motion_dec_compute_jitter(float *current,
                                          float **history,
                                          uint16_t cols,
                                          uint8_t window_size,
                                          uint32_t buff_num);
```

**参数：**
- `current`: 当前结果 `[cols]`
- `history`: 历史数据缓冲区 `[window_size][cols]`
- `cols`: 子载波数量
- `window_size`: 滑动窗口大小
- `buff_num`: 当前缓冲区计数（用于索引历史数据）

**返回值：**
- 抖动值，范围 `[0.0, 1.0]`
- 值越小表示越稳定 
- 数据不足时返回 `0.0`

### 使用示例

Test_app
