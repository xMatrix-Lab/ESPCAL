# ESPCAL : ESP32 Collaborative Antenna Localization
# [[ENGLISH]](./README_EN.md)
**基于ESP32的WiFi CSI感知平台，实现信道状态信息采集、相位校准、阵列信号处理与角度估计，为室内定位、无线感知等研究提供轻量级开源验证平台。**
> [!TIP]
> 此分支为早期验证性实验，IDEA来自于参考项目：[ESPARGOS](https://espargos.net/)，基于其方案搭建了完整硬件平台与数据获取框架、应用实例。

## 目录
- [硬件](#硬件)

## 展示
<img width="450" height="1000" alt="IMG_20260312_144305" src="https://github.com/user-attachments/assets/3ab25cad-7ec4-47e6-ac26-f5823efc4670" />
<img width="1000" height="450" alt="IMG_20260413_145527" src="https://github.com/user-attachments/assets/3f7fd167-e252-4eb2-9859-9d4b2efa1a1a" />
<img width="1000" height="450" alt="IMG_20260413_145511" src="https://github.com/user-attachments/assets/737251ab-a97b-459e-9c68-8310bc27b36a" />
<img src="meterials/G1.gif" width="80%">




## 硬件
此项目硬件使用嘉立创EDA设计绘制，并在立创开源广场进行完全开源：[硬件开源](https://oshwhub.com/greentor/project_ytsbudsf/)

主控：ESP32-S3FH4R2

<img width="1000" height="450" alt="IMG_20251107_201829" src="https://github.com/user-attachments/assets/f9df0ec6-053f-49f7-8784-4047274c0312" />
<img width="1000" height="450" alt="IMG_20251107_201957" src="https://github.com/user-attachments/assets/31af9f2d-d2cc-4ef5-b1f3-f83dc629be80" />

## 软件
drivers文件夹为ESP32S3的源代码，其中CSI_SLAVE为各天线控制芯片的程序，CSI_MASTER为控制板上ESP32的程序

csi_system为上位机程序，用于接收硬件平台的数据并进行数据处理、算法应用。使用Python 3.11.4。
目录结构如下：
```
📁 csi_system/
├── 📁 algorithms/          算法类实现目录
├── 📁 core/                核心框架类
├── 📁 tabs/                功能标签页类
├── 📁 older/               旧版本代码
└── 📄 main_XXX.py          主程序
```
使用到的库：tkinter、socket、threading、numpy、scipy、numba等等

## 环境
<img width="800" height="550" alt="image" src="https://github.com/user-attachments/assets/3671c19b-4160-4d6c-999f-0610cb1ce7a6" />

| 距离 | 信号强度 | 精度 | 信噪  | 描述 |
|------|----------|--------|-------|----------------|
| < 2m | 强 | 高 | 大 | 推荐 |
| 2-5m | 中 | 中 | 中 | 推荐 |
| > 5m | 弱 | 低 | 小 | 难以正确测量DOA |

## 快速开始
1. 使用USB或DC口给设备5V供电，阵列耗电量较大，需要注意满足电源电流足够，确保所有芯片同时上电
2. 为每一个芯片烧录固件，天线单元的ESP32需要使用串口工具进行烧录，控制板内置串口芯片，上面的的ESP32可以直接使用USB接入进行烧录
3. 使用网线连接控制板和上位机，默认地址为192.168.2.10:8000
4. 运行main_XXX.py程序，点击连接设备
<img width="1000" height="600" alt="767804e98fa9837348d18c5499821fa9" src="https://github.com/user-attachments/assets/6178f1f6-6f53-41c1-acd1-d3c192281296" />
5. 连接成功后，点击校准模式，再点击开启校准WiFi。等待设备进行校准，当接收到30+个校准数据包后，关闭校准WiFi，点击应用校准
6. 校准完毕后，输入需要定位的信源MAC地址，点击普通模式，即可开始正常运行



