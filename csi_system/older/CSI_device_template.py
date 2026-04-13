import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import struct
import numpy as np
from abc import ABC, abstractmethod

# 全局 Tab 开关状态字典 (保留模板接口)
tab_enabled = {}

# ==============================================================================
# Tab 模板接口 (保留以便后续扩展)
# ==============================================================================
class DisplayTab(ABC):
    """显示标签页的基类模板"""
    def __init__(self, notebook, title):
        self.notebook = notebook
        self.title = title
        self.frame = ttk.Frame(notebook)
        self.setup_ui()
        # 注意：当前主程序已移除 Notebook，此基类仅供后续扩展使用
        # notebook.add(self.frame, text=title) 

    @abstractmethod
    def setup_ui(self):
        """设置 UI 界面"""
        pass

    @abstractmethod
    def update_data(self, full_data):
        """更新数据显示，full_data dict {antenna_id: data}"""
        pass

    def clear_data(self):
        pass

    def refresh_if_needed(self):
        pass

class TabManager:
    """标签页管理器模板"""
    def __init__(self, notebook, processor):
        self.notebook = notebook
        self.tabs = {}
        self.processor = processor
        self.gui_ref = None
    
    def create_basic_tabs(self):
        """在此处添加新的 Tab 页"""
        # 示例：
        # new_tab = MyCustomTab(self.notebook, "我的算法")
        # self.tabs["my_algo"] = new_tab
        # tab_enabled["my_algo"] = False
        pass

    def update_all_tabs(self, full_data):
        """更新所有启用的标签页"""
        for tab_name, tab in self.tabs.items():
            if tab_enabled.get(tab_name, False):
                tab.update_data(full_data)

    def clear_all_tabs(self):
        for tab in self.tabs.values(): 
            if hasattr(tab, 'clear_data'):
                tab.clear_data()

    def set_tab_enabled(self, tab_name, enabled):
        tab_enabled[tab_name] = enabled
        if enabled and tab_name in self.tabs:
            tab = self.tabs[tab_name]
            if hasattr(tab, 'refresh_if_needed'):
                tab.refresh_if_needed()

# ==============================================================================
# 核心数据处理类
# ==============================================================================
class CSIDataProcessor():
    # ====== 配置参数 ======
    timestamp_scale = 10000
    ENABLE_CALIBRATION = True   # 是否启用校准
    # =============================

    def __init__(self):
        # 天线映射 (索引 -> ID)
        self.antenna_mapping = {
            7: "00", 4: "01", 5: "02", 6: "03",
            0: "10", 1: "11", 2: "12", 3: "13"
        }
        self.expected_antennas = set(self.antenna_mapping.values())
        self.antenna_data = {}
        self.last_coarse_timestamp = None 
        # 固定天线顺序
        self.antenna_order = sorted(self.expected_antennas)
        self.M = len(self.antenna_order)

        # 校准相关参数
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None  # 校准后的参考数据  
        self.h_ref_origin = None  # 原始参考数据
        self.lltf_frequencys = None
        self.ht40_frequencys = None
        self.ht40_center_freq = None
        self.lltf_center_freq = None
        self.wifi_protocol = 'LLTF'

    def calculate_pcb_phase_offsets(self, channel_primary: int, channel_secondary: int):
        """计算 PCB 相位偏移"""
        epsilon_r = 4.2  # FR4 介电常数
        h = 8.28  # 介质厚度 (mil)
        w = 13.8  # 线宽 (mil)
        ch1_frequency = 2.412e9  # WiFi 信道 1 频率
        ch_gap = 5.0e6  # WiFi 信道频率间隔
        subcarriar_gap = 312.5e3  # 子载波频率间隔
        mil_to_meter = 25.4e-6  # mil 转 m 系数
        c = 299792458  # 真空光速
        
        # 天线路径长度差异 (mil)
        pcb_path_lengths = np.array([
            3931.8, 1466.0, 1459.2, 3926.5,
            3916.1, 1457.7, 1471.8, 3918.1
        ]) 
        
        epsilon_eff = (epsilon_r + 1)/2 + (epsilon_r - 1)/2 * (1 + 12*h/w)**(-0.5)
        v_group = c / epsilon_eff**0.5
        
        # LLTF 频率
        self.lltf_center_freq = ch1_frequency + ch_gap * (channel_primary - 1)
        self.lltf_frequencys = self.lltf_center_freq + np.arange(-32, 32) * subcarriar_gap
        
        # HT40 频率
        center_primary = ch1_frequency + ch_gap * (channel_primary - 1)
        center_secondary = ch1_frequency + ch_gap * (channel_primary + 4*channel_secondary - 1)
        self.ht40_center_freq = (center_primary + center_secondary) / 2
        self.ht40_frequencys = self.ht40_center_freq + np.arange(-64, 64) * subcarriar_gap
        
        # 波长组
        lltf_wavelengths = v_group / self.lltf_frequencys
        ht40_wavelengths = v_group / self.ht40_frequencys
        
        # 计算 PCB 走线导致的相位偏移
        tracelengths = pcb_path_lengths * mil_to_meter
        self.prop_calib_each_board_lltf = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / lltf_wavelengths[np.newaxis, :])
        self.prop_calib_each_board_ht40 = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / ht40_wavelengths[np.newaxis, :])

    def set_calibration_reference(self, h_ref_origin):
        """设置校准参考数据"""
        self.h_ref_origin = h_ref_origin
        self.h_ref = {}
        antenna_order = ['00', '01', '02', '03', '10', '11', '12', '13']
        
        if self.wifi_protocol == 'LLTF':
            pcb_offsets = self.prop_calib_each_board_lltf
        elif self.wifi_protocol == 'HT40':
            pcb_offsets = self.prop_calib_each_board_ht40
        else:
            pcb_offsets = None

        for antenna_id, ref_data in h_ref_origin.items():
            row_idx = antenna_order.index(antenna_id)
            csi_data = ref_data.get('csi_complex')
            
            if pcb_offsets is not None:
                pcb_offset_for_antenna = pcb_offsets[row_idx, :]
                csi_ref = csi_data * np.conj(pcb_offset_for_antenna)
            else:
                csi_ref = csi_data
                
            self.h_ref[antenna_id] = {
                'csi_complex': csi_ref,
                'timestamp': ref_data.get('timestamp'),
            }

    def clear_calibration_reference(self):
        """清除校准参考数据"""
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None
        self.h_ref_origin = None 
        self.lltf_frequencys = None
        self.ht40_frequencys = None
        self.ht40_center_freq = None
        self.lltf_center_freq = None

    def _calculate_calibrated_data(self, full_data):
        """计算校准后的 CSI 数据"""
        if not self.h_ref:
            return full_data
            
        delays = [full_data[aid]['timestamp'] - self.h_ref[aid]['timestamp'] for aid in full_data]
        mean_delay = np.mean(delays)
        
        freq_offsets = (self.lltf_frequencys - self.lltf_center_freq) if self.wifi_protocol == 'LLTF' else (self.ht40_frequencys - self.ht40_center_freq)
        
        for i, aid in enumerate(full_data):
            relative_delay = delays[i] - mean_delay
            sto_factor = np.exp(-1.0j * 2 * np.pi * relative_delay * freq_offsets)
            # 这里简化处理，仅应用 PCB 补偿，如需 STO 补偿可取消注释下一行
            # full_data[aid]['csi_complex'] = full_data[aid]['csi_complex'] * sto_factor * np.conj(self.h_ref[aid]['csi_complex'])
            full_data[aid]['csi_complex'] = full_data[aid]['csi_complex'] * np.conj(self.h_ref[aid]['csi_complex'])
            full_data[aid]['csi_phase'] = np.angle(full_data[aid]['csi_complex'])
        return full_data

    def run_algorithms(self, full_data):
        """
        [算法接口模板]
        在此处添加自定义算法（如 MUSIC, TDOA, 滤波等）
        输入：full_data (包含 8 根天线对齐后的数据)
        输出：full_data (可添加算法结果字段)
        """
        # TODO: 在此处调用具体算法函数
        # self._calculate_aoa_1d(full_data)
        # self._estimate_tdoa(full_data)
        
        return full_data

    def process(self, raw_data):
        """
        核心处理流程：收集 -> 对齐 -> 校准 -> 算法接口
        """
        antenna_index = raw_data['antenna_index']
        antenna_id = self.antenna_mapping.get(antenna_index, f"未知{antenna_index}")
        
        # 计算粗略时间戳
        coarse_timestamp = (raw_data['timestamp'] + self.timestamp_scale/2) // self.timestamp_scale
        
        # 如果是新时间戳（新帧），清空旧缓存
        if self.last_coarse_timestamp is not None and coarse_timestamp != self.last_coarse_timestamp:
            self.antenna_data.clear()
        self.last_coarse_timestamp = coarse_timestamp

        # 转换 CSI 为复数
        csi_int8 = np.array(raw_data['csi_data'], dtype=np.int8)
        csi_complex = csi_int8[0::2] + 1j * csi_int8[1::2]  
        csi_complex = csi_complex.astype(np.complex64)       

        # 重新排列子载波 (根据协议)
        if self.wifi_protocol == 'HT40':
            part1 = csi_complex[64:128]
            part2 = csi_complex[128:192]
            csi_complex = np.concatenate([part2, part1])
        else:
            part1 = csi_complex[0:32]
            part2 = csi_complex[32:64]
            csi_complex = np.concatenate([part2, part1])

        # 构建处理后的数据项
        processed = {
            'antenna_id': antenna_id,
            'timestamp': raw_data['timestamp'],
            'coarse_timestamp': coarse_timestamp,
            'mac': raw_data.get('mac', 'N/A'),
            'rssi': raw_data['rssi'],
            'noise_floor': raw_data['noise_floor'],
            'channel': raw_data['channel'],
            'csi_complex': csi_complex,
            'csi_magnitude': np.abs(csi_complex),
            'csi_phase': np.angle(csi_complex),
            'subcarriers_nums': len(csi_complex)
        }
        self.antenna_data[antenna_id] = processed
        
        # 检查是否 8 个天线收齐
        if set(self.antenna_data.keys()) != self.expected_antennas:
            return None
        
        # 检查：所有天线 coarse_timestamp 必须完全一致
        coarse_timestamps = {data['coarse_timestamp'] for data in self.antenna_data.values()}
        if len(coarse_timestamps) != 1:
            self.antenna_data.clear()
            self.last_coarse_timestamp = None
            return None

        full_data = dict(self.antenna_data)
        self.antenna_data.clear()
        self.last_coarse_timestamp = None

        # === 校准 ===
        if self.ENABLE_CALIBRATION and self.h_ref is not None:
            full_data = self._calculate_calibrated_data(full_data)

        # === 算法接口 ===
        full_data = self.run_algorithms(full_data)

        return full_data

    def clear_antenna_data(self):
        self.antenna_data.clear()
        self.last_coarse_timestamp = None

# ==============================================================================
# 数据接收类
# ==============================================================================
class CSIDataReceiver:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.receiving_data = False
        self.processor = CSIDataProcessor()
        self.recv_buffer = b""

    def connect(self, host, port):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((host, port))
            self.socket.settimeout(0.1)
            self.connected = True
            self.receiving_data = True
            return True
        except Exception as e:
            messagebox.showerror("连接错误", f"无法连接到设备：{str(e)}")
            return False

    def disconnect(self):
        self.connected = False
        self.receiving_data = False
        if self.socket:
            self.socket.close()
            self.socket = None 

    def send_command(self, command_data):
        if not self.connected or not self.socket:
            return False
        try:
            self.socket.send(command_data)
            return True
        except Exception as e:
            messagebox.showerror("发送错误", f"发送命令失败：{str(e)}")
            return False

    def receive_data_loop(self, full_set_callback):
        while self.connected:
            try:
                chunk = self.socket.recv(1024)
                if not chunk: 
                    break
                self.recv_buffer += chunk

                while len(self.recv_buffer) >= 420:
                    packet = self.recv_buffer[:420]
                    self.recv_buffer = self.recv_buffer[420:]
                    parsed = self.parse_csi_packet(packet)
                    if parsed is not None:
                        full_data = self.processor.process(parsed)
                        if full_data is not None:
                            full_set_callback(full_data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    def parse_csi_packet(self, data):
        try:
            sequence_num = data[0]
            csi_data = data[1:420]

            timestamp = struct.unpack('<I', csi_data[0:4])[0]
            mac = csi_data[4:10]
            dmac = csi_data[10:16]
            rssi = struct.unpack('<b', csi_data[16:17])[0]
            noise_floor = struct.unpack('<b', csi_data[17:18])[0]
            rate = csi_data[18]
            sgi = csi_data[19]
            sig_mode = csi_data[20]
            mcs = csi_data[21]
            cwb = csi_data[22]
            channel = csi_data[23]
            secondary_channel = csi_data[24]
            rx_state = csi_data[25]
            fft_gain = csi_data[26]
            agc_gain = csi_data[27]
            gain = struct.unpack('<f', csi_data[28:32])[0]
            first_word_invalid = csi_data[32]
            csi_len = struct.unpack('<H', csi_data[33:35])[0]
            csi_buffer = csi_data[35:35+384]
            
            return {
                'antenna_index': sequence_num,
                'timestamp': timestamp,
                'rssi': rssi,
                'noise_floor': noise_floor,
                'mac': ':'.join(f'{b:02x}' for b in mac),
                'dmac': ':'.join(f'{b:02x}' for b in dmac),
                'rate': rate,
                'sgi': sgi,
                'sig_mode': sig_mode,
                'mcs': mcs,
                'cwb': cwb,
                'channel': channel,
                'secondary_channel': -1 if secondary_channel==2 else secondary_channel,
                'rx_state': rx_state,
                'fft_gain': fft_gain,
                'agc_gain': agc_gain,
                'gain': gain,
                'first_word_invalid': first_word_invalid,
                'csi_len': csi_len,
                'csi_data': [b if b < 128 else b - 256 for b in csi_buffer],
            }
        except Exception as e:
            print(f"Parse error: {e}")
            return None

# ==============================================================================
# GUI 界面类
# ==============================================================================
class CSIDataGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("CSI 数据采集系统 (精简版)")
        self.root.geometry("1000x700")
        self.setup_style()
        self.received_packets = 0
        self.receiver = CSIDataReceiver()
        self._last_full_data = None
        self.setup_ui()

    def setup_style(self):
        style = ttk.Style()
        style.theme_use('clam')
        self.colors = {
            "primary": "#2c3e50",
            "secondary": "#3498db", 
            "success": "#27ae60",
            "warning": "#e67e22",
            "danger": "#e74c3c",
            "light_bg": "#ecf0f1"
        }
        style.configure("TButton", font=("微软雅黑", 9))
        style.configure("TEntry", font=("微软雅黑", 9))
        style.configure("TNotebook", background=self.colors["light_bg"])
        style.configure("TNotebook.Tab", font=("微软雅黑", 9))

        for color_name, color_value in [("Primary", self.colors["secondary"]),
                                       ("Success", self.colors["success"]),
                                       ("Warning", self.colors["warning"]),
                                       ("Danger", self.colors["danger"])]:
            style.configure(f"{color_name}.TButton", background=color_value, foreground="white")

    def setup_ui(self):
        self.root.configure(bg=self.colors["light_bg"])
        
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # 1. 控制面板 (完全保留原逻辑)
        control_frame = ttk.LabelFrame(main_paned, text="控制面板", padding="8")
        main_paned.add(control_frame, weight=0)
        self.setup_control_panel(control_frame)

        # 2. 数据显示面板 (移除 Notebook，保留框架)
        data_frame = ttk.LabelFrame(main_paned, text="数据监控", padding="8")
        main_paned.add(data_frame, weight=1)
        self.setup_data_panel(data_frame)

    def setup_control_panel(self, parent):
        # 第一行：连接控制
        conn_frame = ttk.Frame(parent)
        conn_frame.pack(fill=tk.X, pady=2)

        ttk.Label(conn_frame, text="IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry = ttk.Entry(conn_frame, width=12)
        self.ip_entry.insert(0, "192.168.2.10")
        self.ip_entry.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(conn_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry = ttk.Entry(conn_frame, width=8)
        self.port_entry.insert(0, "8000")
        self.port_entry.pack(side=tk.LEFT, padx=(0, 15))

        self.connect_btn = ttk.Button(conn_frame, text="连接", 
                                    command=self.toggle_connection, style="Primary.TButton")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))

        self.clear_btn = ttk.Button(conn_frame, text="清除数据", 
                                  command=self.clear_all_data, style="Warning.TButton")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.clear_btn.state(['disabled'])

        ttk.Label(conn_frame, text="数据包:").pack(side=tk.LEFT, padx=(0, 5))
        self.packet_count_label = ttk.Label(conn_frame, text="0", 
                                          foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.packet_count_label.pack(side=tk.LEFT, padx=(0, 15))

        self.status_label = ttk.Label(conn_frame, text="未连接", foreground="red", 
                                    font=("微软雅黑", 9, "bold"))
        self.status_label.pack(side=tk.LEFT)

        # 第二行：MAC 与 WiFi 设置
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, pady=5)

        mac_frame = ttk.Frame(control_frame)
        mac_frame.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(mac_frame, text="MAC 地址:").pack(side=tk.LEFT, padx=(0, 5))
        self.mac_entries = []
        default_mac = ["00", "55", "66", "77", "88", "88"]
        for i in range(6):
            entry = ttk.Entry(mac_frame, width=3, justify='center', font=("Consolas", 8))
            entry.insert(0, default_mac[i])
            entry.pack(side=tk.LEFT, padx=1)
            self.mac_entries.append(entry)
            if i < 5:
                ttk.Label(mac_frame, text=":").pack(side=tk.LEFT)

        ttk.Button(control_frame, text="普通模式", 
                  command=self.set_normal_mode, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(control_frame, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_var = tk.StringVar(value="1")
        channel_combo = ttk.Combobox(control_frame, textvariable=self.channel_var,
                                   values=[str(i) for i in range(1, 14)], width=3, state="readonly")
        channel_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(control_frame, text="带宽:").pack(side=tk.LEFT, padx=(0, 5))
        self.bandwidth_var = tk.StringVar(value="20M")
        bandwidth_combo = ttk.Combobox(control_frame, textvariable=self.bandwidth_var,
                                   values=["20M", "40M"], width=4, state="readonly")
        bandwidth_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(control_frame, text="设置 wifi", 
                  command=self.set_wifi, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        # 第三行：校准控制
        calib_frame = ttk.Frame(parent)
        calib_frame.pack(fill=tk.X, pady=5)
        
        ttk.Button(calib_frame, text="校准模式", 
                  command=self.set_calibration_mode, style="Success.TButton").pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(calib_frame, text="开启校准 wifi", 
                  command=self.send_wifion_frame, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(calib_frame, text="关闭校准 wifi", 
                  command=self.send_wifioff_frame, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(calib_frame, text="应用校准", 
                  command=self.apply_calibration, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(calib_frame, text="清除校准", 
                  command=self.clear_h_ref, style="Warning.TButton").pack(side=tk.LEFT)
        
        ttk.Label(calib_frame, text="协议:").pack(side=tk.LEFT, padx=(15, 5))
        self.wifi_protocol_var = tk.StringVar(value="LLTF")
        wifi_protocol_combo = ttk.Combobox(calib_frame, textvariable=self.wifi_protocol_var,
                                   values=["LLTF", "HT40"], width=6, state="readonly")
        wifi_protocol_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(calib_frame, text="设置协议", 
             command=self.set_protocol, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        # 状态栏
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, pady=2)

        ttk.Label(status_frame, text="模式:").pack(side=tk.LEFT, padx=(0, 5))
        self.mode_label = ttk.Label(status_frame, text="普通模式", 
                                  foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mode_label.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(status_frame, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_label = ttk.Label(status_frame, text="1", 
                                     foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.channel_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_frame, text="过滤 MAC:").pack(side=tk.LEFT, padx=(0, 5))
        self.mac_label = ttk.Label(status_frame, text="00:00:00:00:00:00", 
                                 foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mac_label.pack(side=tk.LEFT)

        # === 标签页显示开关 (保留模板接口) ===
        tab_switch_frame = ttk.Frame(parent)
        tab_switch_frame.pack(fill=tk.X)

        self.tab_checkboxes = {}
        tab_names = [
            ("multi_antenna", "多天线监控"),

        ]
        for tab_key, tab_label in tab_names:
            var = tk.BooleanVar(value=False)
            cb = ttk.Checkbutton(
                tab_switch_frame,
                text=tab_label,
                variable=var,
                command=lambda key=tab_key, v=var: self.on_tab_toggle(key, v)
            )
            cb.pack(side=tk.LEFT, padx=10)
            self.tab_checkboxes[tab_key] = var

    def setup_data_panel(self, parent):
        # 移除 Notebook，仅保留空白框架作为占位
        # 您后续可在此处重新添加 Notebook 或自定义显示组件
        placeholder_label = ttk.Label(parent, text="数据已就绪 (请在代码中添加显示组件)", foreground="gray")
        placeholder_label.pack(expand=True)
        
        # 保留 TabManager 实例化以便后续扩展
        self.tab_manager = TabManager(None, self.receiver.processor)
        self.tab_manager.gui_ref = self
        # 当前不创建任何 Tab 页
        # self.tab_manager.create_basic_tabs()

    def toggle_connection(self):
        if not self.receiver.connected:
            self.connect_to_device()
        else:
            self.disconnect_from_device()

    def connect_to_device(self):
        ip = self.ip_entry.get()
        try:
            port = int(self.port_entry.get())
        except ValueError:
            messagebox.showerror("错误", "端口号必须是数字")
            return

        if self.receiver.connect(ip, port):
            self.connect_btn.config(text="断开连接", style="Danger.TButton")
            self.clear_btn.state(['!disabled'])
            self.status_label.config(text="已连接", foreground=self.colors["success"])

            self.receive_thread = threading.Thread(
                target=self.receiver.receive_data_loop,
                args=(self.on_full_antenna_set,),
                daemon=True
            )
            self.receive_thread.start()

    def disconnect_from_device(self):
        self.receiver.disconnect()
        self.connect_btn.config(text="连接", style="Primary.TButton")
        self.clear_btn.state(['disabled'])
        self.status_label.config(text="未连接", foreground="red")

    def clear_all_data(self):
        if hasattr(self, 'tab_manager'):
            self.tab_manager.clear_all_tabs()
        self.receiver.processor.clear_antenna_data()
        self.received_packets = 0
        self.packet_count_label.config(text="0")

    def set_normal_mode(self):
        try:
            mac_bytes = []
            for entry in self.mac_entries:
                value = entry.get().strip()
                if not value:
                    messagebox.showerror("错误", "请填写完整的 MAC 地址")
                    return
                mac_bytes.append(int(value, 16))

            command = bytes([0x01] + mac_bytes)
            if self.receiver.send_command(command):
                self.mode_label.config(text="普通模式")
                self.mac_label.config(text=':'.join(f'{x:02X}' for x in mac_bytes))
        except ValueError:
            messagebox.showerror("错误", "MAC 地址必须是十六进制数字")

    def set_wifi(self):
        try:
            channel = int(self.channel_var.get())
            if not 1 <= channel <= 13:
                messagebox.showerror("错误", "信道必须在 1-13 之间")
                return
            bandwidth = 1 if self.bandwidth_var.get() == "20M" else 2
            command = bytes([0x03, channel, bandwidth])
            if self.receiver.send_command(command):
                self.channel_label.config(text=str(channel))
        except ValueError:
            messagebox.showerror("错误", "请选择有效的信道")

    def set_protocol(self):
        self.receiver.processor.wifi_protocol = self.wifi_protocol_var.get()
        messagebox.showinfo("提示", f"协议已设置为 {self.wifi_protocol_var.get()}")

    def set_calibration_mode(self):
        command = bytes([0x02] + [0x00] * 6)
        if self.receiver.send_command(command):
            self.mode_label.config(text="校准模式")
            self.mac_label.config(text="00:11:22:33:44:55")

    def send_wifion_frame(self):
        command = bytes([0xFF])
        if self.receiver.send_command(command):
            messagebox.showinfo("成功", "校准 wifi 已开启")

    def send_wifioff_frame(self):
        command = bytes([0xFE])
        if self.receiver.send_command(command):
            messagebox.showinfo("成功", "校准 wifi 已关闭")

    def apply_calibration(self):
        """将最近一次 full_data 作为新的校准参考"""
        if self._last_full_data is not None:
            h_ref_origin = {
                aid: {
                    'csi_complex': data['csi_complex'].copy(),
                    'timestamp': data.get('timestamp', None)
                }
                for aid, data in self._last_full_data.items()
            }
            # 获取信道信息用于计算 PCB 偏移
            ch = self._last_full_data['00']['channel']
            sec_ch = self._last_full_data['00'].get('secondary_channel', 0)
            
            self.receiver.processor.calculate_pcb_phase_offsets(ch, sec_ch)
            self.receiver.processor.set_calibration_reference(h_ref_origin)
            messagebox.showinfo("校准", "校准数据已更新！")
        else:
            messagebox.showwarning("校准", "尚无可用数据！请先接收完整数据包。")

    def clear_h_ref(self):
        self.receiver.processor.clear_calibration_reference()
        messagebox.showinfo("校准", "校准数据已清除。")

    def on_full_antenna_set(self, full_data):
        """收齐 8 个天线后，更新 UI"""
        self.root.after(0, self.update_ui_with_full_set, full_data)

    def update_ui_with_full_set(self, full_data):
        self.received_packets += 1
        self.packet_count_label.config(text=str(self.received_packets))
        self._last_full_data = full_data
        
        # 更新 Tab 
        if hasattr(self, 'tab_manager'):
            self.tab_manager.update_all_tabs(full_data)

    def on_tab_toggle(self, tab_name, var):
        """标签页开关回调"""
        enabled = var.get()
        if hasattr(self, 'tab_manager'):
            self.tab_manager.set_tab_enabled(tab_name, enabled)

def main():
    root = tk.Tk()
    app = CSIDataGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()