"""
CSI 数据采集系统 - 核心通用模块 (双节点)
包含：数据处理、数据接收、GUI 框架、Tab 管理
"""
import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import struct
import numpy as np
from abc import ABC, abstractmethod

# ==============================================================================
# 数据处理核心
# ==============================================================================
class CSIDataProcessor:
    """CSI 数据处理类 (支持 16 天线双设备)"""
    timestamp_scale = 1000
    ENABLE_CALIBRATION = True

    def __init__(self):
        self.cache = {}
        # 双设备映射：设备 0(0-7), 设备 1(8-15)
        self.antenna_mapping = {
            6: "00", 3: "01", 14: "02", 11: "03",
            5: "10", 2: "11", 13: "12", 10: "13",
            4: "20", 1: "21", 12: "22", 9: "23",
            7: "30", 0: "31", 15: "32", 8: "33"
        }
        self.expected_antennas = set(self.antenna_mapping.values())
        self.antenna_data = {}
        self.last_coarse_timestamp = None
        self.antenna_order = sorted(self.expected_antennas)
        self.M = len(self.antenna_order)
        self.wifi_protocol = 'LLTF'
        
        # 校准相关
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None
        self.h_ref_origin = None
        self.lltf_frequencys = None
        self.ht40_frequencys = None
        self.ht40_center_freq = None
        self.lltf_center_freq = None
        
        # 算法模块注册
        self.algorithms = {}

    def register_algorithm(self, name, algorithm_instance):
        """注册算法模块"""
        self.algorithms[name] = algorithm_instance

    def calculate_pcb_phase_offsets(self, channel_primary, channel_secondary):
        """计算 PCB 相位偏移"""
        epsilon_r = 4.2
        h = 8.28
        w = 13.8
        ch1_frequency = 2.412e9
        ch_gap = 5.0e6
        subcarriar_gap = 312.5e3
        mil_to_meter = 25.4e-6
        c = 299792458
        
        # 16 根天线走线长度相对差 (需根据实际硬件填写)
        pcb_path_lengths = np.array([
            3926.5, 3918.1, 3926.5, 3918.1,
            1459.2, 1471.8, 1459.2, 1471.8,
            1466.0, 1457.7, 1466.0, 1457.7,
            3931.8, 3916.1, 3931.8, 3916.1
        ])
        
        epsilon_eff = (epsilon_r + 1)/2 + (epsilon_r - 1)/2 * (1 + 12*h/w)**(-0.5)
        v_group = c / epsilon_eff**0.5
        
        self.lltf_center_freq = ch1_frequency + ch_gap * (channel_primary - 1)
        self.lltf_frequencys = self.lltf_center_freq + np.arange(-32, 32) * subcarriar_gap
        
        center_primary = ch1_frequency + ch_gap * (channel_primary - 1)
        center_secondary = ch1_frequency + ch_gap * (channel_primary + 4*channel_secondary - 1)
        self.ht40_center_freq = (center_primary + center_secondary) / 2
        self.ht40_frequencys = self.ht40_center_freq + np.arange(-64, 64) * subcarriar_gap
        
        lltf_wavelengths = v_group / self.lltf_frequencys
        ht40_wavelengths = v_group / self.ht40_frequencys
        
        tracelengths = pcb_path_lengths * mil_to_meter
        self.prop_calib_each_board_lltf = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / lltf_wavelengths[np.newaxis, :])
        self.prop_calib_each_board_ht40 = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / ht40_wavelengths[np.newaxis, :])

    def set_calibration_reference(self, h_ref_origin):
        """设置校准参考数据"""
        self.h_ref_origin = h_ref_origin
        self.h_ref = {}
        
        if self.wifi_protocol == 'LLTF':
            pcb_offsets = self.prop_calib_each_board_lltf
        elif self.wifi_protocol == 'HT40':
            pcb_offsets = self.prop_calib_each_board_ht40
        else:
            pcb_offsets = None
        
        for antenna_id, ref_data in h_ref_origin.items():
            row_idx = self.antenna_order.index(antenna_id)
            csi_data = ref_data.get('csi_complex')
            pcb_offset_for_antenna = pcb_offsets[row_idx, :]
            # 避免除0，自动过滤无效值
            csi_ref = np.zeros_like(csi_data, dtype=np.complex64)
            mask = np.abs(csi_data) > 1e-6
            csi_ref[mask] = pcb_offset_for_antenna[mask] / csi_data[mask]
            
            self.h_ref[antenna_id] = {
                # 'csi_complex': np.exp(-1.0j * np.angle(csi_ref)),# 相当于 PCB偏移/参考信号 后面只需要乘以这个相位补偿即可
                'csi_complex': csi_ref,
                'timestamp': ref_data.get('timestamp'),
            }

    def clear_calibration_reference(self):
        """清除校准参考数据"""
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None
        self.h_ref_origin = None

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
            full_data[aid]['csi_complex'] = full_data[aid]['csi_complex'] * self.h_ref[aid]['csi_complex'] * sto_factor
            full_data[aid]['csi_magnitude'] = np.abs(full_data[aid]['csi_complex'])
            full_data[aid]['csi_phase'] = np.angle(full_data[aid]['csi_complex'])
        return full_data

    def run_algorithms(self, full_data):
        """运行所有启用的算法"""
        for algo_name, algo_instance in self.algorithms.items():
            full_data = algo_instance.apply(full_data, self.antenna_order)
        return full_data

    def process(self, raw_data):
        """核心处理流程"""
        antenna_index = raw_data['antenna_index']
        antenna_id = self.antenna_mapping.get(antenna_index, f"未知{antenna_index}")
        
        coarse_timestamp = (raw_data['timestamp'] + self.timestamp_scale/2) // self.timestamp_scale
        
        csi_int8 = np.array(raw_data['csi_data'], dtype=np.int8)
        csi_complex = csi_int8[1::2] + 1j * csi_int8[0::2]
        csi_complex = csi_complex.astype(np.complex64)
        # csi_complex = csi_complex * raw_data['gain']
        
        if self.wifi_protocol == 'HT40':
            part1 = csi_complex[64:128] 
            part2 = csi_complex[128:192]
            csi_complex = np.concatenate([part2, part1])
        else:
            part1 = csi_complex[0:32]
            part2 = csi_complex[32:64]
            csi_complex = np.concatenate([part2, part1])
        
        processed = {
            'antenna_id': antenna_id,
            'timestamp': raw_data['timestamp'],
            'coarse_timestamp': coarse_timestamp,
            'mac': raw_data.get('mac', 'N/A'),
            'dmac': raw_data.get('dmac', 'N/A'),
            'rssi': raw_data['rssi'],
            'noise_floor': raw_data['noise_floor'],
            'rate': raw_data['rate'],
            'sgi': raw_data['sgi'],
            'sig_mode': raw_data['sig_mode'],
            'mcs': raw_data['mcs'],
            'cwb': raw_data['cwb'],
            'channel': raw_data['channel'],
            'secondary_channel': raw_data['secondary_channel'],
            'rx_state': raw_data['rx_state'],
            'fft_gain': raw_data['fft_gain'],
            'agc_gain': raw_data['agc_gain'],
            'gain': raw_data['gain'],
            'csi_complex': csi_complex,
            'csi_magnitude': np.abs(csi_complex),
            'csi_phase': np.angle(csi_complex),
            'subcarriers_nums': len(csi_complex)
        }
        self.antenna_data[antenna_id] = processed
        
        if coarse_timestamp not in self.cache:
            self.cache[coarse_timestamp] = []
        self.cache[coarse_timestamp].append(processed)

        while len(self.cache) > 30:
            oldest = next(iter(self.cache))
            del self.cache[oldest]

        current_group = self.cache[coarse_timestamp]
        antennas = {item['antenna_id'] for item in current_group}
        if antennas != self.expected_antennas:
            return None

        full_data = {item['antenna_id']: item for item in current_group}
        del self.cache[coarse_timestamp]
        
        # 校准
        if self.ENABLE_CALIBRATION and self.h_ref is not None:
            full_data = self._calculate_calibrated_data(full_data)
        
        # 算法
        full_data = self.run_algorithms(full_data)
        
        return full_data

    def clear_antenna_data(self):
        """清空天线数据"""
        self.antenna_data.clear()
        self.last_coarse_timestamp = None
        for algo in self.algorithms.values():
            if hasattr(algo, 'clear'):
                algo.clear()

# ==============================================================================
# 数据接收核心
# ==============================================================================
class CSIDataReceiver:
    """CSI 数据接收类 (支持设备 ID 偏移)"""
    def __init__(self, device_id=0):
        self.socket = None
        self.connected = False
        self.receiving_data = False
        self.processor = None  # 由 GUI 外部注入
        self.recv_buffer = b""
        self.device_id = device_id

    def connect(self, host, port):
        """连接到设备"""
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
        """断开连接"""
        self.connected = False
        self.receiving_data = False
        if self.socket:
            self.socket.close()
            self.socket = None

    def send_command(self, command_data):
        """发送命令"""
        if not self.connected or not self.socket:
            return False
        try:
            self.socket.send(command_data)
            return True
        except Exception as e:
            messagebox.showerror("发送错误", f"发送命令失败：{str(e)}")
            return False

    def receive_data_loop(self, full_set_callback):
        """数据接收循环"""
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
                        # 双设备天线索引偏移 (设备 1 的天线索引 +8)
                        if self.device_id == 1:
                            parsed['antenna_index'] += 8
                        
                        if self.processor:
                            full_data = self.processor.process(parsed)
                            if full_data is not None:
                                full_set_callback(full_data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    def parse_csi_packet(self, data):
        """解析 CSI 数据包"""
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

#==============================================================================
#GUI 主窗口框架
#==============================================================================
class CSIDataGUI:
    """CSI 数据采集系统主窗口框架 (双设备)"""
    def __init__(self, root, title="CSI 数据采集系统 (双设备)", window_size="1400x900"):
        self.root = root
        self.root.title(title)
        self.root.geometry(window_size)
        self.setup_style()
        self.received_packets = 0
        
        # 共享处理器
        self.shared_processor = CSIDataProcessor()
        # 双接收器
        self.receivers = {
            0: CSIDataReceiver(device_id=0),
            1: CSIDataReceiver(device_id=1)
        }
        self.receivers[0].processor = self.shared_processor
        self.receivers[1].processor = self.shared_processor
        
        self._last_full_data = None
        self.tab_checkboxes = {}
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
        
        control_frame = ttk.Frame(main_paned)
        main_paned.add(control_frame, weight=0)
        
        data_frame = ttk.Frame(main_paned)
        main_paned.add(data_frame, weight=1)
        
        self.setup_control_panel(control_frame) 
        self.setup_data_panel(data_frame)

    def setup_control_panel(self, parent):
        # 连接控制 (双设备 IP/Port)
        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, pady=2)
        
        # 设备 0
        ttk.Label(status_frame, text="设备 0 IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry0 = ttk.Entry(status_frame, width=12)
        self.ip_entry0.insert(0, "192.168.2.10")
        self.ip_entry0.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry0 = ttk.Entry(status_frame, width=8)
        self.port_entry0.insert(0, "8000")
        self.port_entry0.pack(side=tk.LEFT, padx=(0, 15))

        # 设备 1
        ttk.Label(status_frame, text="设备 1 IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry1 = ttk.Entry(status_frame, width=12)
        self.ip_entry1.insert(0, "192.168.2.11")
        self.ip_entry1.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry1 = ttk.Entry(status_frame, width=8)
        self.port_entry1.insert(0, "8000")
        self.port_entry1.pack(side=tk.LEFT, padx=(0, 15))
        
        self.connect_btn = ttk.Button(status_frame, text="连接",
                                    command=self.toggle_connection, style="Primary.TButton")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        self.clear_btn = ttk.Button(status_frame, text="清除数据",
                                  command=self.clear_all_data, style="Warning.TButton")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.clear_btn.state(['disabled'])
        
        ttk.Label(status_frame, text="数据包:").pack(side=tk.LEFT, padx=(0, 5))
        self.packet_count_label = ttk.Label(status_frame, text="0",
                                          foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.packet_count_label.pack(side=tk.LEFT, padx=(0, 15))
        
        self.status_label = ttk.Label(status_frame, text="未连接", foreground="red",
                                    font=("微软雅黑", 9, "bold"))
        self.status_label.pack(side=tk.LEFT)

        ttk.Label(status_frame, text="模式:").pack(side=tk.LEFT, padx=(15, 5))
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

        # 控制区
        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, pady=5)
        
        mac_frame = ttk.Frame(control_frame)
        mac_frame.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(mac_frame, text="MAC 地址：").pack(side=tk.LEFT, padx=(0, 5))
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
        
        ttk.Label(control_frame, text="信道：").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_var = tk.StringVar(value="1")
        channel_combo = ttk.Combobox(control_frame, textvariable=self.channel_var,
                                   values=[str(i) for i in range(1, 14)], width=3, state="readonly")
        channel_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(control_frame, text="带宽：").pack(side=tk.LEFT, padx=(0, 5))
        self.bandwidth_var = tk.StringVar(value="20M")
        bandwidth_combo = ttk.Combobox(control_frame, textvariable=self.bandwidth_var,
                                   values=["20M", "40M"], width=4, state="readonly")
        bandwidth_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(control_frame, text="设置 wifi",
                  command=self.set_wifi, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(control_frame, text="协议：").pack(side=tk.LEFT, padx=(15, 5))
        self.wifi_protocol_var = tk.StringVar(value="LLTF")
        wifi_protocol_combo = ttk.Combobox(control_frame, textvariable=self.wifi_protocol_var,
                                   values=["LLTF", "HT40"], width=6, state="readonly")
        wifi_protocol_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(control_frame, text="设置协议",
             command=self.set_protocol, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        # 校准控制
        ttk.Button(control_frame, text="校准模式",
                  command=self.set_calibration_mode, style="Success.TButton").pack(side=tk.LEFT, padx=(0, 10))
        ttk.Button(control_frame, text="开启校准 wifi",
                  command=self.send_wifion_frame, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(control_frame, text="关闭校准 wifi",
                  command=self.send_wifioff_frame, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(control_frame, text="应用校准",
                  command=self.apply_calibration, style="Warning.TButton").pack(side=tk.LEFT)
        ttk.Button(control_frame, text="清除校准",
                  command=self.clear_h_ref, style="Warning.TButton").pack(side=tk.LEFT)
        
        # --- 算法动态控件区域 ---
        self.algo_controls_frame = ttk.Frame(parent)
        self.algo_controls_frame.pack(fill=tk.X)
        self.algo_widgets = {}  # 用于追踪算法创建的控件，方便清理

        # Tab 开关
        self.tab_switch_frame = ttk.Frame(parent)
        self.tab_switch_frame.pack(fill=tk.X)

    def setup_data_panel(self, parent):
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        self.tab_manager = TabManager(self.notebook, self.shared_processor)

    def register_tab(self, name, tab_class, enabled=False):
        tab_instance = tab_class(self.notebook, name)
        self.tab_manager.register_tab(name, tab_instance, enabled)
        
        var = tk.BooleanVar(value=enabled)
        cb = ttk.Checkbutton(
            self.tab_switch_frame,
            text=name,
            variable=var,
            command=lambda key=name, v=var: self.on_tab_toggle(key, v)
        )
        cb.pack(side=tk.LEFT, padx=10)
        self.tab_checkboxes[name] = var

    def register_algorithm(self, name, algorithm_instance):
        """注册算法（供子类调用）"""
        self.shared_processor.register_algorithm(name, algorithm_instance)
        if hasattr(algorithm_instance, 'setup_controls'):
            # 传入算法控件容器帧
            container = self.algo_controls_frame
            widgets = algorithm_instance.setup_controls(container, self)
            if widgets:
                self.algo_widgets[name] = widgets

    def toggle_connection(self):
        if not self.receivers[0].connected:
            self.connect_to_device()
        else:
            self.disconnect_from_device()

    def connect_to_device(self):
        ip0 = self.ip_entry0.get()
        try:
            port0 = int(self.port_entry0.get())
            ip1 = self.ip_entry1.get()
            port1 = int(self.port_entry1.get())
        except ValueError:
            messagebox.showerror("错误", "端口号必须是数字")
            return
        
        ok0 = self.receivers[0].connect(ip0, port0)
        ok1 = self.receivers[1].connect(ip1, port1)

        if ok0 and ok1:
            self.connect_btn.config(text="断开连接", style="Danger.TButton")
            self.clear_btn.state(['!disabled'])
            self.status_label.config(text="双设备已连接", foreground=self.colors["success"])
            
            # 启动两个接收线程
            threading.Thread(
                target=self.receivers[0].receive_data_loop,
                args=(self.on_full_antenna_set,),
                daemon=True
            ).start()
            threading.Thread(
                target=self.receivers[1].receive_data_loop,
                args=(self.on_full_antenna_set,),
                daemon=True
            ).start()
        else:
            self.receivers[0].disconnect()
            self.receivers[1].disconnect()
            messagebox.showerror("连接失败", "请检查设备 IP 和端口")

    def disconnect_from_device(self):
        self.receivers[0].disconnect()
        self.receivers[1].disconnect()
        self.connect_btn.config(text="连接", style="Primary.TButton")
        self.clear_btn.state(['disabled'])
        self.status_label.config(text="未连接", foreground="red")

    def clear_all_data(self):
        self.tab_manager.clear_all_tabs()
        self.shared_processor.clear_antenna_data()
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
            # 发送给两个设备
            self.receivers[0].send_command(command)
            self.receivers[1].send_command(command)
            
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
            # 发送给两个设备
            self.receivers[0].send_command(command)
            self.receivers[1].send_command(command)
            
            self.channel_label.config(text=str(channel))
        except ValueError:
            messagebox.showerror("错误", "请选择有效的信道")

    def set_protocol(self):
        self.shared_processor.wifi_protocol = self.wifi_protocol_var.get()
        messagebox.showinfo("提示", f"协议已设置为 {self.wifi_protocol_var.get()}")

    def set_calibration_mode(self):
        command = bytes([0x02] + [0x00] * 6)
        self.receivers[0].send_command(command)
        self.receivers[1].send_command(command)
        self.mode_label.config(text="校准模式")
        self.mac_label.config(text="00:11:22:33:44:55")

    def send_wifion_frame(self):
        command = bytes([0xFF])
        if self.receivers[0].send_command(command):
            messagebox.showinfo("成功", "校准 wifi 已开启")

    def send_wifioff_frame(self):
        command = bytes([0xFE])
        if self.receivers[0].send_command(command):
            messagebox.showinfo("成功", "校准 wifi 已关闭")

    def apply_calibration(self):
        if self._last_full_data is None:
            messagebox.showwarning("校准", "尚无可用数据！请先接收完整数据包。")
            return
        try:
            antenna_order = self.shared_processor.antenna_order
            h_ref_origin = {}
            for aid in antenna_order: 
                data = self._last_full_data[aid]
                csi_data = data['csi_complex']
                h_ref_origin[aid] = {
                    'csi_complex': csi_data.copy(),
                    'timestamp': data.get('timestamp', None)
                }
            # 提取信道信息（从第一个有效天线获取）
            first_aid = next(iter(h_ref_origin.keys()))
            ch = self._last_full_data[first_aid].get('channel', 1)
            sec_ch = self._last_full_data[first_aid].get('secondary_channel', 0)
            # 执行校准计算
            self.shared_processor.calculate_pcb_phase_offsets(ch, sec_ch)
            self.shared_processor.set_calibration_reference(h_ref_origin)
            messagebox.showinfo("校准成功", "校准成功")
        except Exception as e:
            messagebox.showerror("校准错误", f"校准失败：{str(e)}")

    def clear_h_ref(self):
        self.shared_processor.clear_calibration_reference()
        messagebox.showinfo("校准", "校准数据已清除。")

    def on_full_antenna_set(self, full_data):
        self.root.after(0, self.update_ui_with_full_set, full_data)

    def update_ui_with_full_set(self, full_data):
        self.received_packets += 1
        self.packet_count_label.config(text=str(self.received_packets))
        self._last_full_data = full_data
        self.tab_manager.update_all_tabs(full_data)

    def on_tab_toggle(self, tab_name, var):
        enabled = var.get()
        self.tab_manager.set_tab_enabled(tab_name, enabled)

# ==============================================================================
# 显示 Tab 基类
# ==============================================================================
class DisplayTab(ABC):
    """显示标签页的基类模板"""
    def __init__(self, notebook, title):
        self.notebook = notebook
        self.title = title
        self.frame = ttk.Frame(notebook)
        self.setup_ui()
        notebook.add(self.frame, text=title)

    @abstractmethod
    def setup_ui(self):
        """设置 UI 界面"""
        pass

    @abstractmethod
    def update_data(self, full_data):
        """更新数据显示"""
        pass

    def clear_data(self):
        pass

# ==============================================================================
# Tab 管理器
# ==============================================================================
class TabManager:
    """标签页管理器"""
    def __init__(self, notebook, processor):
        self.notebook = notebook
        self.tabs = {}
        self.processor = processor
        self.tab_enabled = {}

    def register_tab(self, name, tab_instance, enabled=False):
        """注册 Tab 页"""
        self.tabs[name] = tab_instance
        self.tab_enabled[name] = enabled

    def update_all_tabs(self, full_data):
        """更新所有启用的标签页"""
        for tab_name, tab in self.tabs.items():
            if self.tab_enabled.get(tab_name, False):
                tab.update_data(full_data)

    def clear_all_tabs(self):
        """清空所有标签页"""
        for tab in self.tabs.values():
            if hasattr(tab, 'clear_data'):
                tab.clear_data()

    def set_tab_enabled(self, tab_name, enabled):
        """设置 Tab 启用状态"""
        self.tab_enabled[tab_name] = enabled
        if enabled and tab_name in self.tabs:
            tab = self.tabs[tab_name]
            if hasattr(tab, 'refresh_if_needed'):
                tab.refresh_if_needed()

def main():
    root = tk.Tk()
    app = CSIDataGUI(root)
    root.mainloop()

if __name__ == "__main__":
    main()