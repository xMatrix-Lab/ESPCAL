import tkinter as tk
from tkinter import ttk, messagebox, filedialog
import socket
import threading
import struct
import numpy as np
from collections import deque
import csv
from datetime import datetime


tab_enabled = {}

class CSIDataProcessor():
    """简化版 CSI 数据处理器 - 只收集原始数据"""
    timestamp_scale = 10000

    def __init__(self):
        
        self.antenna_mapping = {
            6:  "00", 3:  "01",  14:  "02", 11:  "03",
            5:  "10", 2:  "11",  13:  "12", 10:  "13",
            4:  "20", 1:  "21",  12:  "22", 9:   "23",
            7:  "30", 0:  "31",  15:  "32", 8:   "33"
        }
        self.expected_antennas = set(self.antenna_mapping.values())
        self.antenna_order = [
            "00", "01", "02", "03",
            "10", "11", "12", "13",
            "20", "21", "22", "23",
            "30", "31", "32", "33"
        ]
        
        self.antenna_data = {}
        self.last_coarse_timestamp = None
        self.wifi_protocol = 'LLTF'
        
        # 数据缓存
        self.data_buffer = deque(maxlen=5000)  # 缓存最近 5000 帧
        self.buffer_lock = threading.Lock()
        
        # 校准相关（保留但简化）
        self.h_ref = None
        self.h_ref_origin = None
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
    
    def process(self, raw_data):
        """处理原始数据，只进行基本解析和缓存"""
        antenna_index = raw_data['antenna_index']
        antenna_id = self.antenna_mapping.get(antenna_index, f"未知{antenna_index}")
        
        # 计算粗略时间戳（用于帧对齐）
        coarse_timestamp = (raw_data['timestamp'] + self.timestamp_scale/2) // self.timestamp_scale
        
        # 如果是新时间戳（新帧），清空旧缓存
        if self.last_coarse_timestamp is not None and coarse_timestamp != self.last_coarse_timestamp:
            self.antenna_data.clear()
        self.last_coarse_timestamp = coarse_timestamp
        
        # 转换 CSI 为复数
        csi_int8 = np.array(raw_data['csi_data'], dtype=np.int8)
        csi_complex = csi_int8[0::2] + 1j * csi_int8[1::2]
        csi_complex = csi_complex.astype(np.complex64)
        
        # 重新排列子载波
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
            'dmac': raw_data.get('dmac', 'N/A'),
            'rssi': raw_data['rssi'],
            'noise_floor': raw_data['noise_floor'],
            'rate': raw_data['rate'],
            'channel': raw_data['channel'],
            'secondary_channel': raw_data['secondary_channel'],
            'csi_complex': csi_complex,
            'csi_magnitude': np.abs(csi_complex),
            'csi_phase': np.angle(csi_complex),
            'subcarriers_nums': len(csi_complex)
        }
        
        self.antenna_data[antenna_id] = processed
        
        # 检查是否收齐 16 根天线
        if set(self.antenna_data.keys()) != self.expected_antennas:
            return None
        
        # 检查：所有天线 coarse_timestamp 必须完全一致
        coarse_timestamps = {data['coarse_timestamp'] for data in self.antenna_data.values()}
        if len(coarse_timestamps) != 1:
            self.antenna_data.clear()
            self.last_coarse_timestamp = None
            return None
        
        # 收齐一帧完整数据
        full_data = dict(self.antenna_data)
        self.antenna_data.clear()
        self.last_coarse_timestamp = None
        
        # 添加到缓存（线程安全）
        with self.buffer_lock:
            self.data_buffer.append(full_data)
        
        return full_data
    
    def get_buffer_size(self):
        """获取当前缓存长度"""
        with self.buffer_lock:
            return len(self.data_buffer)
    
    def clear_buffer(self):
        """清空缓存"""
        with self.buffer_lock:
            self.data_buffer.clear()
        self.antenna_data.clear()
        self.last_coarse_timestamp = None
    
    def save_to_csv(self, filepath):
        """保存缓存数据到 CSV 文件 - 复数形式"""
        try:
            with self.buffer_lock:
                buffer_copy = list(self.data_buffer)
            
            with open(filepath, 'w', newline='', encoding='utf-8') as f:
                writer = csv.writer(f)
                
                # 获取子载波数量
                num_subcarriers = 64 if self.wifi_protocol == 'LLTF' else 128
                
                # 写入表头
                header = ['frame_index', 'timestamp', 'antenna_id', 'channel', 'mac', 'rssi', 'noise_floor']
                # 添加子载波列（复数形式）
                for i in range(num_subcarriers):
                    header.append(f'sub{i}_csi')
                writer.writerow(header)
                
                # 写入数据
                for frame_idx, frame_data in enumerate(buffer_copy):
                    for antenna_id in self.antenna_order:
                        if antenna_id in frame_data:
                            data = frame_data[antenna_id]
                            row = [
                                frame_idx,
                                data['timestamp'],
                                antenna_id,
                                data['channel'],
                                data['mac'],
                                data['rssi'],
                                data['noise_floor']
                            ]
                            # 添加复数形式 CSI（如 0.5+0.3j）
                            for i in range(data['subcarriers_nums']):
                                csi_val = data['csi_complex'][i]
                                # 格式化为复数字符串
                                row.append(f"{csi_val.real:.6f}+{csi_val.imag:.6f}j")
                            writer.writerow(row)
            
            return True, f"成功保存 {len(buffer_copy)} 帧数据 ({len(buffer_copy) * 16} 行)"
        except Exception as e:
            return False, f"保存失败：{str(e)}"
    
    def clear_antenna_data(self):
        """清空天线数据缓存"""
        self.antenna_data.clear()
        self.last_coarse_timestamp = None
    
    def set_calibration_reference(self, h_ref_origin):
        """设置校准参考数据（保留接口）"""
        self.h_ref_origin = h_ref_origin
        self.h_ref = h_ref_origin
    
    def clear_calibration_reference(self):
        """清除校准参考数据（保留接口）"""
        self.h_ref = None
        self.h_ref_origin = None


class CSIDataReceiver:
    """数据接收器（保持原逻辑）"""
    
    def __init__(self):
        self.socket = None
        self.connected = False
        self.receiving_data = False
        self.processor = None
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
    
    def receive_data_loop(self, full_set_callback, device_id):
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
                        if device_id == 1:
                            parsed['antenna_index'] += 8
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
                'csi_len': csi_len,
                'csi_data': [b if b < 128 else b - 256 for b in csi_buffer],
            }
        except Exception as e:
            print(f"Parse error: {e}")
            return None


class CSIDataGUI:
    """CSI 数据收集 GUI - 保留所有控制按钮"""
    
    def __init__(self, root):
        self.root = root
        self.root.title("CSI 原始数据收集系统")
        self.root.geometry("1200x800")
        self.setup_style()
        
        self.received_packets = 0
        self.shared_processor = CSIDataProcessor()
        
        self.receivers = {
            0: CSIDataReceiver(),
            1: CSIDataReceiver()
        }
        self.receivers[0].processor = self.shared_processor
        self.receivers[1].processor = self.shared_processor
        
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
        style.configure("TCombobox", font=("微软雅黑", 9))
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
        
        # 控制面板
        control_frame = ttk.LabelFrame(main_paned, text="控制面板", padding="8")
        main_paned.add(control_frame, weight=0)
        
        # 状态面板
        status_frame = ttk.LabelFrame(main_paned, text="数据状态", padding="8")
        main_paned.add(status_frame, weight=0)
        
        # 数据显示面板
        data_frame = ttk.Frame(main_paned)
        main_paned.add(data_frame, weight=1)
        
        self.setup_control_panel(control_frame)
        self.setup_status_panel(status_frame)
        self.setup_data_panel(data_frame)
    
    def setup_control_panel(self, parent):
        # === 第一行：连接配置 ===
        conn_frame = ttk.Frame(parent)
        conn_frame.pack(fill=tk.X, pady=2)
        
        # 设备 0
        ttk.Label(conn_frame, text="设备 0 IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry0 = ttk.Entry(conn_frame, width=12)
        self.ip_entry0.insert(0, "192.168.2.10")
        self.ip_entry0.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(conn_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry0 = ttk.Entry(conn_frame, width=8)
        self.port_entry0.insert(0, "8000")
        self.port_entry0.pack(side=tk.LEFT, padx=(0, 15))
        
        # 设备 1
        ttk.Label(conn_frame, text="设备 1 IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry1 = ttk.Entry(conn_frame, width=12)
        self.ip_entry1.insert(0, "192.168.2.11")
        self.ip_entry1.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(conn_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry1 = ttk.Entry(conn_frame, width=8)
        self.port_entry1.insert(0, "8000")
        self.port_entry1.pack(side=tk.LEFT, padx=(0, 15))
        
        # 连接按钮
        self.connect_btn = ttk.Button(conn_frame, text="连接",
                                    command=self.toggle_connection, style="Primary.TButton")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))
        
        # 清除数据按钮
        self.clear_btn = ttk.Button(conn_frame, text="清除数据",
                                  command=self.clear_all_data, style="Warning.TButton")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.clear_btn.state(['disabled'])
        
        # 保存数据按钮
        self.save_btn = ttk.Button(conn_frame, text="保存数据到 CSV",
                                 command=self.save_data, style="Success.TButton")
        self.save_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.save_btn.state(['disabled'])
        
        # === 第二行：MAC 和模式设置 ===
        mode_frame = ttk.Frame(parent)
        mode_frame.pack(fill=tk.X, pady=5)
        
        mac_frame = ttk.Frame(mode_frame)
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
        
        ttk.Button(mode_frame, text="普通模式", 
                  command=self.set_normal_mode, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))
        
        # === 第三行：WiFi 设置 ===
        wifi_frame = ttk.Frame(parent)
        wifi_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(wifi_frame, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_var = tk.StringVar(value="1")
        channel_combo = ttk.Combobox(wifi_frame, textvariable=self.channel_var,
                                   values=[str(i) for i in range(1, 14)], width=3, state="readonly")
        channel_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Label(wifi_frame, text="带宽:").pack(side=tk.LEFT, padx=(0, 5))
        self.bandwidth_var = tk.StringVar(value="20M")
        bandwidth_combo = ttk.Combobox(wifi_frame, textvariable=self.bandwidth_var,
                                   values=["20M", "40M"], width=4, state="readonly")
        bandwidth_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(wifi_frame, text="设置 WiFi",
                  command=self.set_wifi, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(wifi_frame, text="校准模式",
                  command=self.set_calibration_mode, style="Success.TButton").pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(wifi_frame, text="开启校准 WiFi",
                  command=self.send_wifion_frame, style="Warning.TButton").pack(side=tk.LEFT, padx=(0, 5))
        
        ttk.Button(wifi_frame, text="关闭校准 WiFi",
                  command=self.send_wifioff_frame, style="Warning.TButton").pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(wifi_frame, text="应用校准",
                  command=self.apply_calibration, style="Warning.TButton").pack(side=tk.LEFT, padx=(0, 5))
        
        ttk.Button(wifi_frame, text="清除校准",
                  command=self.clear_h_ref, style="Warning.TButton").pack(side=tk.LEFT)
        
        # === 第四行：协议设置 ===
        protocol_frame = ttk.Frame(parent)
        protocol_frame.pack(fill=tk.X, pady=5)
        
        ttk.Label(protocol_frame, text="协议:").pack(side=tk.LEFT, padx=(5, 5))
        self.wifi_protocol_var = tk.StringVar(value="LLTF")
        wifi_protocol_combo = ttk.Combobox(protocol_frame, textvariable=self.wifi_protocol_var,
                                   values=["LLTF", "HT40"], width=6, state="readonly")
        wifi_protocol_combo.pack(side=tk.LEFT, padx=(0, 10))
        
        ttk.Button(protocol_frame, text="设置协议",
                  command=self.set_protocol, style="Primary.TButton").pack(side=tk.LEFT)
        
        # === 第五行：标签页显示开关 ===
        tab_switch_frame = ttk.Frame(parent)
        tab_switch_frame.pack(fill=tk.X)
        
        self.tab_checkboxes = {}
        tab_names = [
            ("multi_antenna", "多天线监控"),
            ("csi_plot", "CSI 可视化"),
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
            tab_enabled[tab_key] = False
    
    def setup_status_panel(self, parent):
        # 连接状态
        status_row1 = ttk.Frame(parent)
        status_row1.pack(fill=tk.X, pady=2)
        
        self.status_label = ttk.Label(status_row1, text="未连接", foreground="red",
                                    font=("微软雅黑", 9, "bold"))
        self.status_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row1, text="数据包:").pack(side=tk.LEFT, padx=(0, 5))
        self.packet_count_label = ttk.Label(status_row1, text="0",
                                          foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.packet_count_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row1, text="缓存帧数:").pack(side=tk.LEFT, padx=(0, 5))
        self.buffer_count_label = ttk.Label(status_row1, text="0",
                                          foreground=self.colors["secondary"], font=("微软雅黑", 9, "bold"))
        self.buffer_count_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row1, text="缓存大小 (MB):").pack(side=tk.LEFT, padx=(0, 5))
        self.buffer_size_label = ttk.Label(status_row1, text="0.0",
                                         foreground=self.colors["secondary"], font=("微软雅黑", 9, "bold"))
        self.buffer_size_label.pack(side=tk.LEFT)
        
        # 配置状态
        status_row2 = ttk.Frame(parent)
        status_row2.pack(fill=tk.X, pady=2)
        
        ttk.Label(status_row2, text="模式:").pack(side=tk.LEFT, padx=(0, 5))
        self.mode_label = ttk.Label(status_row2, text="普通模式",
                                  foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mode_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row2, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_label = ttk.Label(status_row2, text="1",
                                     foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.channel_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row2, text="协议:").pack(side=tk.LEFT, padx=(0, 5))
        self.protocol_label = ttk.Label(status_row2, text="LLTF",
                                      foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.protocol_label.pack(side=tk.LEFT, padx=(0, 15))
        
        ttk.Label(status_row2, text="过滤 MAC:").pack(side=tk.LEFT, padx=(0, 5))
        self.mac_label = ttk.Label(status_row2, text="00:55:66:77:88:88",
                                 foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mac_label.pack(side=tk.LEFT)
    
    def setup_data_panel(self, parent):
        # 多天线数据显示
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)
        
        # 多天线监控标签页
        self.multi_antenna_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.multi_antenna_frame, text="多天线监控")
        self.setup_multi_antenna_display()
        
        # CSI 可视化标签页
        self.csi_plot_frame = ttk.Frame(self.notebook)
        self.notebook.add(self.csi_plot_frame, text="CSI 可视化")
        self.setup_csi_plot_display()
    
    def setup_multi_antenna_display(self):
        """设置多天线数据显示"""
        main_frame = ttk.Frame(self.multi_antenna_frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        for i in range(4):
            main_frame.rowconfigure(i, weight=1)
        for j in range(4):
            main_frame.columnconfigure(j, weight=1)
        
        self.antenna_labels = {}
        antenna_ids = self.shared_processor.antenna_order
        
        for i, antenna_id in enumerate(antenna_ids):
            row = i // 4
            col = i % 4
            antenna_frame = ttk.LabelFrame(main_frame, text=f"天线 {antenna_id}", padding="5")
            antenna_frame.grid(row=row, column=col, padx=3, pady=3, sticky="nsew")
            
            text_widget = tk.Text(antenna_frame, height=8, width=20, font=("Consolas", 8),
                                bg="#f8f9fa", relief="flat", bd=0, wrap=tk.WORD)
            text_widget.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
            text_widget.insert(tk.END, f"等待天线 {antenna_id} 数据...\n")
            text_widget.config(state=tk.DISABLED)
            self.antenna_labels[antenna_id] = text_widget
    
    def setup_csi_plot_display(self):
        """设置 CSI 可视化显示（简化版，用文本显示）"""
        info_frame = ttk.Frame(self.csi_plot_frame)
        info_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        ttk.Label(info_frame, text="CSI 数据已缓存，可点击'保存数据到 CSV'导出",
                 font=("微软雅黑", 11)).pack(pady=20)
        
        self.csi_info_text = tk.Text(info_frame, height=20, width=80, font=("Consolas", 9))
        self.csi_info_text.pack(fill=tk.BOTH, expand=True)
        self.csi_info_text.insert(tk.END, "等待数据...\n")
        self.csi_info_text.config(state=tk.DISABLED)
    
    def toggle_connection(self):
        if not self.receivers[0].connected:
            self.connect_to_device()
        else:
            self.disconnect_from_device()
    
    def connect_to_device(self):
        ip0 = self.ip_entry0.get()
        port0 = int(self.port_entry0.get())
        ip1 = self.ip_entry1.get()
        port1 = int(self.port_entry1.get())
        
        ok0 = self.receivers[0].connect(ip0, port0)
        ok1 = self.receivers[1].connect(ip1, port1)
        
        if ok0 and ok1:
            self.connect_btn.config(text="断开连接", style="Danger.TButton")
            self.clear_btn.state(['!disabled'])
            self.save_btn.state(['!disabled'])
            self.status_label.config(text="双设备已连接", foreground=self.colors["success"])
            
            # 启动两个独立接收线程
            threading.Thread(target=self.receivers[0].receive_data_loop,
                            args=(self.on_full_antenna_set, 0), daemon=True).start()
            threading.Thread(target=self.receivers[1].receive_data_loop,
                            args=(self.on_full_antenna_set, 1), daemon=True).start()
        else:
            self.receivers[0].disconnect()
            self.receivers[1].disconnect()
            messagebox.showerror("连接失败", "请检查设备 IP 和端口")
    
    def disconnect_from_device(self):
        self.receivers[0].disconnect()
        self.receivers[1].disconnect()
        self.connect_btn.config(text="连接", style="Primary.TButton")
        self.clear_btn.state(['disabled'])
        self.save_btn.state(['disabled'])
        self.status_label.config(text="未连接", foreground="red")
    
    def clear_all_data(self):
        self.shared_processor.clear_buffer()
        self.received_packets = 0
        self.packet_count_label.config(text="0")
        self.buffer_count_label.config(text="0")
        self.buffer_size_label.config(text="0.0")
        self.clear_csi_display()
        messagebox.showinfo("清除", "数据缓存已清空")
    
    def save_data(self):
        buffer_size = self.shared_processor.get_buffer_size()
        if buffer_size == 0:
            messagebox.showwarning("保存", "暂无数据可保存")
            return
        
        filepath = filedialog.asksaveasfilename(
            defaultextension=".csv",
            filetypes=[("CSV 文件", "*.csv"), ("所有文件", "*.*")],
            initialfile=f"csi_data_{datetime.now().strftime('%Y%m%d_%H%M%S')}.csv"
        )
        
        if filepath:
            success, message = self.shared_processor.save_to_csv(filepath)
            if success:
                messagebox.showinfo("保存成功", message)
            else:
                messagebox.showerror("保存失败", message)
    
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
            ok0 = self.receivers[0].send_command(command)
            ok1 = self.receivers[1].send_command(command)
            if ok0 and ok1:
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
            ok0 = self.receivers[0].send_command(command)
            ok1 = self.receivers[1].send_command(command)
            if ok0 and ok1:
                self.channel_label.config(text=str(channel))
                messagebox.showinfo("成功", f"WiFi 设置已发送 (信道{channel}, 带宽{self.bandwidth_var.get()})")
        except ValueError:
            messagebox.showerror("错误", "请选择有效的信道")
    
    def set_protocol(self):
        self.shared_processor.wifi_protocol = self.wifi_protocol_var.get()
        self.protocol_label.config(text=self.wifi_protocol_var.get())
        messagebox.showinfo("成功", f"协议已设置为 {self.wifi_protocol_var.get()}")
    
    def set_calibration_mode(self):
        command = bytes([0x02] + [0x00] * 6)
        ok0 = self.receivers[0].send_command(command)
        ok1 = self.receivers[1].send_command(command)
        if ok0 and ok1:
            self.mode_label.config(text="校准模式")
            self.mac_label.config(text="00:11:22:33:44:55")
    
    def send_wifion_frame(self):
        command = bytes([0xFF])
        if self.receivers[0].send_command(command):
            messagebox.showinfo("成功", "校准 WiFi 已开启（仅发送给设备 0）")
    
    def send_wifioff_frame(self):
        command = bytes([0xFE])
        if self.receivers[0].send_command(command):
            messagebox.showinfo("成功", "校准 WiFi 已关闭（仅发送给设备 0）")
    
    def apply_calibration(self):
        if self._last_full_data is not None:
            h_ref_origin = {
                aid: {
                    'csi_complex': data['csi_complex'].copy(),
                    'timestamp': data.get('timestamp', None)
                }
                for aid, data in self._last_full_data.items()
            }
            self.shared_processor.set_calibration_reference(h_ref_origin)
            messagebox.showinfo("校准", "校准数据已更新！")
        else:
            messagebox.showwarning("校准", "尚无可用数据！请先接收完整数据包。")
    
    def clear_h_ref(self):
        self.shared_processor.clear_calibration_reference()
        messagebox.showinfo("校准", "校准数据已清除")
    
    def on_full_antenna_set(self, full_data):
        """收齐 16 个天线后，更新 UI"""
        self.root.after(0, self.update_ui_with_full_set, full_data)
    
    def update_ui_with_full_set(self, full_data):
        self.received_packets += 1
        self.packet_count_label.config(text=str(self.received_packets))
        self._last_full_data = full_data
        
        # 更新缓存计数
        buffer_size = self.shared_processor.get_buffer_size()
        self.buffer_count_label.config(text=str(buffer_size))
        
        # 估算缓存大小（每帧约 16 天线 × 64 子载波 × 16 字节 ≈ 16KB）
        buffer_mb = buffer_size * 16 * 64 * 16 / (1024 * 1024)
        self.buffer_size_label.config(text=f"{buffer_mb:.1f}")
        
        # 更新多天线显示
        if tab_enabled.get("multi_antenna", False):
            self.update_multi_antenna_display(full_data)
        
        # 更新 CSI 信息显示
        if tab_enabled.get("csi_plot", False):
            self.update_csi_display(full_data)
    
    def update_multi_antenna_display(self, full_data):
        """更新多天线数据显示"""
        for antenna_id, data in full_data.items():
            if antenna_id in self.antenna_labels:
                text_widget = self.antenna_labels[antenna_id]
                text_widget.config(state=tk.NORMAL)
                text_widget.delete(1.0, tk.END)
                
                info = (
                    f"时间戳：{data['timestamp']}\n"
                    f"RSSI: {data['rssi']} dBm\n"
                    f"噪声：{data['noise_floor']} dBm\n"
                    f"MAC: {data['mac']}\n"
                    f"信道：{data['channel']}\n"
                    f"子载波：{data['subcarriers_nums']}\n"
                )
                text_widget.insert(tk.END, info)
                text_widget.config(state=tk.DISABLED)
    
    def update_csi_display(self, full_data):
        """更新 CSI 信息显示"""
        self.csi_info_text.config(state=tk.NORMAL)
        self.csi_info_text.delete(1.0, tk.END)
        
        sample_antenna = "00"
        if sample_antenna in full_data:
            data = full_data[sample_antenna]
            info = (
                f"=== 天线 {sample_antenna} CSI 数据 ===\n"
                f"时间戳：{data['timestamp']}\n"
                f"RSSI: {data['rssi']} dBm\n"
                f"子载波数：{data['subcarriers_nums']}\n\n"
                f"前 10 个子载波 CSI（复数形式）:\n"
            )
            for i in range(min(10, data['subcarriers_nums'])):
                csi_val = data['csi_complex'][i]
                info += f"  Sub{i}: {csi_val.real:.6f}+{csi_val.imag:.6f}j\n"
            self.csi_info_text.insert(tk.END, info)
        self.csi_info_text.config(state=tk.DISABLED)
    
    def clear_csi_display(self):
        """清除 CSI 显示"""
        for text_widget in self.antenna_labels.values():
            text_widget.config(state=tk.NORMAL)
            text_widget.delete(1.0, tk.END)
            text_widget.insert(tk.END, "等待数据...\n")
            text_widget.config(state=tk.DISABLED)
        
        self.csi_info_text.config(state=tk.NORMAL)
        self.csi_info_text.delete(1.0, tk.END)
        self.csi_info_text.insert(tk.END, "等待数据...\n")
        self.csi_info_text.config(state=tk.DISABLED)
    
    def on_tab_toggle(self, tab_name, var):
        """标签页开关回调"""
        enabled = var.get()
        tab_enabled[tab_name] = enabled


def main():
    root = tk.Tk()
    app = CSIDataGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()