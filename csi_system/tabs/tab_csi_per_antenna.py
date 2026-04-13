"""CSI 幅度与相位 显示标签页模块"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from core.csi_core_single import DisplayTab

class CSI_Per_Ant_Tab(DisplayTab):
    def __init__(self, notebook, title):
        self.selected_index = 0
        super().__init__(notebook, title)

    def setup_ui(self):
        # 顶部天线选择
        frame = tk.Frame(self.frame)
        frame.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(frame, text="Antenna:").pack(side=tk.LEFT)
        self.combo = ttk.Combobox(frame, width=5, state="readonly")
        self.combo.pack(side=tk.LEFT, padx=5)
        self.combo.bind("<<ComboboxSelected>>", lambda e: setattr(self, 'selected_index', int(self.combo.get())))

        # 绘图区域
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(8, 5), sharex=True)
        self.fig.tight_layout()
        self.ax1.set_ylabel('Magnitude')
        self.ax1.grid(True, alpha=0.3)
        self.ax2.set_xlabel('Subcarrier')
        self.ax2.set_ylabel('Phase (deg)')
        self.ax2.grid(True, alpha=0.3)
        
        self.line1, = self.ax1.plot([], [], 'b-')
        self.line2, = self.ax2.plot([], [], 'g-')
        
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

    def update_data(self, full_data):
        if not isinstance(full_data, dict):
            return
            
        # 获取天线顺序
        order = full_data.get('antenna_order', [])
            
        # 更新下拉框
        vals = [str(i) for i in range(len(order))]
        if list(self.combo['values']) != vals:
            self.combo['values'] = vals
            self.combo.set(0)
            self.selected_index = 0
            
        # 获取当前天线数据
        ant_id = order[self.selected_index]
        data = full_data.get(ant_id)
            
        mag = np.array(data.get('csi_magnitude', []))
        phase = np.array(data.get('csi_phase', []))
            
        # 相位解卷绕 (相邻差 > PI 则补偿 2PI)
        phase_unwrapped = np.unwrap(phase, discont=np.pi)
        phase_deg = np.degrees(phase_unwrapped)
        subs = np.arange(len(mag))
        # 更新图表
        self.line1.set_data(subs, mag)
        self.line2.set_data(subs, phase_deg)
        
        # 自动缩放
        self.ax1.relim()
        self.ax1.autoscale_view()
        self.ax2.relim()
        self.ax2.autoscale_view()
        
        self.canvas.draw_idle()

    def clear_data(self):
        self.line1.set_data([], [])
        self.line2.set_data([], [])
        self.canvas.draw_idle()