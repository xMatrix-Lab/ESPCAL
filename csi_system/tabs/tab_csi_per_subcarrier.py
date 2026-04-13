"""子载波天线对比 显示标签页模块（相位-时间曲线图）"""
import numpy as np
import matplotlib.pyplot as plt
plt.rcParams['font.sans-serif'] = ['SimHei']
plt.rcParams['axes.unicode_minus'] = False

from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk
from collections import deque
from core.csi_core_single import DisplayTab

class CSI_Per_Sub_Tab(DisplayTab):
    def __init__(self, notebook, title):
        self.selected_subcarrier = 0
        self.max_history = 2000
        self.phase_history = {}
        self.time_steps = deque(maxlen=self.max_history)
        self.current_step = 0
        super().__init__(notebook, title)

    def setup_ui(self):
        # 顶部子载波选择
        frame = tk.Frame(self.frame)
        frame.pack(fill=tk.X, padx=5, pady=2)
        tk.Label(frame, text="Subcarrier:").pack(side=tk.LEFT)
        self.sub_combo = ttk.Combobox(frame, width=5, state="readonly")
        self.sub_combo.pack(side=tk.LEFT, padx=5)
        self.sub_combo.bind("<<ComboboxSelected>>", lambda e: self._reset_on_switch())

        # 绘图区域
        self.fig, self.ax = plt.subplots(figsize=(8, 5))
        self.fig.tight_layout()
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)

        # 8根天线颜色
        self.colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728',
                       '#9467bd', '#8c564b', '#e377c2', '#7f7f7f']

    def _reset_on_switch(self):
        # 切换子载波时清空
        self.selected_subcarrier = int(self.sub_combo.get())
        self.phase_history.clear()
        self.time_steps.clear()
        self.current_step = 0

    def update_data(self, full_data):
        if not isinstance(full_data, dict):
            return

        order = full_data.get('antenna_order', [])
        if not order:
            return

        first = full_data.get(order[0])
        if not first:
            return
        num_subs = len(first.get('csi_phase', []))
        if num_subs <= 0:
            return

        # 初始化子载波选项
        vals = [str(i) for i in range(num_subs)]
        if list(self.sub_combo['values']) != vals:
            self.sub_combo['values'] = vals
            self.sub_combo.set('0')
            self.selected_subcarrier = 0

        if self.selected_subcarrier >= num_subs:
            self.selected_subcarrier = 0
            self.sub_combo.set('0')

        # 时间轴
        self.current_step += 1
        self.time_steps.append(self.current_step)

        # 读取当前子载波相位
        current = {}
        for ant_id in order:
            data = full_data.get(ant_id)
            if not data:
                continue
            phase = data.get('csi_phase', [])
            if len(phase) > self.selected_subcarrier:
                val = phase[self.selected_subcarrier]
                current[ant_id] = val

        if not current:
            return

        # 更新历史
        for k, v in current.items():
            if k not in self.phase_history:
                self.phase_history[k] = deque(maxlen=self.max_history)
            self.phase_history[k].append(v)

        # 绘图
        self.ax.clear()

        for idx, ant_id in enumerate(order[:8]):
            if ant_id not in self.phase_history:
                continue
            y_data = self.phase_history[ant_id]
            x_data = list(self.time_steps)
            
            if len(x_data) == len(y_data) and len(y_data) > 0:
                self.ax.plot(x_data, y_data, color=self.colors[idx], linewidth=1.5, label=f'Ant {idx}')

        # 图表样式
        self.ax.set_title(f'Subcarrier {self.selected_subcarrier} - Phase vs Time')
        self.ax.set_xlabel('Time Step')
        self.ax.set_ylabel('Phase (rad)')
        self.ax.grid(True, alpha=0.3)
        self.ax.legend(loc='upper right', fontsize=9)
        self.ax.relim()
        self.ax.autoscale_view()

        self.canvas.draw_idle()

    def clear_data(self):
        self.phase_history.clear()
        self.time_steps.clear()
        self.current_step = 0
        self.ax.clear()
        self.canvas.draw_idle()