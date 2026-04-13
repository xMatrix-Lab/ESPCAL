"""AOA 1D 显示标签页模块"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import time

# 从核心模块导入基类
from core.csi_core_single import DisplayTab

class AOA1DTab(DisplayTab):
    """1D AoA 估计结果显示"""
    
    def __init__(self, notebook, title):     
        # 调用父类初始化
        super().__init__(notebook, title)

    def setup_ui(self):
        """设置UI界面"""
        self.fig, self.ax = plt.subplots(1, 1, figsize=(8, 4))
        self.fig.tight_layout(pad=2.0)
        
        # 关闭Matplotlib交互模式
        plt.ioff()
        
        # 设置坐标轴样式
        self.ax.set_title('1D AoA Estimation (Azimuth)', fontsize=10)
        self.ax.set_xlabel('Azimuth Angle (°)', fontsize=8)
        self.ax.set_ylabel('MUSIC Spectrum (dB)', fontsize=8)
        self.ax.set_xlim(-90, 90)  # X轴固定范围
        self.ax.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.tick_params(axis='both', labelsize=8)
        
        # 初始化谱线
        self.line_spectrum, = self.ax.plot([], [], 'b-', linewidth=1.5, alpha=0.8)
        
        # 峰值标记
        self.peak_marker = self.ax.axvline(0, color='r', linestyle='--', linewidth=1.5, alpha=0.8)
        
        # 峰值文本
        self.text_angle = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=8,
            bbox=dict(boxstyle='round,pad=0.2', facecolor='wheat', alpha=0.8)
        )
        
        # 创建画布
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def update_data(self, full_data):
        """更新显示数据"""
        # 数据有效性检查
        if not full_data.get('music_computed', False):
            return
        
        # 提取数据
        spectrum = full_data.get('aoa_spectrum', None)
        est_angle = full_data.get('aoa', None)
        angles = full_data.get('aoa_angles', None)
        
        # 数据有效性检查
        if spectrum is None or angles is None or est_angle is None:
            return
        
        # 更新谱线数据
        self.line_spectrum.set_data(angles, spectrum)
        
        # 自动缩放Y轴
        spectrum_np = np.array(spectrum)
        y_min = np.min(spectrum_np) - 1  # 底部留1dB余量
        y_max = np.max(spectrum_np) + 1  # 顶部留1dB余量
        self.ax.set_ylim(y_min, y_max)
        
        # 更新峰值标记
        self.peak_marker.set_xdata([est_angle, est_angle])
        self.text_angle.set_text(f'Peak: {est_angle:.1f}°')
        
        # 重绘画布
        try:
            self.canvas.draw_idle()
        except:
            self.ax.draw_artist(self.ax.patch)
            self.ax.draw_artist(self.line_spectrum)
            self.ax.draw_artist(self.peak_marker)
            self.ax.draw_artist(self.text_angle)
            self.canvas.blit(self.fig.bbox)

    def clear_data(self):
        """清空显示数据"""
        # 重置显示元素
        self.line_spectrum.set_data([], [])
        self.peak_marker.set_xdata([0, 0])
        self.text_angle.set_text('')
        
        # 重置Y轴为默认范围
        self.ax.set_ylim(-5, 2.5)
        
        # 重绘画布
        self.canvas.draw_idle()