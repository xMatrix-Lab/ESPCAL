"""AOA 2D 显示标签页模块 - 优化版"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import time

from core.csi_core_single import DisplayTab

class AOA2DTab(DisplayTab):
    """2D AoA 估计结果显示"""
    
    def __init__(self, notebook, title):
        # 调用父类初始化
        super().__init__(notebook, title)

    def setup_ui(self):
        """设置 UI 界面"""
        # 创建双子图（方位角 + 仰角）
        self.fig, (self.ax_azimuth, self.ax_elevation) = plt.subplots(1, 2, figsize=(10, 4))
        self.fig.tight_layout(pad=2.0)
        
        # 关闭 Matplotlib 交互模式
        plt.ioff()
        
        # 配置方位角子图
        self.ax_azimuth.set_title('Azimuth Spectrum', fontsize=10)
        self.ax_azimuth.set_xlabel('Azimuth Angle (°)', fontsize=8)
        self.ax_azimuth.set_ylabel('MUSIC Spectrum (dB)', fontsize=8)
        self.ax_azimuth.set_xlim(-45, 45)
        self.ax_azimuth.set_ylim(-5, 2.5)
        self.ax_azimuth.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
        self.ax_azimuth.spines['top'].set_visible(False)
        self.ax_azimuth.spines['right'].set_visible(False)
        self.ax_azimuth.tick_params(axis='both', labelsize=8)
        
        # 配置仰角子图
        self.ax_elevation.set_title('Elevation Spectrum', fontsize=10)
        self.ax_elevation.set_xlabel('Elevation Angle (°)', fontsize=8)
        self.ax_elevation.set_ylabel('MUSIC Spectrum (dB)', fontsize=8)
        self.ax_elevation.set_xlim(-90, 90)
        self.ax_elevation.set_ylim(-5, 2.5)
        self.ax_elevation.grid(True, alpha=0.3, linestyle='-', linewidth=0.5)
        self.ax_elevation.spines['top'].set_visible(False)
        self.ax_elevation.spines['right'].set_visible(False)
        self.ax_elevation.tick_params(axis='both', labelsize=8)
        
        # 初始化谱线
        self.line_azimuth, = self.ax_azimuth.plot([], [], 'b-', linewidth=1.5, alpha=0.8)
        self.line_elevation, = self.ax_elevation.plot([], [], 'g-', linewidth=1.5, alpha=0.8)
        
        # 峰值标记
        self.peak_az_marker = self.ax_azimuth.axvline(0, color='r', linestyle='--', linewidth=1.5, alpha=0.8)
        self.peak_el_marker = self.ax_elevation.axvline(0, color='r', linestyle='--', linewidth=1.5, alpha=0.8)
        
        # 峰值文本
        self.text_az = self.ax_azimuth.text(
            0.02, 0.98, '', transform=self.ax_azimuth.transAxes,
            verticalalignment='top', fontsize=8,
            bbox=dict(boxstyle='round,pad=0.2', facecolor='wheat', alpha=0.8)
        )
        self.text_el = self.ax_elevation.text(
            0.02, 0.98, '', transform=self.ax_elevation.transAxes,
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
        if not full_data.get('music2d_computed', False):
            return
        
        # 提取数据
        azimuth_spectrum = full_data.get('aoa_spectrum_azimuth', None)
        elevation_spectrum = full_data.get('aoa_spectrum_elevation', None)
        est_az = full_data.get('aoa_azimuth', None)
        est_el = full_data.get('aoa_elevation', None)
        az_axis = full_data.get('aoa_azimuths', None)
        el_axis = full_data.get('aoa_elevations', None)
        
        # 数据完整性检查
        if any(x is None for x in [azimuth_spectrum, elevation_spectrum, 
                                    est_az, est_el, az_axis, el_axis]):
            return
        
        # 更新谱线数据
        self.line_azimuth.set_data(az_axis, azimuth_spectrum)
        self.line_elevation.set_data(el_axis, elevation_spectrum)
        
        # 自动缩放 Y 轴（每个子图独立缩放，最大最小值各偏移 1dB）
        # 方位角子图
        spectrum_az_np = np.array(azimuth_spectrum)
        valid_az = spectrum_az_np[~np.isnan(spectrum_az_np) & np.isfinite(spectrum_az_np)]
        if len(valid_az) > 0:
            y_min_az = np.min(valid_az) - 1
            y_max_az = np.max(valid_az) + 1
            if y_min_az < y_max_az:
                self.ax_azimuth.set_ylim(y_min_az, y_max_az)
        
        # 仰角子图
        spectrum_el_np = np.array(elevation_spectrum)
        valid_el = spectrum_el_np[~np.isnan(spectrum_el_np) & np.isfinite(spectrum_el_np)]
        if len(valid_el) > 0:
            y_min_el = np.min(valid_el) - 1
            y_max_el = np.max(valid_el) + 1
            if y_min_el < y_max_el:
                self.ax_elevation.set_ylim(y_min_el, y_max_el)
        
        # 更新峰值标记
        self.peak_az_marker.set_xdata([est_az, est_az])
        self.peak_el_marker.set_xdata([est_el, est_el])
        
        # 更新峰值文本
        self.text_az.set_text(f'Peak: {est_az:.1f}°')
        self.text_el.set_text(f'Peak: {est_el:.1f}°')
        
        # 重绘画布（带异常处理）
        try:
            self.canvas.draw_idle()
        except Exception:
            # 降级为手动重绘
            for ax in [self.ax_azimuth, self.ax_elevation]:
                ax.draw_artist(ax.patch)
                ax.draw_artist(self.line_azimuth if ax == self.ax_azimuth else self.line_elevation)
                ax.draw_artist(self.peak_az_marker if ax == self.ax_azimuth else self.peak_el_marker)
                ax.draw_artist(self.text_az if ax == self.ax_azimuth else self.text_el)
            self.canvas.blit(self.fig.bbox)

    def clear_data(self):
        """清空显示数据"""
        # 重置显示元素
        self.line_azimuth.set_data([], [])
        self.line_elevation.set_data([], [])
        self.peak_az_marker.set_xdata([0, 0])
        self.peak_el_marker.set_xdata([0, 0])
        self.text_az.set_text('')
        self.text_el.set_text('')
        
        # 重置 Y 轴为默认范围
        self.ax_azimuth.set_ylim(-5, 2.5)
        self.ax_elevation.set_ylim(-5, 2.5)
        
        # 重绘画布
        self.canvas.draw_idle()