"""FFT 波束空间显示标签页模块"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import time
from matplotlib.colors import LinearSegmentedColormap

from core.csi_core_single import DisplayTab

# 高对比度自定义颜色映射
colors_high_contrast = [
    (0.0, 0.0, 0.0),      # 黑
    (0.5, 0.0, 0.0),      # 深红
    (1.0, 0.0, 0.0),      # 红
    (1.0, 1.0, 0.0),      # 黄
    (1.0, 1.0, 1.0)       # 白
]
cmap_high_contrast = LinearSegmentedColormap.from_list("high_contrast", colors_high_contrast, N=256)

class FFTBeamspaceTab(DisplayTab):
    """FFT 波束空间 2D 热力图结果显示"""
    
    def __init__(self, notebook, title, resolution_az=64, resolution_el=64):   
        # 分辨率配置（和算法严格对应）
        self.res_az = resolution_az
        self.res_el = resolution_el
        
        # 对比度增强参数
        self.contrast_power = 3.0
        self.noise_floor_clip = 0.01
        
        # 调用父类初始化
        super().__init__(notebook, title)

    def setup_ui(self):
        self.fig, self.ax = plt.subplots(1, 1, figsize=(8, 6))
        self.fig.tight_layout(pad=2.0)
        
        plt.ioff()
        
        self.ax.set_title('FFT Beamspace Heatmap (Raw, No Correction)', fontsize=12)
        self.ax.set_xlabel('Azimuth Angle (°)', fontsize=10)  # X轴=方位角
        self.ax.set_ylabel('Elevation Angle (°)', fontsize=10) # Y轴=仰角
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-90, 90)
        self.ax.grid(False)
        self.ax.spines['top'].set_visible(False)
        self.ax.spines['right'].set_visible(False)
        self.ax.tick_params(axis='both', labelsize=9)
        
        # 初始化热力图shape：(Y轴el长度, X轴az长度)
        self.im_heatmap = self.ax.imshow(
            np.zeros((self.res_el, self.res_az)), 
            cmap='plasma',
            aspect='auto', 
            origin='lower',
            extent=[-90, 90, -90, 90], # X:az(-90~90), Y:el(-90~90)
            vmin=0, vmax=1,
            alpha=0.9
        )
        
        self.cbar = self.fig.colorbar(self.im_heatmap, ax=self.ax, shrink=0.8)
        self.cbar.set_label('Relative Power (Enhanced)', fontsize=9)
        
        self.text_peak = self.ax.text(
            0.02, 0.98, '', transform=self.ax.transAxes,
            verticalalignment='top', fontsize=11, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='black', alpha=0.7),
            color='white'
        )
        
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

    def update_data(self, full_data):
        if not full_data.get('fft2d_computed', False):
            return
        
        # 优先使用原始线性功率谱
        raw_power = full_data.get('raw_power_spectrum', None)
        est_az = full_data.get('aoa_azimuth', None)
        est_el = full_data.get('aoa_elevation', None)
        
        if raw_power is None:
            spectrum_db = full_data.get('aoa_spectrum_2d', None)
            if spectrum_db is None:
                return
            spectrum_np = np.array(spectrum_db)
            min_val = np.min(spectrum_np[np.isfinite(spectrum_np)])
            raw_power = 10 ** ((spectrum_np - min_val) / 10.0)
        else:
            raw_power = np.array(raw_power)
        
        # 功率增强处理
        enhanced_power = raw_power ** self.contrast_power
        max_enhanced = np.max(enhanced_power)
        if max_enhanced > 0:
            threshold = max_enhanced * self.noise_floor_clip
            enhanced_power[enhanced_power < threshold] = 0
        norm_power = enhanced_power / (max_enhanced + 1e-10)
        
        # 更新热力图：shape已经是(el, az)，和imshow匹配
        self.im_heatmap.set_data(norm_power)
        self.im_heatmap.set_clim(0.0, 1.0)
        
        # 更新峰值标记
        if est_az is not None and est_el is not None:
            self.text_peak.set_text(f'Azimuth: {est_az:.1f}°\nElevation: {est_el:.1f}°')
        
        try:
            self.canvas.draw_idle()
        except Exception:
            self.ax.draw_artist(self.ax.patch)
            self.ax.draw_artist(self.im_heatmap)
            self.ax.draw_artist(self.text_peak)
            self.canvas.flush_events()

    def clear_data(self):
        """清空显示数据"""
        self.im_heatmap.set_data(np.zeros((self.res_el, self.res_az)))
        self.im_heatmap.set_clim(0, 1)
        
        self.text_peak.set_text('')
        
        self.canvas.draw_idle()