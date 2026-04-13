"""AOA 2D 显示标签页模块 - 真二维联合谱优化版"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from matplotlib.colors import Normalize
import tkinter as tk


from core.csi_core_single import DisplayTab

class AOA2DTab(DisplayTab):
    """2D AoA 估计结果显示"""
    
    def __init__(self, notebook, title):      
        # 颜色映射配置
        self.cmap_name = 'viridis'  # 可选: 'plasma', 'inferno', 'magma', 'jet'
        self.colorbar_vmin = -40    # dB 下限
        self.colorbar_vmax = 0      # dB 上限
        
        # 调用父类初始化
        super().__init__(notebook, title)

    def setup_ui(self):
        """设置 UI 界面 - 2维热力图 + 边际投影"""
        # 创建布局: 主图(2D谱) + 右侧边际(仰角投影) + 下方边际(方位投影)
        # 使用 gridspec 精确控制子图比例
        from matplotlib.gridspec import GridSpec
        
        self.fig = plt.figure(figsize=(10, 8), constrained_layout=True)
        gs = GridSpec(3, 3, figure=self.fig, width_ratios=[1, 0.05, 0.3], 
                      height_ratios=[0.3, 0.05, 1], wspace=0.02, hspace=0.02)
        
        # 主图: 2维联合谱
        self.ax_main = self.fig.add_subplot(gs[2, 0])
        # 颜色条
        self.ax_cbar = self.fig.add_subplot(gs[2, 1])
        
        # 关闭交互模式
        plt.ioff()
        
        # ========== 配置主图 (2D 联合谱) ==========
        self.ax_main.set_title('2D MUSIC Spectrum (Azimuth × Elevation)', fontsize=10, pad=10)
        self.ax_main.set_xlabel('Azimuth Angle (°)', fontsize=9)
        self.ax_main.set_ylabel('Elevation Angle (°)', fontsize=9)
        self.ax_main.set_xlim(-90, 90)
        self.ax_main.set_ylim(-90, 90)
        self.ax_main.grid(True, alpha=0.2, linestyle=':', linewidth=0.5)
        
        # 初始化2维谱图像
        self.im_2d = self.ax_main.imshow(
            np.zeros((128, 128)), 
            extent=[-90, 90, -90, 90],  # [left, right, bottom, top]
            origin='lower',             # 仰角-90°在底部
            cmap=self.cmap_name,
            norm=Normalize(vmin=self.colorbar_vmin, vmax=self.colorbar_vmax),
            aspect='auto',
            interpolation='bilinear'
        )
        
        # 峰值标记 (十字线 + 点)
        self.peak_cross_h = self.ax_main.axhline(0, color='white', linestyle='--', linewidth=1, alpha=0.9)
        self.peak_cross_v = self.ax_main.axvline(0, color='white', linestyle='--', linewidth=1, alpha=0.9)
        self.peak_dot, = self.ax_main.plot([], [], 'r+', markersize=12, markeredgewidth=2, label='Peak')
        
        # 峰值文本
        self.text_peak = self.ax_main.text(
            0.02, 0.98, '', transform=self.ax_main.transAxes,
            verticalalignment='top', fontsize=9, fontweight='bold',
            bbox=dict(boxstyle='round,pad=0.3', facecolor='white', edgecolor='red', alpha=0.9)
        )
        
        # ========== 颜色条 ==========
        self.cbar = self.fig.colorbar(self.im_2d, cax=self.ax_cbar, orientation='vertical')
        self.cbar.set_label('MUSIC Spectrum (dB)', fontsize=8, rotation=90, labelpad=10)
        self.cbar.ax.tick_params(labelsize=7)
        
        # 创建画布
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)
        
        self.frame.bind('<Configure>', self._on_resize)

    def _on_resize(self, event=None):
        """窗口大小变化时自适应调整"""
        if event and event.widget != self.frame:
            return
        try:
            self.canvas.draw_idle()
        except:
            pass

    def update_data(self, full_data):
        """更新显示数据 - 支持真2D谱或兼容旧版1D谱"""
        # 数据有效性检查
        if not full_data.get('music2d_computed', False):
            return

        spectrum_2d = full_data.get('aoa_spectrum_2d', None)
        az_axis = full_data.get('aoa_azimuths', None)
        el_axis = full_data.get('aoa_elevations', None)
        est_az = full_data.get('aoa_azimuth', None)
        est_el = full_data.get('aoa_elevation', None)
        
        self._update_2d_mode(spectrum_2d, az_axis, el_axis, est_az, est_el)
        
        # 重绘画布（带异常保护）
        try:
            self.canvas.draw_idle()
        except Exception as e:
            # 降级：只更新必要艺术家对象
            self._safe_blit_draw()

    def _update_2d_mode(self, spectrum_2d, est_az, est_el):
        """更新真2D联合谱显示"""
        # 转换数据
        spec_array = np.array(spectrum_2d)  # [az_points, el_points]
        spec_array = np.nan_to_num(spec_array, nan=self.colorbar_vmin, posinf=self.colorbar_vmax, neginf=self.colorbar_vmin)
        
        # 更新主图: 2维热力图
        # 注意: imshow 的 extent=[xmin, xmax, ymin, ymax], origin='lower' 使y轴向上增长
        self.im_2d.set_data(spec_array.T)  # 转置: imshow 期望 [row, col] = [y, x]
        
        # 更新峰值标记
        if est_az is not None and est_el is not None:
            self.peak_cross_h.set_ydata([est_el, est_el])
            self.peak_cross_v.set_xdata([est_az, est_az])
            self.peak_dot.set_data([est_az], [est_el])
            self.text_peak.set_text(f'Peak: ({est_az:.1f}°, {est_el:.1f}°)')
        else:
            self.peak_cross_h.set_ydata([0, 0])
            self.peak_cross_v.set_xdata([0, 0])
            self.peak_dot.set_data([], [])
            self.text_peak.set_text('')

    def _safe_blit_draw(self):
        """安全降级绘制：仅更新必要元素"""
        try:
            # 主图
            self.ax_main.draw_artist(self.im_2d)
            self.ax_main.draw_artist(self.peak_cross_h)
            self.ax_main.draw_artist(self.peak_cross_v)
            self.ax_main.draw_artist(self.peak_dot)
            self.ax_main.draw_artist(self.text_peak)
            # 边际图
            self.ax_az_proj.draw_artist(self.line_az_proj)
            self.ax_az_proj.draw_artist(self.peak_az_marker)
            self.ax_el_proj.draw_artist(self.line_el_proj)
            self.ax_el_proj.draw_artist(self.peak_el_marker)
            # 整体刷新
            self.canvas.blit(self.fig.bbox)
        except:
            self.canvas.draw()

    def clear_data(self):
        """清空显示数据"""
        # 清空主图
        self.im_2d.set_data(np.zeros((128, 128)))
        self.peak_cross_h.set_ydata([0, 0])
        self.peak_cross_v.set_xdata([0, 0])
        self.peak_dot.set_data([], [])
        self.text_peak.set_text('')
        
        # 重置颜色条范围
        self.im_2d.set_norm(Normalize(vmin=self.colorbar_vmin, vmax=self.colorbar_vmax))
        
        # 重绘
        self.canvas.draw_idle()
