"""AOA 2D 摄像头视场 1.5m距离"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import cv2

from core.csi_core_single import DisplayTab

class MUSIC2D_Carame_Tab(DisplayTab):
    def __init__(self, notebook, title, camera_index=1):
        self.fps = 60
        self.update_interval = int(1000 / self.fps)

        # 显示的物理角度范围
        self.AZ_RANGE = (-90, 90)    # 方位显示范围
        self.EL_RANGE = (-10, 30)    # 俯仰显示范围
        self.RAW_ANGLE = (-90, 90)

        # 垂直放大系数
        self.EXTRA_SCALE = 1.5
        # 水平放大系数，越大 = 画面越窄
        self.HORIZONTAL_SCALE = 0.4

        # 摄像头
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.heatmap = None
        self.bg_img = None
        self.peak_marker = None

        super().__init__(notebook, title)

    def get_auto_aspect(self):
        az_span = self.AZ_RANGE[1] - self.AZ_RANGE[0]
        el_span = self.EL_RANGE[1] - self.EL_RANGE[0]
        return (az_span / el_span) * self.EXTRA_SCALE * self.HORIZONTAL_SCALE

    def crop_spectrum(self, spec_full, target_range):
        raw_min, raw_max = self.RAW_ANGLE
        tar_min, tar_max = target_range
        
        total_pts = len(spec_full)
        idx_start = int((tar_min - raw_min) / (raw_max - raw_min) * total_pts)
        idx_end = int((tar_max - raw_min) / (raw_max - raw_min) * total_pts)
        
        return spec_full[idx_start:idx_end]

    def setup_ui(self):
        plt.ioff()
        self.fig, self.ax_main = plt.subplots(1, 1, figsize=(10, 8))

        self.ax_main.set_xlim(*self.AZ_RANGE)
        self.ax_main.set_ylim(*self.EL_RANGE)
        self.ax_main.set_xlabel('Azimuth °')
        self.ax_main.set_ylabel('Elevation °')
        self.ax_main.grid(False)
        self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')

        # 摄像头背景
        self.bg_img = self.ax_main.imshow(
            np.zeros((480, 640, 3), dtype=np.uint8),
            extent=(*self.AZ_RANGE, *self.EL_RANGE),
            aspect='auto', zorder=1
        )

        self.peak_marker, = self.ax_main.plot([], [], 'r+', markersize=12, zorder=3)

        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        self._update_camera_loop()

    def _update_camera_loop(self):
        ret, frame = self.cap.read()
        if ret:
            # 需要镜像就打开
            # frame = cv2.flip(frame, 1)
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            self.bg_img.set_data(frame_rgb)
            self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')

        self.canvas.draw_idle()
        self.frame.after(self.update_interval, self._update_camera_loop)

    def update_data(self, full_data):
        if not full_data.get('music2d_computed', False):
            return

        az_spec_full = full_data['aoa_spectrum_azimuth']
        el_spec_full = full_data['aoa_spectrum_elevation']
        az = full_data['aoa_azimuth']
        el = full_data['aoa_elevation']

        az_spec = self.crop_spectrum(az_spec_full, self.AZ_RANGE)
        el_spec = self.crop_spectrum(el_spec_full, self.EL_RANGE)

        spec_2d = np.outer(el_spec, az_spec)

        if self.heatmap is None:
            self.heatmap = self.ax_main.imshow(
                spec_2d,
                extent=(*self.AZ_RANGE, *self.EL_RANGE),
                cmap='jet', alpha=0.4, vmin=-5, vmax=5,
                origin='lower', zorder=2
            )
            self.fig.colorbar(self.heatmap, ax=self.ax_main, shrink=0.7)
        else:
            self.heatmap.set_data(spec_2d)

        self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')
        self.peak_marker.set_data([az], [el])
        self.canvas.draw_idle()

    def clear_data(self):
        if self.heatmap is not None:
            self.heatmap.set_data(np.zeros_like(self.heatmap.get_array()))
        self.peak_marker.set_data([], [])
        self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')
        self.canvas.draw_idle()

    def __del__(self):
        self.cap.release()