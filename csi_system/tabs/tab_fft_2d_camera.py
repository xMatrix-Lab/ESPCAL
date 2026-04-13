"""FFT 波束空间 + 摄像头视场融合显示模块"""
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
import cv2
import sys

from core.csi_core_single import DisplayTab

class FFTBeamspaceCameraTab(DisplayTab):
    def __init__(self, notebook, title, camera_index=1, resolution_az=128, resolution_el=128):
        self.fps = 30
        self.update_interval = int(1000 / self.fps)

        self.AZ_RANGE = (-45, 45)
        self.EL_RANGE = (-20, 45)
        self.RAW_ANGLE = (-90, 90)

        self.EXTRA_SCALE = 1.5
        self.HORIZONTAL_SCALE = 0.4

        self.res_az = resolution_az
        self.res_el = resolution_el

        # ======================== 正确位置：TAB 内部初始化 GPU ========================
        self.gpu_available = False
        self.cuda_stream = None
        self.cuda_frame = None
        try:
            # 必须在这里初始化，不能在文件顶部！
            if cv2.cuda.getCudaEnabledDeviceCount() > 0:
                cv2.cuda.setDevice(0)
                self.gpu_available = True
                self.cuda_stream = cv2.cuda_Stream()
                self.cuda_frame = cv2.cuda_GpuMat()
                print("GPU启动成功")
        except Exception as e:
            self.gpu_available = False
            print("GPU不可用，自动使用CPU")

        # 摄像头
        self.cap = cv2.VideoCapture(camera_index, cv2.CAP_DSHOW)
        if not self.cap.isOpened():
            print(f"Warning: Camera {camera_index} could not be opened.")
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self.cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)

        self.contrast_power = 3.0
        self.noise_floor_clip = 0.01

        self.bg_img = None
        self.heatmap = None
        self.once_drawn = False

        super().__init__(notebook, title)

    def get_auto_aspect(self):
        az_span = self.AZ_RANGE[1] - self.AZ_RANGE[0]
        el_span = self.EL_RANGE[1] - self.EL_RANGE[0]
        return (az_span / el_span) * self.EXTRA_SCALE * self.HORIZONTAL_SCALE

    def crop_2d_spectrum(self, spec_2d, az_range, el_range):
        raw_min, raw_max = self.RAW_ANGLE
        total_az = spec_2d.shape[1]
        total_el = spec_2d.shape[0]

        az_start = int((az_range[0] - raw_min) / (raw_max - raw_min) * total_az)
        az_end   = int((az_range[1] - raw_min) / (raw_max - raw_min) * total_az)
        el_start = int((el_range[0] - raw_min) / (raw_max - raw_min) * total_el)
        el_end   = int((el_range[1] - raw_min) / (raw_max - raw_min) * total_el)

        az_start = max(0, az_start)
        az_end = min(total_az, az_end)
        el_start = max(0, el_start)
        el_end = min(total_el, el_end)
        return spec_2d[el_start:el_end, az_start:az_end]

    def setup_ui(self):
        plt.ioff()

        # ===================== 在这里强制启用硬件加速渲染 =====================
        import matplotlib
        matplotlib.rcParams['backend'] = 'TkAgg'
        matplotlib.rcParams['toolbar'] = 'None'
        # =====================================================================

        self.fig, self.ax_main = plt.subplots(1, 1, figsize=(10, 8))
        self.fig.tight_layout(pad=2.0)

        self.ax_main.set_xlim(*self.AZ_RANGE)
        self.ax_main.set_ylim(*self.EL_RANGE)
        self.ax_main.set_xlabel('Azimuth (°)', fontsize=10)
        self.ax_main.set_ylabel('Elevation (°)', fontsize=10)
        self.ax_main.grid(False)
        self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')
        self.ax_main.tick_params(axis='both', labelsize=9)

        self.bg_img = self.ax_main.imshow(
            np.zeros((480, 640, 3), dtype=np.uint8),
            extent=(*self.AZ_RANGE, *self.EL_RANGE),
            aspect='auto', zorder=1
        )

        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        self._update_camera_loop()

    def _update_camera_loop(self):
        ret, frame = self.cap.read()
        if ret:
            # ======================== TAB 内部 GPU 处理 ========================
            if self.gpu_available:
                try:
                    self.cuda_frame.upload(frame, self.cuda_stream)
                    flipped = cv2.cuda.flip(self.cuda_frame, 1)
                    frame_rgb = cv2.cuda.cvtColor(flipped, cv2.COLOR_BGR2RGB)
                    frame_rgb = frame_rgb.download(self.cuda_stream)
                except:
                    frame = cv2.flip(frame, 1)
                    frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            else:
                frame = cv2.flip(frame, 1)
                frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            # ====================================================================

            self.bg_img.set_data(frame_rgb)
            self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')

        self.canvas.draw_idle()
        self.frame.after(self.update_interval, self._update_camera_loop)

    def update_data(self, full_data):
        if not isinstance(full_data, dict):
            return
        if not full_data.get('fft2d_computed', False):
            return

        raw_power = full_data.get('raw_power_spectrum', None)
        if raw_power is None:
            spectrum_db = full_data.get('aoa_spectrum_2d', None)
            if spectrum_db is None:
                return
            spectrum_np = np.array(spectrum_db)
            if np.max(spectrum_np) > 100 or np.min(spectrum_np) < 0:
                min_val = np.min(spectrum_np[np.isfinite(spectrum_np)])
                raw_power = 10 ** ((spectrum_np - min_val) / 10.0)
            else:
                raw_power = spectrum_np
        else:
            raw_power = np.array(raw_power)

        if raw_power.shape != (self.res_el, self.res_az):
            raw_power = np.resize(raw_power, (self.res_el, self.res_az))

        cropped_power = self.crop_2d_spectrum(raw_power, self.AZ_RANGE, self.EL_RANGE)
        enhanced_power = cropped_power ** self.contrast_power
        max_enhanced = np.max(enhanced_power)

        if max_enhanced > 0:
            threshold = max_enhanced * self.noise_floor_clip
            enhanced_power[enhanced_power < threshold] = 0
            norm_power = enhanced_power / (max_enhanced + 1e-10)
        else:
            norm_power = np.zeros_like(enhanced_power)

        if self.heatmap is None:
            self.heatmap = self.ax_main.imshow(
                norm_power,
                extent=(*self.AZ_RANGE, *self.EL_RANGE),
                cmap='plasma', alpha=0.4, vmin=0, vmax=1.0,
                origin='lower', zorder=2
            )
            self.fig.colorbar(self.heatmap, ax=self.ax_main, shrink=0.7, label='Peak Power')
        else:
            self.heatmap.set_data(norm_power)
            self.heatmap.set_clim(0.9, 1.0)  # 只显示最强15%

        if not self.once_drawn:
            self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')
            self.once_drawn = True

        try:
            self.canvas.draw_idle()
        except:
            self.canvas.draw()

    def clear_data(self):
        if self.heatmap is not None:
            self.heatmap.set_data(np.zeros_like(self.heatmap.get_array()))
            self.heatmap.set_clim(0, 1)
        self.ax_main.set_aspect(self.get_auto_aspect(), adjustable='box')
        self.once_drawn = False
        self.canvas.draw_idle()

    def __del__(self):
        if hasattr(self, 'cap') and self.cap.isOpened():
            self.cap.release()