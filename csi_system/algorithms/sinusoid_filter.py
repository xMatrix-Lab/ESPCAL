"""复正弦曲线拟合滤波"""
import numpy as np
from collections import deque
from numba import jit
import tkinter as tk
from tkinter import ttk


class ComplexSinusoidFitFilter:
    """
    复正弦曲线拟合滤波
    对每个天线的 CSI 频域响应拟合：H[k] = A * exp(j(φ₀ + ωk))
    实现超强降噪 + 线性相位校正 + STO 消除
    """

    def __init__(self, sliding_window_size=5):
        self.sliding_window_size = sliding_window_size  # 滑动窗口长度
        self.windows = None  # 子载波滑动窗口队列
        self.current_num_subcarriers = 0  # 当前子载波数量
        self.enabled = False  # 算法启用开关，默认关闭

        # UI 控件绑定变量
        self._var_window_size = None
        self._var_enabled = None  # 启用状态变量
        self._spinbox_widget = None
        self._checkbutton_widget = None

    def setup_controls(self, parent, gui_ref):
        """
        创建 GUI 控制面板：启用开关 + 窗口大小调节
        """
        widgets = []
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        widgets.append(frame)

        # 启用复选框
        self._var_enabled = tk.BooleanVar(value=self.enabled)
        self._checkbutton_widget = ttk.Checkbutton(
            frame,
            text="启用复正弦拟合滤波",
            variable=self._var_enabled,
            command=self._update_enabled
        )
        self._checkbutton_widget.pack(side=tk.LEFT, padx=(0, 10))
        widgets.append(self._checkbutton_widget)

        lbl_title = ttk.Label(
            frame,
            text=f"窗口大小:",
            font=("微软雅黑", 9, "bold")
        )
        lbl_title.pack(side=tk.LEFT, padx=(0, 5))
        widgets.append(lbl_title)

        # 窗口大小变量
        self._var_window_size = tk.IntVar(value=self.sliding_window_size)

        # 数字调节框
        self._spinbox_widget = ttk.Spinbox(
            frame,
            from_=1,
            to=50,
            increment=1,
            textvariable=self._var_window_size,
            width=6,
            command=self._update_params,
            validate="key"
        )
        self._spinbox_widget.bind("<Return>", lambda e: self._update_params())
        self._spinbox_widget.bind("<FocusOut>", lambda e: self._update_params())
        self._spinbox_widget.pack(side=tk.LEFT, padx=5)
        widgets.append(self._spinbox_widget)

        return widgets

    def _update_enabled(self):
        """更新启用状态"""
        if self._var_enabled is not None:
            self.enabled = self._var_enabled.get()
            # 禁用时清空历史窗口，避免残留数据影响
            if not self.enabled:
                self.clear()

    def _update_params(self):
        """UI 数值变化时更新参数"""
        if self._var_window_size is None:
            return
        try:
            new_val = self._var_window_size.get()
            if new_val != self.sliding_window_size:
                self.sliding_window_size = new_val
                self.clear()  # 参数改变，清空历史窗口
        except Exception:
            # 输入非法，恢复原值
            self._var_window_size.set(self.sliding_window_size)

    @staticmethod
    def fit_complex_sinusoid(csi_data: np.ndarray) -> np.ndarray:
        """
        输入 shape: (n_samples, n_antennas, n_subcarriers)
        输出 shape: (n_antennas, n_subcarriers)
        """
        n_subcarriers = csi_data.shape[-1]
        k = np.arange(n_subcarriers)

        # 估计相位斜率 ω
        phase_diff = csi_data[..., 1:] * np.conj(csi_data[..., :-1])
        omega = np.angle(np.sum(phase_diff, axis=-1))  # (n_samples, n_antennas)

        # 对窗口内所有样本取平均斜率（适配滑动窗口）
        omega = np.mean(omega, axis=0)  # (n_antennas,)

        # 去旋转 + 估计复幅度 A·exp(jφ₀)
        derotated = csi_data * np.exp(-1.0j * omega[np.newaxis, ..., np.newaxis] * k)
        complex_amplitude = np.mean(derotated, axis=(0, -1))  # (n_antennas,)

        # 重构拟合后的纯净 CSI
        fitted = complex_amplitude[..., np.newaxis] * np.exp(1.0j * omega[..., np.newaxis] * k)
        return fitted

    def apply(self, full_data, antenna_order):
        """
        统一接口
        """
        # 关键：未启用时直接返回原始数据，不做任何处理
        if not self.enabled or not full_data or antenna_order[0] not in full_data:
            return full_data

        N_SC = full_data[antenna_order[0]]['subcarriers_nums']

        # 初始化滑动窗口
        if self.windows is None or self.current_num_subcarriers != N_SC:
            self.windows = [deque(maxlen=self.sliding_window_size) for _ in range(N_SC)]
            self.current_num_subcarriers = N_SC

        # 将当前帧多天线 CSI 压入窗口
        for n in range(N_SC):
            try:
                csi_vector_n = np.array([
                    full_data[aid]['csi_complex'][n] for aid in antenna_order
                ], dtype=np.complex64)
                self.windows[n].append(csi_vector_n)
            except Exception:
                continue

        if all(len(w) == self.sliding_window_size for w in self.windows):
            # 构造窗口数据 shape: (n_samples, n_antennas, n_subcarriers)
            csi_stack = [np.array(queue) for queue in self.windows]
            csi_matrix = np.stack(csi_stack, axis=-1)

            h_hat_all = self.fit_complex_sinusoid(csi_matrix)

            for idx, aid in enumerate(antenna_order):
                if aid in full_data:
                    data = full_data[aid]
                    h_vec = h_hat_all[idx, :]
                    data['csi_complex'] = h_vec
                    data['csi_magnitude'] = np.abs(h_vec)
                    data['csi_phase'] = np.angle(h_vec)

        return full_data

    def clear(self):
        """清空状态"""
        self.current_num_subcarriers = 0
        if self.windows:
            for w in self.windows:
                w.clear()
            self.windows = None