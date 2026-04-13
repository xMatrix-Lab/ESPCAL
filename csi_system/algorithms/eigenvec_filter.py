"""特征值插值算法模块"""
import numpy as np
from collections import deque
from numba import jit
import tkinter as tk
from tkinter import ttk


class EigenvecPerSubcarrierFilter:
    """
    每子载波特征值分解 CSI 插值算法
    对每个子载波独立计算协方差矩阵 → 主特征向量 → 相位参考 + 幅度校正
    """

    def __init__(self, sliding_window_size=5):
        # 窗口大小（与你原有算法保持一致接口）
        self.sliding_window_size = sliding_window_size
        self.windows = None
        self.current_num_subcarriers = 0

        # UI 绑定变量
        self._var_window_size = None
        self._spinbox_widget = None

    def setup_controls(self, parent, gui_ref):
        """
        创建控制面板：仅显示窗口大小（与接口统一）
        """
        widgets = []
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        widgets.append(frame)

        lbl_title = ttk.Label(frame, text=f"[{self.__class__.__name__}] 窗口大小:", font=("微软雅黑", 9, "bold"))
        lbl_title.pack(side=tk.LEFT, padx=(0, 5))
        widgets.append(lbl_title)

        self._var_window_size = tk.IntVar(value=self.sliding_window_size)

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

    def _update_params(self):
        """UI 参数更新"""
        if self._var_window_size is None:
            return
        try:
            new_val = self._var_window_size.get()
            if new_val != self.sliding_window_size:
                self.sliding_window_size = new_val
                self.clear()
        except Exception:
            self._var_window_size.set(self.sliding_window_size)

    @staticmethod
    def _csi_interp_eigenvec_per_subcarrier(csi: np.ndarray) -> np.ndarray:
        """
        输入 shape: (n_samples, n_antennas, n_subcarriers)
        输出 shape: (n_antennas, n_subcarriers)
        """
        antenna_shape = csi.shape[1:-1]
        n_subcarriers = csi.shape[-1]

        # 展平天线维度
        csi_flat = csi.reshape(csi.shape[0], -1, n_subcarriers)

        # 逐子载波协方差矩阵
        R = np.einsum("nas,nbs->sab", csi_flat, np.conj(csi_flat))

        # 特征值分解
        eigvals, eigvecs = np.linalg.eig(R)
        idx = np.argsort(np.abs(eigvals), axis=1)[:, ::-1]
        eigvals = np.take_along_axis(eigvals, idx, axis=1)
        eigvecs = np.take_along_axis(eigvecs, idx[:, np.newaxis, :], axis=2)

        # 主特征向量/值
        principal_eigenvectors = eigvecs[:, :, 0]
        principal_eigenvalues = eigvals[:, 0]

        # 相位参考 0 号天线 + 幅度校正
        result_flat = np.sqrt(principal_eigenvalues)[:, np.newaxis] * principal_eigenvectors
        result_flat = result_flat * np.exp(-1.0j * np.angle(principal_eigenvectors[:, 0][:, np.newaxis]))

        # 维度交换 + 恢复形状
        result_flat = np.swapaxes(result_flat, 0, 1)
        return result_flat.reshape(antenna_shape + (n_subcarriers,))

    def apply(self, full_data, antenna_order):
        """
        与你的 IterativeFilter 完全相同的接口
        滑动窗口 + 逐子载波特征值插值
        """
        if not full_data or antenna_order[0] not in full_data:
            return full_data

        N_SC = full_data[antenna_order[0]]['subcarriers_nums']

        # 初始化窗口
        if self.windows is None or self.current_num_subcarriers != N_SC:
            self.windows = [deque(maxlen=self.sliding_window_size) for _ in range(N_SC)]
            self.current_num_subcarriers = N_SC

        # 把当前时刻多天线 CSI 压入窗口
        for n in range(N_SC):
            try:
                csi_vector_n = np.array([
                    full_data[aid]['csi_complex'][n] for aid in antenna_order
                ], dtype=np.complex64)
                self.windows[n].append(csi_vector_n)
            except Exception:
                continue

        if all(len(w) == self.sliding_window_size for w in self.windows):
            # 构造窗口数据：(n_samples, n_antennas, n_subcarriers)
            csi_window = []
            for queue in self.windows:
                csi_window.append(np.array(queue))
            # 转置 → (n_samples, n_antennas, n_subcarriers)
            csi_matrix = np.stack(csi_window, axis=-1)

            # 调用特征值插值
            h_hat_all = self._csi_interp_eigenvec_per_subcarrier(csi_matrix)

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