"""迭代滤波算法模块"""
import numpy as np
from collections import deque
from numba import jit
import tkinter as tk
from tkinter import ttk

class IterativeFilter:
    """迭代滤波算法类"""
    
    def __init__(self, sliding_window_size=5, iterations=5):
        self.sliding_window_size = sliding_window_size
        self.iterations = iterations  # 固定值，不在 UI 中修改
        self.windows = None
        self.current_num_subcarriers = 0
        
        # UI 绑定变量
        self._var_window_size = None
        self._spinbox_widget = None

    def setup_controls(self, parent, gui_ref):
        """
        动态创建控制面板控件：仅包含滑动窗口大小输入框
        :param parent: 父容器帧 (ttk.Frame)
        :param gui_ref: 主 GUI 实例引用
        :return: 创建的控件列表
        """
        widgets = []
        
        # 创建内部容器
        frame = ttk.Frame(parent)
        frame.pack(fill=tk.X, padx=5, pady=2)
        widgets.append(frame)
        
        # 标题标签
        lbl_title = ttk.Label(frame, text=f"[{self.__class__.__name__}] 窗口大小:", font=("微软雅黑", 9, "bold"))
        lbl_title.pack(side=tk.LEFT, padx=(0, 5))
        widgets.append(lbl_title)
        
        # 绑定变量
        self._var_window_size = tk.IntVar(value=self.sliding_window_size)
        
        # 创建 Spinbox (数字输入框)
        # from_: 最小值, to: 最大值, increment: 步进值
        self._spinbox_widget = ttk.Spinbox(
            frame, 
            from_=1, 
            to=50, 
            increment=1,
            textvariable=self._var_window_size,
            width=6,
            command=self._update_params,  # 点击上下箭头时触发
            validate="key"
        )
        # 绑定键盘输入事件 (当用户手动输入并回车或失去焦点时更新)
        self._spinbox_widget.bind("<Return>", lambda e: self._update_params())
        self._spinbox_widget.bind("<FocusOut>", lambda e: self._update_params())
        
        self._spinbox_widget.pack(side=tk.LEFT, padx=5)
        widgets.append(self._spinbox_widget)

        return widgets

    def _update_params(self):
        """
        当 UI 控件变动时，实时更新算法内部参数
        """
        if self._var_window_size is None:
            return
        try:
            new_val = self._var_window_size.get()
            if new_val != self.sliding_window_size:
                self.sliding_window_size = new_val
                self.clear()
                
        except Exception as e:
            if self._var_window_size:
                self._var_window_size.set(self.sliding_window_size)

    @staticmethod
    @jit(nopython=True, cache=True)
    def _interp_iterative(csi_matrix, iterations):
        """迭代插值 CSI 矩阵"""
        N, M = csi_matrix.shape
        weights = np.ones(N, dtype=np.float32) / N
        phi = np.zeros(N, dtype=np.float32)
        w = np.zeros(M, dtype=np.complex64)

        for _ in range(iterations):
            w[:] = 0
            for m in range(M):
                sum_val = 0j
                for n in range(N):
                    angle = -phi[n]
                    rot = complex(np.cos(angle), np.sin(angle))
                    sum_val += weights[n] * rot * csi_matrix[n, m]
                w[m] = sum_val
            
            for n in range(N):
                dot_val = 0j
                for m in range(M):
                    dot_val += np.conj(w[m]) * csi_matrix[n, m]
                phi[n] = np.angle(dot_val)
        return w
    
    def apply(self, full_data, antenna_order):
        """应用滑动窗口迭代滤波"""
        if not full_data or antenna_order[0] not in full_data:
            return full_data

        N_SC = full_data[antenna_order[0]]['subcarriers_nums']
        M = len(antenna_order)

        if self.windows is None or self.current_num_subcarriers != N_SC:
            self.windows = [deque(maxlen=self.sliding_window_size) for _ in range(N_SC)]
            self.current_num_subcarriers = N_SC

        for n in range(N_SC):
            try:
                csi_vector_n = np.array([
                    full_data[aid]['csi_complex'][n] for aid in antenna_order
                ], dtype=np.complex64)
                self.windows[n].append(csi_vector_n)
            except Exception:
                continue

        if all(len(w) == self.sliding_window_size for w in self.windows):
            h_hat_all = np.zeros((M, N_SC), dtype=np.complex64)
            for n in range(N_SC):
                csi_matrix_n = np.array(self.windows[n])
                h_hat_all[:, n] = self._interp_iterative(
                    csi_matrix_n.astype(np.complex64), iterations=self.iterations
                )
            
            for idx, aid in enumerate(antenna_order):
                if aid in full_data:
                    data = full_data[aid]
                    h_vec = h_hat_all[idx, :]
                    data['csi_complex'] = h_vec
                    data['csi_magnitude'] = np.abs(h_vec)
                    data['csi_phase'] = np.angle(h_vec)

        return full_data
    
    def clear(self):
        """清空滤波状态"""
        self.current_num_subcarriers = 0
        if self.windows:
            for w in self.windows:
                w.clear()
            self.windows = None 