"""数据采集与导出标签页模块"""
import numpy as np
import tkinter as tk
import csv
from scipy.io import savemat
from tkinter import ttk, filedialog, messagebox
from core.csi_core_single import DisplayTab


class DataCollectTab(DisplayTab):
    """CSI 数据采集与导出（无限缓存 + 自动维度重构 + 支持 mat/csv）"""

    def __init__(self, notebook, title):
        self.data_buffer = []  # 无限缓存，不设上限
        super().__init__(notebook, title)

    def setup_ui(self):
        # 状态显示
        top_frame = ttk.Frame(self.frame)
        top_frame.pack(fill=tk.X, padx=10, pady=10)

        self.status_label = ttk.Label(
            top_frame,
            text="已采集：0 个数据点",
            font=("Arial", 11)
        )
        self.status_label.pack(side=tk.LEFT, padx=5)

        # 按钮区域
        btn_frame = ttk.Frame(self.frame)
        btn_frame.pack(fill=tk.X, padx=10, pady=5)

        # 导出按钮
        ttk.Button(btn_frame, text="导出 .dat", command=lambda: self.export("dat")).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="导出 .npz", command=lambda: self.export("npz")).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="导出 .mat", command=lambda: self.export("mat")).pack(side=tk.LEFT, padx=5)
        ttk.Button(btn_frame, text="导出 .csv", command=lambda: self.export("csv")).pack(side=tk.LEFT, padx=5)
        
        # 清空按钮
        ttk.Button(btn_frame, text="清空数据", command=self.clear_all).pack(side=tk.LEFT, padx=5)

    def update_data(self, full_data):
        """接收一帧数据，直接存入缓存（无限存）"""
        if not isinstance(full_data, dict):
            return full_data

        try:
            # 取出 CSI 矩阵 (Total_Ant, Subcarrier)
            antenna_order = full_data['antenna_order']
            csi_matrix = np.array([full_data[aid]['csi_complex'] for aid in antenna_order])
            self.data_buffer.append(csi_matrix)
        except Exception:
            pass  # 静默失败，避免阻塞主线程

        # 更新计数
        self.status_label.config(text=f"已采集：{len(self.data_buffer)} 个数据点")
        return full_data

    def _get_reshaped_data(self):
        """
        维度重构：(时间，总天线数，子载波) -> (时间，4 行，列天线，子载波)
        """
        if len(self.data_buffer) == 0:
            return None
        
        raw = np.array(self.data_buffer)  # (Time, Ant, Sub)
        if raw.shape[1] % 4 != 0:
            messagebox.showerror("错误", "天线总数必须是 4 的倍数才能进行 4D 重构！")
            return None
            
        return raw.reshape(raw.shape[0], 4, -1, raw.shape[-1])

    def export(self, fmt):
        """导出已经 reshape 好的 4D 张量"""
        data = self._get_reshaped_data()
        
        if data is None or len(data) == 0:
            messagebox.showwarning("提示", "没有数据可导出！")
            return

        filepath = ""
        try:
            if fmt == "dat":
                filepath = filedialog.asksaveasfilename(defaultextension=".dat", filetypes=[("二进制文件", "*.dat")])
                if filepath:
                    data.astype(np.complex64).tofile(filepath)

            elif fmt == "npz":
                filepath = filedialog.asksaveasfilename(defaultextension=".npz", filetypes=[("NumPy 归档", "*.npz")])
                if filepath:
                    np.savez(filepath, csi_data=data)

            elif fmt == "mat":
                filepath = filedialog.asksaveasfilename(defaultextension=".mat", filetypes=[("MATLAB 文件", "*.mat")])
                if filepath:
                    savemat(filepath, {'csi_data': data.astype(np.complex128)})

            elif fmt == "csv":
                filepath = filedialog.asksaveasfilename(defaultextension=".csv", filetypes=[("CSV 文件", "*.csv")])
                if filepath:
                    self._export_csv(filepath, data)

            if filepath:
                messagebox.showinfo("成功", f"导出完成！\n路径：{filepath}\n最终形状：{data.shape}")

        except Exception as e:
            messagebox.showerror("导出失败", f"发生错误：{str(e)}")

    def _export_csv(self, filepath, data):
        """
        专用 CSV 导出：复数字符串 + 正确转义的表头
        表头格式：data[行，列，子载波]，不含时间维
        """
        T, R, C, S = data.shape  # 时间，4 行，列天线，子载波
        
        # 1. 生成表头（时间索引 + 各位置复数列）
        headers = ["Time_Index"]
        for r in range(R):
            for c in range(C):
                for s in range(S):
                    # 表头不含时间维，格式：data[0,1,3]
                    headers.append(f"data[{r},{c},{s}]")
        
        # 2. 使用 csv.writer 自动处理逗号转义
        with open(filepath, 'w', newline='', encoding='utf-8') as f:
            writer = csv.writer(f)
            writer.writerow(headers)  # 自动给含逗号的表头加双引号
            
            # 3. 逐行写入数据
            for t in range(T):
                row = [t]  # 时间索引
                for r in range(R):
                    for c in range(C):
                        for s in range(S):
                            val = data[t, r, c, s]
                            # 格式：1.234567+2.345678j 或 1.234567-2.345678j
                            # :+.6f 保证虚部始终带符号，方便解析
                            row.append(f"{val.real:.6f}{val.imag:+.6f}j")
                writer.writerow(row)

    def clear_all(self):
        self.data_buffer.clear()
        self.status_label.config(text="已采集：0 个数据点")

    def clear_data(self):
        self.clear_all()