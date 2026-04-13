import tkinter as tk
from tkinter import ttk, messagebox
import socket
import threading
import struct
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
from collections import deque
from abc import ABC, abstractmethod
from numba import jit  
from scipy.linalg import eig


tab_enabled = {}


# ================================== 计算加速 ==================================
@jit(nopython=True, cache=True)
def _core_music_2d(A, QnH, N_el, N_az):
    """MUSIC 2D 核心计算加速"""
    spectrum = np.empty((N_el, N_az), dtype=np.float64)
    # 展开循环以获得最大加速比
    for i in range(N_el):
        for j in range(N_az):
            # 提取导向矢量: (8,)
            a = A[i, j, :]
            # 矩阵乘法: (7, 8) x (8,) -> (7,)
            # 手动实现 QnH @ a
            temp_res = np.zeros(7, dtype=np.complex128)
            for r in range(7):
                acc = 0j
                for c in range(8):
                    acc += QnH[r, c] * a[c]
                temp_res[r] = acc
            
            # 计算范数平方: sum(abs(x)^2)
            norm_sq = 0.0
            for k in range(7):
                real = temp_res[k].real
                imag = temp_res[k].imag
                norm_sq += real*real + imag*imag
            
            if norm_sq < 1e-12:
                spectrum[i, j] = 100.0 # 避免除以零
            else:
                spectrum[i, j] = -10.0 * np.log10(norm_sq)
    return spectrum

@jit(nopython=True, cache=True)
def _core_iterative_filter(csi_matrix, iterations):
    """迭代滤波核心计算加速版"""
    N, M = csi_matrix.shape
    weights = np.ones(N, dtype=np.float32) / N
    phi = np.zeros(N, dtype=np.float32)
    w = np.zeros(M, dtype=np.complex64)
    
    for _ in range(iterations):
        # 1. 加权求和
        w[:] = 0
        for m in range(M):
            sum_val = 0j
            for n in range(N):
                # weights[n] * exp(-j * phi[n]) * csi_matrix[n, m]
                # exp(-j*x) = cos(x) - j*sin(x)
                angle = -phi[n]
                rot = complex(np.cos(angle), np.sin(angle))
                sum_val += weights[n] * rot * csi_matrix[n, m]
            w[m] = sum_val
            
        # 2. 更新相位 phi
        for n in range(N):
            # dot(w.conj, csi_matrix[n])
            dot_val = 0j
            for m in range(M):
                dot_val += np.conj(w[m]) * csi_matrix[n, m]
            phi[n] = np.angle(dot_val)
            
    return w
# ==============================================================================

class CSIDataProcessor():

    # ====== 配置参数（可修改） ======
    timestamp_scale = 10000

    ENABLE_RSSI_WEIGHTING = True   # 是否启用RSSI权重  

    ENABLE_CIR_SYNC = False  # 是否启用 CIR 峰值同步（在插值前）
    CIR_MAX_DELAY_TAPS = 3
    CIR_SEARCH_RES = 40
    CIR_PEAK_THRESHOLD = 0.5

    SLIDING_WINDOW_SIZE = 3     # 滑动窗口长度 T
    ENABLE_EIGENVEC_FILTERING = False     # <<< 协方差插值主开关：True=启用，False=禁用
    ENABLE_ITERATIVE_FILTERING = True     # <<< 迭代插值主开关：True=启用，False=禁用
    ITERATIONS = 5 # 迭代次数

    MUSIC_BUFFER_SIZE = 1
    # =============================

    def __init__(self):
        self.antenna_mapping = {
            7: "00", 4: "01", 5: "02", 6: "03",
            0: "10", 1: "11", 2: "12", 3: "13"
        }
        self.expected_antennas = set(self.antenna_mapping.values())
        self.antenna_data = {}
        self.last_coarse_timestamp = None 
        # 固定天线顺序
        self.antenna_order = sorted(self.expected_antennas)
        self.M = len(self.antenna_order)

        self.wifi_protocol = 'LLTF'

        # 校准相关参数
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None # 参考校准数据  
        self.h_ref_origin = None  # 原始参考校准数据
        self.lltf_frequencys = None
        self.ht40_frequencys = None
        self.ht40_center_freq = None
        self.lltf_center_freq = None


        # === 协方差插值状态 ===
        self.eigenvec_windows = None
        # === 迭代插值状态 ===
        self.iterative_windows = None


        # MUSIC1D
        self.music_buffer = []  # 存储完整帧数据
        # 初始化MUSIC扫描角度、导向矢量等
		# 扫描角度范围：-90度到90度，共180个点
        self.scanning_angles = np.linspace(-np.pi / 2, np.pi / 2, 180)
		# 计算导向矢量：exp(-j * π * sin(θ) * n)，其中n是天线索引
        self.steering_vectors = np.exp(-1.0j * np.outer(np.pi * np.sin(self.scanning_angles), np.arange(4)))


        # MUSIC2D
        self.music2d_buffer = []
        # 扫描网格
        self.azimuths_2d_deg = np.arange(-90, 91, 2)  
        self.elevations_2d_deg = np.arange(-90, 91, 2)
        self.azimuths_2d = np.deg2rad(self.azimuths_2d_deg)
        self.elevations_2d = np.deg2rad(self.elevations_2d_deg)
        # 天线物理位置映射（2x4 阵列）
        self.d = 0.5  # 天线间距（以波长为单位）
        # 天线位置定义
        self.n_vals = np.array([0, 0, 0, 0, 1, 1, 1, 1])  
        self.m_vals = np.array([0, 1, 2, 3, 0, 1, 2, 3]) 
        # 使用广播计算所有导向矢量
        # 创建维度：(n_el, n_az, 8)
        elevations_3d = self.elevations_2d[:, np.newaxis, np.newaxis]  # (n_el, 1, 1)
        azimuths_3d = self.azimuths_2d[np.newaxis, :, np.newaxis]      # (1, n_az, 1)
        # 天线位置矩阵
        m_vals_3d = self.m_vals[np.newaxis, np.newaxis, :]  # (1, 1, 8)
        n_vals_3d = self.n_vals[np.newaxis, np.newaxis, :]  # (1, 1, 8)
        # 计算相位phase = 2π * d * [n*sin(θ)cos(φ) + m*sin(θ)sin(φ)]
        sin_theta = np.sin(elevations_3d)
        cos_phi = np.cos(azimuths_3d)
        sin_phi = np.sin(azimuths_3d)
        phase = 2 * np.pi * self.d * (
            n_vals_3d * sin_theta * cos_phi + 
            m_vals_3d * sin_theta * sin_phi
        )
        self.steering_vectors_2d = np.exp(1j * phase)  # (n_el, n_az, 8)

        #TDOA
        self.tdoa_buffer = []


    def calculate_pcb_phase_offsets(self, channel_primary: int, channel_secondary: int):
        """
        channel_primary: 主信道号
        channel_secondary: 副信道编号,1表示在上方,-1表示在下方,0表示不使用
        """
        epsilon_r = 4.2  # FR4介电常数
        h = 8.28  # 介质厚度 (mil)
        w = 13.8  # 线宽 (mil)
        ch1_frequency = 2.412e9  # WiFi信道1频率
        ch_gap = 5.0e6 # WiFi信道频率间隔
        subcarriar_gap = 312.5e3 # 子载波频率间隔
        mil_to_meter = 25.4e-6 # mil转m系数
        c = 299792458 # 真空光速
        pcb_path_lengths = np.array([
            3931.8, 1466.0, 1459.2, 3926.5,
            3916.1, 1457.7, 1471.8, 3918.1
        ]) # 天线路径长度差异 (mil)
        epsilon_eff = (epsilon_r + 1)/2 + (epsilon_r - 1)/2 * (1 + 12*h/w)**(-0.5) # 有效介电常数
        v_group = c / epsilon_eff**0.5 # 信号群速度
        # LLTF所有子载波的频率数组
        self.lltf_center_freq = ch1_frequency + ch_gap * (channel_primary - 1)
        self.lltf_frequencys = self.lltf_center_freq + np.arange(-32, 32) * subcarriar_gap
        # HTLTF所有子载波的频率数组
        center_primary = ch1_frequency + ch_gap * (channel_primary - 1)
        center_secondary = ch1_frequency + ch_gap * (channel_primary + 4*channel_secondary - 1)
        self.ht40_center_freq = (center_primary + center_secondary) / 2
        self.ht40_frequencys = self.ht40_center_freq + np.arange(-64, 64) * subcarriar_gap
        # 波长组
        lltf_wavelengths = v_group / self.lltf_frequencys
        ht40_wavelengths = v_group / self.ht40_frequencys
        # 计算PCB走线导致的相位偏移
        tracelengths = pcb_path_lengths * mil_to_meter
        self.prop_calib_each_board_lltf = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / lltf_wavelengths[np.newaxis, :])
        self.prop_calib_each_board_ht40 = np.exp(-1.0j * 2 * np.pi * tracelengths[:,np.newaxis] / ht40_wavelengths[np.newaxis, :])


    def _calculate_aoa_1d(self, full_data):
        """
        使用MUSIC算法计算1D到达角（方位角）
        """
        csi_matrix = np.array([
            full_data[aid]['csi_complex'] for aid in self.antenna_order
        ])  # 形状: (8, K)
        self.music_buffer.append(csi_matrix)
        
        if len(self.music_buffer) < self.MUSIC_BUFFER_SIZE:
            full_data['aoa'] = None
            full_data['aoa_spectrum'] = None
            full_data['music_computed'] = False
            return full_data
        
        csi_backlog = np.array(self.music_buffer)  # 形状: (buffer_size, 8, K)
        
        # 提取LOS分量（沿子载波轴求和）
        # csi_shifted_los = np.sum(csi_backlog, axis=-1)  # 形状: (buffer_size, 8)
        csi_shifted_los = csi_backlog[:, :, 40]  # 直接使用一个子载波作为LOS分量，形状: (buffer_size, 8)
        R = np.einsum("ti,tj->ij", csi_shifted_los, np.conj(csi_shifted_los))
        # 其中：t = 时间样本 (buffer_size), i/j = 天线索引 (8)

        # 标准化（除以样本数）
        R = R / len(csi_shifted_los)
        
        # 特征值分解
        eig_val, eig_vec = np.linalg.eigh(R)
        order = np.argsort(eig_val)[::-1]  # 降序排序
        eig_vec_sorted = eig_vec[:, order]
        
        # 构建噪声子空间（假设只有1个信号源）
        Qn = eig_vec_sorted[:, 1:]  # 形状: (8, 7)
        
        # 使用4个天线的导向矢量（假设使用第一行的4个天线）
        num_antennas = 4
        steering_vectors = np.exp(-1.0j * np.outer(
            np.pi * np.sin(self.scanning_angles), 
            np.arange(num_antennas)
        ))  # 形状: (180, 4)
        
        # 只使用前4个天线
        R_4 = R[:4, :4]
        Qn_4 = Qn[:4, :]  # 只使用前4个天线
        
        # 重新计算特征分解（使用4天线子集）
        eig_val_4, eig_vec_4 = np.linalg.eig(R_4)
        order_4 = np.argsort(eig_val_4)[::-1]
        Qn_4 = eig_vec_4[:, order_4][:, 1:]  # 噪声子空间: (4, 3)
        
        # 计算 Qn^H * a(θ) for each θ
        # steering_vectors: (180, 4), Qn_4: (4, 3)
        # 计算: (3, 4) @ (4, 180) -> (3, 180) 然后取范数
        QnH = np.conj(Qn_4).T  # (3, 4)
        QnH_a = QnH @ steering_vectors.T  # (3, 4) @ (4, 180) = (3, 180)
        
        # 计算谱：1 / ||Qn^H * a(θ)||^2 for each θ
        norms = np.linalg.norm(QnH_a, axis=0)  # (180,)
        spatial_spectrum_linear = 1 / (norms ** 2)
        
        # 转换为对数刻度（dB）
        spatial_spectrum_log = 20 * np.log10(spatial_spectrum_linear + 1e-10)
        peak_idx = np.argmax(spatial_spectrum_log)
        aoa_angle = np.rad2deg(self.scanning_angles[peak_idx])

        full_data['aoa'] = aoa_angle
        full_data['aoa_spectrum'] = spatial_spectrum_log.tolist()
        full_data['aoa_angles'] = np.rad2deg(self.scanning_angles).tolist()
        full_data['music_computed'] = True

        self.music_buffer = []
        return full_data
    
    def _calculate_aoa_2d(self, full_data):
        """
        使用MUSIC算法计算2D到达角（方位角 + 仰角）
        2x4阵列：00 01 10 11 20 21 30 31（索引对应：0→00,1→01,2→10,3→11,4→20,5→21,6→30,7→31）
        输入：full_data - 包含各天线CSI复数值的字典
        输出：更新后的full_data（添加方位角/仰角/功率谱）
        """
        # ========== 1. 系统参数配置（适配2×4阵列） ==========
        Fc = 2.42e9  # WiFi 5G频段中心频率（可根据实际硬件调整）
        c = 3e8      # 光速 (m/s)
        wavelength = c / Fc  # 波长 (m)
        d = wavelength / 2   # 阵元间距（半波长，阵列信号处理经典配置）
        Mx, My = 4, 2        # 2×4均匀面阵（My=2列，Mx=4行，对应天线顺序：00 01 10 11 20 21 30 31）
        source_num = 1       # 信号源数量（默认1个，可根据场景调整）
        
        # 角度搜索范围（与显示界面匹配）
        theta_vec = np.arange(0, 90.1, 0.5)  # 仰角：0~90°，步长0.5°
        phi_vec = np.arange(-180, 180.1, 0.5) # 方位角：-180~180°，步长0.5°
        
        # ========== 2. 提取并预处理CSI数据 ==========
        # 提取8个天线的CSI复数值 (形状: (8, K), K是子载波数)
        try:
            csi_matrix = np.array([
                full_data[aid]['csi_complex'] for aid in self.antenna_order
            ], dtype=np.complex64)  # (8, K)
        except KeyError:
            # 天线索引缺失，返回空值
            for aid in self.antenna_order:
                full_data[aid]['azimuth'] = np.nan
                full_data[aid]['elevation'] = np.nan
                full_data[aid]['music_spectrum'] = np.array([])
            return full_data
        
        # 移除全零/噪声子载波（避免协方差矩阵奇异）
        non_zero_idx = np.where(np.sum(np.abs(csi_matrix), axis=0) > 1e-6)[0]
        if len(non_zero_idx) == 0:
            # 无有效CSI数据
            for aid in self.antenna_order:
                full_data[aid]['azimuth'] = np.nan
                full_data[aid]['elevation'] = np.nan
                full_data[aid]['music_spectrum'] = np.array([])
            return full_data
        csi_matrix = csi_matrix[:, non_zero_idx]  # (8, K_valid)
        
        # ========== 3. 计算空间协方差矩阵 ==========
        num_packets = csi_matrix.shape[1]
        # 协方差矩阵：E[X*X^H]，H表示共轭转置（样本均值近似期望）
        R = csi_matrix @ csi_matrix.conj().T / num_packets  # (8, 8)
        
        # ========== 4. MUSIC核心：特征值分解（分离信号/噪声子空间） ==========
        eig_vals, eig_vecs = eig(R)
        # 特征值降序排序
        eig_idx = np.argsort(np.abs(eig_vals))[::-1]
        eig_vals = eig_vals[eig_idx]
        eig_vecs = eig_vecs[:, eig_idx]
        # 噪声子空间：取后 Mx*My - source_num 个特征向量
        noise_subspace = eig_vecs[:, source_num:]  # (8, 8-source_num)
        
        # ========== 5. 2D MUSIC功率谱搜索（核心） ==========
        music_spectrum = np.zeros((len(theta_vec), len(phi_vec)), dtype=np.float64)
        
        # 遍历所有搜索角度，计算功率谱
        for theta_idx, theta_deg in enumerate(theta_vec):
            theta = np.deg2rad(theta_deg)  # 仰角转弧度
            for phi_idx, phi_deg in enumerate(phi_vec):
                phi = np.deg2rad(phi_deg)    # 方位角转弧度
                
                # 构造2×4 URA的导向矢量a(theta, phi)
                a = np.zeros(Mx*My, dtype=np.complex64)
                # 天线顺序：00(0) 01(1) 10(2) 11(3) 20(4) 21(5) 30(6) 31(7)
                # 对应：行m(0-3)，列n(0-1) → 索引 = m*My + n
                for m in range(Mx):  # 行：0,1,2,3（对应30/20/10/00）
                    for n in range(My):  # 列：0,1（对应x0/x1）
                        idx = m * My + n  # 天线索引
                        # URA导向矢量公式：exp(-j*2π/λ*(m*d*sinθcosφ + n*d*sinθsinφ))
                        phase = 2 * np.pi / wavelength * d * np.sin(theta) * (
                            m * np.cos(phi) + n * np.sin(phi)
                        )
                        a[idx] = np.exp(-1j * phase)
                
                # MUSIC功率谱：1 / ||a^H * 噪声子空间||²
                a_H = a.conj().T
                denominator = np.linalg.norm(a_H @ noise_subspace) **2
                if denominator < 1e-12:
                    music_spectrum[theta_idx, phi_idx] = 0
                else:
                    music_spectrum[theta_idx, phi_idx] = 1 / denominator
        
        # 功率谱转dB（归一化到最大值0dB）
        music_spectrum_dB = 10 * np.log10(music_spectrum / np.max(music_spectrum))
        
        # ========== 6. 峰值提取（估计角度） ==========
        # 找功率谱全局峰值对应的角度
        peak_idx = np.unravel_index(np.argmax(music_spectrum), music_spectrum.shape)
        elevation_est = theta_vec[peak_idx[0]]  # 估计仰角 (°)
        azimuth_est = phi_vec[peak_idx[1]]      # 估计方位角 (°)
        
        # 方位角归一化（-180~180°）
        if azimuth_est > 180:
            azimuth_est -= 360
        elif azimuth_est < -180:
            azimuth_est += 360
        
        # ========== 7. 结果写入full_data（适配显示界面） ==========
        for aid in self.antenna_order:
            full_data[aid]['azimuth'] = azimuth_est
            full_data[aid]['elevation'] = elevation_est
            full_data[aid]['music_spectrum'] = music_spectrum_dB  # 2D功率谱
            # 可选：写入角度轴（供显示界面参考）
            full_data[aid]['aoa_azimuths'] = phi_vec
            full_data[aid]['aoa_elevations'] = theta_vec
        
        return full_data
    
    def _estimate_tdoa_rootmusic(self, full_data):
        """
        使用 root-MUSIC 在频域估计每根天线的 ToA（LoS 路径）
        利用：8 天线作为“快拍”，子载波作为“阵元”
        """
        csi_matrix = np.array([
            full_data[aid]['csi_complex'] for aid in self.antenna_order
        ])  # (8, K)

        self.tdoa_buffer.append(csi_matrix)

        if len(self.tdoa_buffer) < self.MUSIC_BUFFER_SIZE:
            full_data['tdoa_computed'] = False
            return full_data

        # 形状: (T, 8, K)
        csi_backlog = np.array(self.tdoa_buffer)
        T, M, K = csi_backlog.shape

        # === 每根天线单独估计 ToA ===
        toa_sec = {}
        tdoa_ns = {}

        for idx, aid in enumerate(self.antenna_order):
            # 提取该天线的所有快拍: (T, K)
            csi_ant = csi_backlog[:, idx, :]  # (T, K)

            # 构建协方差矩阵: R = E[ x x^H ]，x 是频域CSI (K,)
            R = np.zeros((K, K), dtype=np.complex64)
            for t in range(T):
                x = csi_ant[t, :].reshape(-1, 1)  # (K, 1)
                R += x @ x.conj().T
            R /= T

            # 前后向平滑（提升分辨率）
            R = (R + np.flip(np.conj(R), axis=(0, 1))) / 2

            # 特征分解
            try:
                eigvals, eigvecs = np.linalg.eigh(R)
            except np.linalg.LinAlgError:
                toa = 0.0
            else:
                # 降序排序
                order = np.argsort(eigvals)[::-1]
                eigvals = eigvals[order]
                eigvecs = eigvecs[:, order]

                # MDL 准则估计源数（简化：固定 1~2）
                num_signals = 1
                num_noise = K - num_signals
                if num_noise <= 0:
                    toa = 0.0
                else:
                    # 噪声子空间
                    Qn = eigvecs[:, num_signals:]  # (K, num_noise)
                    C = Qn @ Qn.conj().T           # (K, K)

                    # 构造自相关系数（Toeplitz 对角线）
                    coeffs = np.array([np.trace(C, offset=d) for d in range(1, K)])
                    # 构造共轭对称多项式
                    poly_coeffs = np.concatenate([coeffs[::-1], [np.trace(C)], np.conj(coeffs)])

                    # 求根
                    roots = np.roots(poly_coeffs)
                    # 保留单位圆内根
                    roots = roots[np.abs(roots) < 1]
                    if len(roots) == 0:
                        toa = 0.0
                    else:
                        # 计算功率：1 / (1 - |root|)
                        powers = 1.0 / (1.0 - np.abs(roots))
                        strongest = np.argsort(powers)[::-1][:num_signals]
                        delays_sec = -np.angle(roots[strongest]) / (2 * np.pi * 312.5e3)
                        toa = np.min(delays_sec)  # 选最早路径

            toa_sec[aid] = toa

        # 计算 TDOA（相对于 00）
        ref_toa = toa_sec["00"]
        for aid in self.antenna_order:
            tdoa_ns[aid] = (toa_sec[aid] - ref_toa) * 1e9  # 纳秒

        full_data['toa_sec'] = toa_sec
        full_data['tdoa_ns'] = tdoa_ns
        full_data['tdoa_computed'] = True

        self.tdoa_buffer = []  # 清空缓冲
        return full_data
    
    def _estimate_coherent_h(self, csi_matrix):
        """对单个子载波的 (T, M) 矩阵估计相位一致 h_hat (M,)"""
        T, M = csi_matrix.shape
        C_hat = np.zeros((M, M), dtype=np.complex64)

        # 计算样本协方差
        for i in range(M):
            for j in range(M):
                C_hat[i, j] = np.mean(csi_matrix[:, i] * np.conj(csi_matrix[:, j]))

        # 噪声估计（信号秩=1）
        C_sym = (C_hat + C_hat.conj().T) / 2
        eigvals = np.linalg.eigvalsh(C_sym)
        eigvals = np.sort(eigvals)[::-1]
        sigma2 = np.mean(eigvals[1:]) if M > 1 else 0.0

        C_signal = C_hat - sigma2 * np.eye(M)
        C_herm = (C_signal + C_signal.conj().T) / 2

        # 特征分解
        eigvals, eigvecs = np.linalg.eigh(C_herm)
        idx_max = np.argmax(eigvals)
        lambda_max = eigvals[idx_max]
        u_max = eigvecs[:, idx_max]

        scale = np.sqrt(max(lambda_max - sigma2, 0.0))
        return scale * u_max

    def _interp_iterative(self, csi_matrix, iterations=10):
            """迭代插值CSI矩阵 (已替换为加速版)"""
            # 直接调用外部定义的加速函数，输入纯 Numpy 数组
            csi_matrix = csi_matrix.astype(np.complex128)
            return _core_iterative_filter(csi_matrix, iterations)

    def _apply_rssi_weighting(self, csi_frame: np.ndarray, rssi_vals: np.ndarray):
        """
        对单帧 CSI 应用 RSSI 加权。
        
        :param csi_frame: shape (M, K) - M 天线, K 子载波
        :param rssi_vals: shape (M,) - 每根天线的 RSSI (dBm)
        :return: 加权后的 csi_frame, shape (M, K)
        """
        # 转换 RSSI (dBm) 到线性幅度尺度: 10^(RSSI/20)
        weights = 10 ** (rssi_vals / 20.0)  # (M,)
        # 广播到子载波维度
        csi_weighted = csi_frame * weights[:, np.newaxis]  # (M, K)
        return csi_weighted

    def _shift_to_firstpeak_sync_single(self, csi_frame: np.ndarray):
        """ 对 *单帧* CSI 数据进行全局 CIR 峰值同步（所有天线使用相同移位）"""

        M, K = csi_frame.shape
        # 1. 生成时延候选和移位向量
        shifts = np.linspace(-self.CIR_MAX_DELAY_TAPS, 0, self.CIR_SEARCH_RES)  # (D,)
        subcarrier_range = np.arange(-K // 2, K // 2) + 1  # (K,)
        # 移位向量: 对每个时延 d，生成频域相位旋转因子
        shift_vectors = np.exp(1.0j * np.outer(shifts, 2 * np.pi * subcarrier_range / K))  # (D, K)

        # 2. 计算每个时延 d 下的 *总接收功率*
        # 将 csi_frame 应用每个移位向量，然后求总能量
        # shifted = csi_frame[None, :, :] * shift_vectors[:, None, :]  # (D, M, K)
        # 用 einsum 更高效：
        shifted = np.einsum("mk,dk->dmk", csi_frame, shift_vectors)  # (D, M, K)
        total_power = np.sum(np.abs(shifted) ** 2)  # 标量？不！要按 d 求和
        # 正确：对每个 d，求所有 m,k 的能量
        total_power = np.sum(np.abs(shifted) ** 2, axis=(1, 2))  # (D,)

        # 3. 找到第一个超过阈值的峰值
        max_power = np.max(total_power)
        threshold = self.CIR_PEAK_THRESHOLD * max_power
        # 从左到右找第一个超过阈值的
        valid = total_power > threshold
        if np.any(valid):
            best_delay_idx = np.argmax(valid)  # 第一个 True 的索引
        else:
            best_delay_idx = 0  # 无显著峰值，不移位

        # 4. 应用最佳移位（广播到所有天线）
        best_shift = shift_vectors[best_delay_idx]  # (K,)
        csi_sync = csi_frame * best_shift  # (M, K) * (K,) → (M, K)

        return csi_sync

    def _calculate_calibrated_data(self, full_data):
        """计算校准后的CSI数据"""
        # 延迟
        delays = [full_data[aid]['timestamp'] - self.h_ref[aid]['timestamp'] for aid in full_data]
        mean_delay = np.mean(delays)
        # 子载波频率偏移
        freq_offsets = (self.lltf_frequencys - self.lltf_center_freq) if self.wifi_protocol == 'LLTF' else (self.ht40_frequencys - self.ht40_center_freq)
        # 对每个天线处理
        for i, aid in enumerate(full_data):
            relative_delay = delays[i] - mean_delay
            sto_factor = np.exp(-1.0j * 2 * np.pi * relative_delay * freq_offsets)
            pcb_factor = np.exp(-1.0j * np.angle(self.h_ref[aid]['csi_complex']))
            # full_data[aid]['csi_complex'] = full_data[aid]['csi_complex'] * sto_factor * pcb_factor
            full_data[aid]['csi_complex'] = full_data[aid]['csi_complex'] * pcb_factor 
            full_data[aid]['csi_phase'] = np.angle(full_data[aid]['csi_complex'])
        return full_data

    def set_calibration_reference(self, h_ref_origin):
        """设置校准参考数据"""
        self.h_ref_origin = h_ref_origin
        self.h_ref = {}  # 校准后的数据也保存为字典
        antenna_order = ['00', '01', '02', '03', '10', '11', '12', '13']
        
        if self.wifi_protocol == 'LLTF':
            pcb_offsets = self.prop_calib_each_board_lltf  # 形状: (8, 子载波数)
        elif self.wifi_protocol == 'HT40':
            pcb_offsets = self.prop_calib_each_board_ht40  # 形状: (8, 子载波数)

        for antenna_id, ref_data in h_ref_origin.items():
            row_idx = antenna_order.index(antenna_id)
            csi_data = ref_data.get('csi_complex')
            # PCB补偿
            pcb_offset_for_antenna = pcb_offsets[row_idx, :]
            # csi_ref = csi_data * np.conj(pcb_offset_for_antenna)
            csi_ref = csi_data
            self.h_ref[antenna_id] = {
                'csi_complex': csi_ref,  # 更新为校准后的CSI
                'timestamp': ref_data.get('timestamp'),  # 保留时间戳
            }

    def clear_calibration_reference(self):
        """清除校准参考数据"""
        self.prop_calib_each_board_lltf = None
        self.prop_calib_each_board_ht40 = None
        self.h_ref = None
        self.h_ref_origin = None 
        self.lltf_frequencys = None
        self.ht40_frequencys = None
        self.ht40_center_freq = None
        self.lltf_center_freq = None

    def process(self, raw_data):
        antenna_index = raw_data['antenna_index']
        antenna_id = self.antenna_mapping.get(antenna_index, f"未知{antenna_index}")
        # 计算粗略时间戳
        coarse_timestamp = (raw_data['timestamp'] + self.timestamp_scale/2) // self.timestamp_scale
        # print(coarse_timestamp)
        # 如果是新时间戳（新帧），清空旧缓存
        if self.last_coarse_timestamp is not None and coarse_timestamp != self.last_coarse_timestamp:
            self.antenna_data.clear()
        self.last_coarse_timestamp = coarse_timestamp

        # 转换 CSI 为复数
        csi_int8 = np.array(raw_data['csi_data'], dtype=np.int8)
        csi_complex = csi_int8[0::2] + 1j * csi_int8[1::2]  
        csi_complex = csi_complex.astype(np.complex64)      

        # 重新排列子载波
        if self.wifi_protocol == 'HT40':# and raw_data['cwb'] == 1 and raw_data['sig_mode'] == 1:
            part1 = csi_complex[64:128] # 原 64–127
            part2 = csi_complex[128:192] # 原 127-191
            csi_complex = np.concatenate([part2, part1])
        else:
            part1 = csi_complex[0:32]     # 原 0–31
            part2 = csi_complex[32:64]    # 原 32–63
            csi_complex = np.concatenate([part2, part1])

        # 构建处理后的数据项（不再包含 csi_index）
        processed = {
            'antenna_id': antenna_id,
            'timestamp': raw_data['timestamp'],
            'coarse_timestamp': coarse_timestamp,
            'mac': raw_data.get('mac', 'N/A'),
            'dmac': raw_data.get('dmac', 'N/A'),
            'rssi': raw_data['rssi'],
            'noise_floor': raw_data['noise_floor'],
            'rate': raw_data['rate'],
            'sgi': raw_data['sgi'],
            'sig_mode': raw_data['sig_mode'],
            'mcs': raw_data['mcs'],
            'cwb': raw_data['cwb'],
            'channel': raw_data['channel'],
            'secondary_channel': raw_data['secondary_channel'],
            'rx_state': raw_data['rx_state'],
            'fft_gain': raw_data['fft_gain'],
            'agc_gain': raw_data['agc_gain'],
            'gain': raw_data['gain'],
            'csi_complex': csi_complex,
            'csi_magnitude': np.abs(csi_complex),
            'csi_phase': np.angle(csi_complex),
            'subcarriers': np.arange(len(csi_complex)),
            'subcarriers_nums': len(csi_complex)
        }
        self.antenna_data[antenna_id] = processed
        # 检查是否8个天线收齐
        if set(self.antenna_data.keys()) != self.expected_antennas:
            return None
        # 检查：所有天线 coarse_timestamp 必须完全一致
        coarse_timestamps = {data['coarse_timestamp'] for data in self.antenna_data.values()}
        if len(coarse_timestamps) != 1:
            self.antenna_data.clear()
            self.last_coarse_timestamp = None  # 同步清空
            return None

        full_data = dict(self.antenna_data)
        self.antenna_data.clear()
        self.last_coarse_timestamp = None  # 本组已处理，重置状态

        # ========== 1. 构造单帧 CSI 和 RSSI ==========
        csi_frame = np.array([
            full_data[aid]['csi_complex'] for aid in self.antenna_order
        ])  # (M, K)
        rssi_vals = np.array([
            full_data[aid]['rssi'] for aid in self.antenna_order
        ], dtype=np.float32)  # (M,)

        if self.ENABLE_RSSI_WEIGHTING:
            csi_frame = self._apply_rssi_weighting(csi_frame, rssi_vals)

        if self.ENABLE_CIR_SYNC:
            csi_frame = self._shift_to_firstpeak_sync_single(csi_frame)

        for idx, aid in enumerate(self.antenna_order):
            cvec = csi_frame[idx]
            full_data[aid]['csi_complex'] = cvec
            full_data[aid]['csi_magnitude'] = np.abs(cvec)
            full_data[aid]['csi_phase'] = np.angle(cvec)

        try:
            sample_aid = self.antenna_order[0]
            N_SC = full_data[sample_aid]['subcarriers_nums']

            # ========== 协方差插值 ==========
            if self.ENABLE_EIGENVEC_FILTERING:
                if self.eigenvec_windows is None or self.current_num_subcarriers != N_SC:
                    self.eigenvec_windows = [
                        deque(maxlen=self.SLIDING_WINDOW_SIZE) for _ in range(N_SC)
                    ]
                    self.current_num_subcarriers = N_SC
                # 填充窗口
                for n in range(N_SC):
                    csi_vector_n = np.array([
                        full_data[aid]['csi_complex'][n] for aid in self.antenna_order
                    ], dtype=np.complex64)
                    self.eigenvec_windows[n].append(csi_vector_n)
                # 执行插值
                if all(len(w) == self.SLIDING_WINDOW_SIZE for w in self.eigenvec_windows):
                    h_hat_all = np.zeros((self.M, N_SC), dtype=np.complex64)
                    for n in range(N_SC):
                        csi_matrix_n = np.array(self.eigenvec_windows[n])
                        h_hat_all[:, n] = self._estimate_coherent_h(csi_matrix_n)
                    for idx, aid in enumerate(self.antenna_order):
                        data = full_data[aid]
                        h_vec = h_hat_all[idx, :]
                        data['csi_complex'] = h_vec
                        data['csi_magnitude'] = np.abs(h_vec)
                        data['csi_phase'] = np.angle(h_vec)
                        data['subcarriers'] = np.arange(N_SC)
            # ========== 迭代插值 ==========
            if self.ENABLE_ITERATIVE_FILTERING: 
                # 初始化：首次/子载波数变化时，创建空列表（无maxlen）
                if self.iterative_windows is None or self.current_num_subcarriers != N_SC:
                    self.iterative_windows = [deque(maxlen=self.SLIDING_WINDOW_SIZE) for _ in range(N_SC)]
                    self.current_num_subcarriers = N_SC
                
                # 1. 填充窗口：每帧追加数据
                for n in range(N_SC):
                    csi_vector_n = np.array([
                        full_data[aid]['csi_complex'][n] for aid in self.antenna_order
                    ], dtype=np.complex64)
                    self.iterative_windows[n].append(csi_vector_n)
                
                # 2. 窗口填满后：计算插值
                if all(len(w) == self.SLIDING_WINDOW_SIZE for w in self.iterative_windows):
                    h_hat_all = np.zeros((self.M, N_SC), dtype=np.complex64)
                    for n in range(N_SC):
                        # 用当前满窗口的数据计算
                        csi_matrix_n = np.array(self.iterative_windows[n])
                        h_hat_all[:, n] = self._interp_iterative(
                            csi_matrix_n, iterations=self.ITERATIONS
                        )
                    
                    # 更新CSI数据（图像会基于此更新）
                    for idx, aid in enumerate(self.antenna_order):
                        data = full_data[aid]
                        h_vec = h_hat_all[idx, :]
                        data['csi_complex'] = h_vec
                        data['csi_magnitude'] = np.abs(h_vec)
                        data['csi_phase'] = np.angle(h_vec)
                        data['subcarriers'] = np.arange(N_SC)

        except Exception as e:
            print(f"[Filter] Error: {e}")
            return full_data

        # 计算校准后数据
        if self.h_ref is not None:
            full_data = self._calculate_calibrated_data(full_data)

        if tab_enabled["aoa_1d"] == True:
            full_data = self._calculate_aoa_1d(full_data)

        if tab_enabled["aoa_2d"] == True:
            full_data = self._calculate_aoa_2d(full_data)

        if tab_enabled["tdoa"] == True:
            full_data = self._estimate_tdoa_rootmusic(full_data)
        return full_data

    def clear_antenna_data(self):
        self.antenna_data.clear()
        self.last_coarse_timestamp = None  # <<< 关键：清空时间戳状态
        if self.ENABLE_EIGENVEC_FILTERING:
            for w in self.sliding_windows:
                w.clear()

class DisplayTab(ABC):
    """显示标签页的基类"""
    
    def __init__(self, notebook, title):
        self.notebook = notebook
        self.title = title
        self.frame = ttk.Frame(notebook)
        self.setup_ui()
        notebook.add(self.frame, text=title)
    
    @abstractmethod
    def setup_ui(self):
        """设置UI界面"""
        pass
    
    @abstractmethod
    def update_data(self, full_data):
        """更新数据显示，full_ dict {antenna_id: data}"""
        pass
    
    def clear_data(self):
        pass

    def refresh_if_needed(self):
        pass


class MultiAntennaDisplayTab(DisplayTab):
    """多天线同时显示标签页"""
    
    def __init__(self, notebook, title):
        self.antenna_labels = {}
        super().__init__(notebook, title)
    
    def setup_ui(self):
        main_frame = ttk.Frame(self.frame)
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)
        
        for i in range(2):
            main_frame.rowconfigure(i, weight=1)
        for j in range(4):
            main_frame.columnconfigure(j, weight=1)
        
        antenna_ids = ["00", "01", "02", "03", "10", "11", "12", "13"]
        for i, antenna_id in enumerate(antenna_ids):
            row = i // 4
            col = i % 4
            antenna_frame = ttk.LabelFrame(main_frame, text=f"天线 {antenna_id}", padding="8")
            antenna_frame.grid(row=row, column=col, padx=5, pady=5, sticky="nsew")
            
            text_widget = tk.Text(antenna_frame, height=12, width=25, font=("Consolas", 10),
                                bg="#f8f9fa", relief="flat", bd=0, wrap=tk.WORD)
            text_widget.pack(fill=tk.BOTH, expand=True, padx=2, pady=2)
            text_widget.insert(tk.END, f"等待天线 {antenna_id} 数据...\n")
            text_widget.config(state=tk.DISABLED)
            self.antenna_labels[antenna_id] = text_widget
    
    def update_data(self, full_data):
        """更新所有8个天线的数据显示"""
        for antenna_id, data in full_data.items():
            if antenna_id in self.antenna_labels:
                text_widget = self.antenna_labels[antenna_id]
                text_widget.config(state=tk.NORMAL)
                text_widget.delete(1.0, tk.END)
                
                info = (
                    f"时间戳: {data['timestamp']}\n"
                    f"RSSI: {data['rssi']} dBm\n"
                    f"噪声基底: {data['noise_floor']} dBm\n"
                    f"MAC: {data['mac']}\n"
                    f"DMAC: {data['dmac']}\n"
                    f"速率: {data['rate']}\n"
                    f"sgi: {data['sgi']}\n"
                    f"信号模式: {data['sig_mode']}\n"
                    f"MCS: {data['mcs']}\n"
                    f"CWB: {data['cwb']}\n"
                    f"信道: {data['channel']}\n"
                    f"副信道: {data['secondary_channel']}\n"
                    f"RX状态: {data['rx_state']}\n"
                    f"FFT增益: {data['fft_gain']}\n"
                    f"AGC增益: {data['agc_gain']}\n"
                    f"补偿增益: {data['gain']}\n"
                    f"子载波数: {data['subcarriers_nums']}\n"
                    
                )
                text_widget.insert(tk.END, info)
                text_widget.config(state=tk.DISABLED)
                text_widget.see(tk.END)
    
    def clear_data(self):
        for antenna_id, text_widget in self.antenna_labels.items():
            text_widget.config(state=tk.NORMAL)
            text_widget.delete(1.0, tk.END)
            text_widget.insert(tk.END, f"等待天线 {antenna_id} 数据...\n")
            text_widget.config(state=tk.DISABLED)


class PlotDisplayTab(DisplayTab):
    """仅显示 CSI 幅度和相位的图形标签页，支持选择天线"""

    def __init__(self, notebook, title):
        self.selected_antenna = tk.StringVar(value="00")  # 默认选"00"
        self.antenna_list = ["00", "01", "02", "03", "10", "11", "12", "13"]
        super().__init__(notebook, title)

    def setup_ui(self):
        # 上方控制区：天线选择
        control_frame = ttk.Frame(self.frame)
        control_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        ttk.Label(control_frame, text="选择天线:").pack(side=tk.LEFT, padx=(0, 5))
        antenna_combo = ttk.Combobox(
            control_frame,
            textvariable=self.selected_antenna,
            values=self.antenna_list,
            state="readonly",
            width=5
        )
        antenna_combo.pack(side=tk.LEFT, padx=(0, 10))
        antenna_combo.bind("<<ComboboxSelected>>", self.on_antenna_selected)

        # 图形区
        self.fig, (self.ax1, self.ax2) = plt.subplots(2, 1, figsize=(10, 8))
        self.fig.tight_layout(pad=3.0)

        for ax in [self.ax1, self.ax2]:
            ax.grid(True, alpha=0.3)

        self.ax1.set_title('CSI Magnitude Spectrum')
        self.ax1.set_xlabel('Subcarrier Index')
        self.ax1.set_ylabel('Magnitude')
        self.line1, = self.ax1.plot([], [], 'b-', linewidth=1, marker='o', markersize=2)

        self.ax2.set_title('CSI Phase Spectrum')
        self.ax2.set_xlabel('Subcarrier Index')
        self.ax2.set_ylabel('Phase (rad)')
        self.line2, = self.ax2.plot([], [], 'r-', linewidth=1, marker='s', markersize=2)

        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def on_antenna_selected(self, event=None):
        """当天线选择改变时，若已有数据，则立即刷新"""
        if hasattr(self, '_last_full_data'):
            self.update_data(self._last_full_data)

    def update_data(self, full_data):
        """根据当前选中的天线更新图形"""
        self._last_full_data = full_data  # 缓存最新数据，用于切换天线时刷新

        antenna_id = self.selected_antenna.get()
        if antenna_id not in full_data:
            return

        data = full_data[antenna_id]
        self.line1.set_data(data['subcarriers'], data['csi_magnitude'][:data['subcarriers_nums']])
        self.ax1.relim()
        self.ax1.autoscale_view()

        phase_unwrapped = np.unwrap(data['csi_phase'], discont=np.pi)
        self.line2.set_data(data['subcarriers'], phase_unwrapped[:data['subcarriers_nums']])
        self.ax2.relim()
        self.ax2.autoscale_view()

        self.canvas.draw()

    def refresh_if_needed(self):
        if hasattr(self, '_last_full_data'):
            self.update_data(self._last_full_data)


class PhaseTimeDisplayTab(DisplayTab):
    """显示指定子载波上 8 根天线的相位随时间变化（滚动窗口）"""

    def __init__(self, notebook, title, max_history=2000):
        self.max_history = max_history  # 滚动窗口大小（包数）
        self.selected_subcarrier = tk.StringVar(value="15")  # 默认选15
        self.subcarrier_list = [str(i) for i in range(64)]  # 0~63

        # 每根天线一个 deque，存最近 max_history 个 (timestamp, phase) 元组
        self.antennas = ["00", "01", "02", "03", "10", "11", "12", "13"]
        self.phase_history = {aid: deque(maxlen=self.max_history) for aid in self.antennas}
        self.time_history = {aid: deque(maxlen=self.max_history) for aid in self.antennas}

        # 存最新 full_data（用于切换子载波时重绘）
        self.last_full_data = None

        super().__init__(notebook, title)

    def setup_ui(self):
        # 控制区
        control_frame = ttk.Frame(self.frame)
        control_frame.pack(fill=tk.X, padx=10, pady=(10, 5))

        ttk.Label(control_frame, text="子载波:").pack(side=tk.LEFT, padx=(0, 5))
        subcarrier_combo = ttk.Combobox(
            control_frame,
            textvariable=self.selected_subcarrier,
            values=self.subcarrier_list,
            state="readonly",
            width=5
        )
        subcarrier_combo.pack(side=tk.LEFT, padx=(0, 10))
        subcarrier_combo.bind("<<ComboboxSelected>>", self.on_subcarrier_selected)

        # 绘图区
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.tight_layout(pad=3.0)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Phase vs. Time (per antenna, selected subcarrier)')
        self.ax.set_xlabel('Packet Index (relative)')
        self.ax.set_ylabel('Phase (rad)')

        # 为 8 根天线创建线条（用不同颜色）
        self.lines = {}
        colors = plt.cm.tab10(np.linspace(0, 1, 8))
        for i, aid in enumerate(self.antennas):
            line, = self.ax.plot([], [], label=f'Antenna {aid}', color=colors[i], linewidth=1.2)
            self.lines[aid] = line

        self.ax.legend(loc='upper right', fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def on_subcarrier_selected(self, event=None):
        """子载波切换时，用缓存的 last_full_data 重绘（如果存在）"""
        if self.last_full_data is not None:
            self._update_plot(self.last_full_data)

    def update_data(self, full_data):
        """接收 full_data，更新所有天线在选定子载波的相位历史"""
        self.last_full_data = full_data

        try:
            subcarrier_idx = int(self.selected_subcarrier.get())
        except:
            return

        # 更新每根天线的历史
        packet_index = len(next(iter(self.time_history.values())))  # 当前包序号（相对）
        for aid in self.antennas:
            if aid in full_data:
                csi_complex = full_data[aid]['csi_complex']
                if subcarrier_idx < len(csi_complex):
                    phase = np.angle(csi_complex[subcarrier_idx])
                    self.phase_history[aid].append(phase)
                    self.time_history[aid].append(packet_index)

        self._update_plot(full_data)

    def _update_plot(self, full_data):
        """实际绘图函数"""
        try:
            subcarrier_idx = int(self.selected_subcarrier.get())
        except:
            return

        # 获取当前最长历史长度（用于 x 轴）
        max_len = max(len(hist) for hist in self.time_history.values()) if any(self.time_history.values()) else 1
        x_base = np.arange(max_len)

        for aid in self.antennas:
            phases = list(self.phase_history[aid])
            if phases:
                x = x_base[-len(phases):]
                self.lines[aid].set_data(x, phases)
            else:
                self.lines[aid].set_data([], [])

        self.ax.relim()
        self.ax.autoscale_view()

        # 直接修改原始代码，添加微小偏移
        if len(x_base) > 0:
            self.ax.set_xlim(
                x_base[-self.max_history] if len(x_base) >= self.max_history else 0,
                x_base[-1] + 1e-10  # 添加微小偏移
            )
        else:
            self.ax.set_xlim(0, self.max_history)

        self.ax.set_title(f'Phase vs. Time (Subcarrier {subcarrier_idx})')
        self.canvas.draw()

    def clear_data(self):
        for aid in self.antennas:
            self.phase_history[aid].clear()
            self.time_history[aid].clear()
        self._update_plot({})

    def refresh_if_needed(self):
        if hasattr(self, 'last_full_data'):
            self._update_plot(self.last_full_data)


class PhaseStabilityTab(DisplayTab):
    """复现论文 IV-A: Phase Stability 实验 —— 宽带相位 vs 时间（以 00 为参考）"""

    def __init__(self, notebook, title, max_history=2000):
        self.max_history = max_history
        self.antennas = ["00", "01", "02", "03", "10", "11", "12", "13"]
        # 存储相对于 00 的相位差历史
        self.phase_diff_history = {aid: deque(maxlen=self.max_history) for aid in self.antennas}
        self.time_index = 0  # 相对包序号
        super().__init__(notebook, title)

    def setup_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 6))
        self.fig.tight_layout(pad=3.0)
        self.ax.grid(True, alpha=0.3)
        self.ax.set_title('Phase Stability (Broadband Phase Difference vs Time)')
        self.ax.set_xlabel('Packet Index (relative)')
        self.ax.set_ylabel('Phase Difference (rad)')

        # 创建 8 条曲线（00 本身应恒为 0，可不画或画为黑线）
        self.lines = {}
        colors = plt.cm.tab10(np.linspace(0, 1, 8))
        for i, aid in enumerate(self.antennas):
            if aid == "00":
                # 参考天线，理论上应为 0
                line, = self.ax.plot([], [], 'k--', linewidth=1, label=f'{aid} (ref)')
            else:
                line, = self.ax.plot([], [], color=colors[i], linewidth=1.2, label=f'{aid} - 00')
            self.lines[aid] = line

        self.ax.legend(loc='upper right', fontsize=8)
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def update_data(self, full_data):
        """计算宽带相位并更新历史"""
        # 1. 计算每根天线的宽带相位 φ_m = angle(sum(csi_complex))
        broadband_phases = {}
        for aid in self.antennas:
            if aid in full_data:
                csi = full_data[aid]['csi_complex'][1:24]
                # csi_sum = np.sum(csi[0:full_data[aid]['subcarriers_nums']])
                csi_sum = np.sum(csi)
                broadband_phases[aid] = np.angle(csi_sum)
            else:
                return  # 任意天线缺失则跳过

        # 2. 以 "00" 为参考，计算相位差
        ref_phase = broadband_phases["00"]
        for aid in self.antennas:
            diff = broadband_phases[aid] - ref_phase
            # 归一化到 [-π, π]
            while diff > np.pi:
                diff -= 2 * np.pi
            while diff < -np.pi:
                diff += 2 * np.pi
            self.phase_diff_history[aid].append(diff)

        self.time_index += 1
        self._update_plot()

    def _update_plot(self):
        max_len = max(len(hist) for hist in self.phase_diff_history.values()) if any(self.phase_diff_history.values()) else 1
        x_base = np.arange(max_len)

        for aid in self.antennas:
            diffs = list(self.phase_diff_history[aid])
            if diffs:
                x = x_base[-len(diffs):]
                self.lines[aid].set_data(x, diffs)
            else:
                self.lines[aid].set_data([], [])

        self.ax.relim()
        self.ax.autoscale_view()
        # 固定 Y 轴范围为 [-3π, 3π]
        self.ax.set_ylim(-3 * np.pi, 3 * np.pi)
        y_ticks = np.arange(-3, 4) * np.pi
        y_labels = [f"{i}π" if i != 0 else "0" for i in range(-3, 4)]
        self.ax.set_yticks(y_ticks)
        self.ax.set_yticklabels(y_labels)

        # X 轴滚动 - 修改这里，避免相同的最小值和最大值
        if len(x_base) > 0:
            x_min = x_base[-self.max_history] if len(x_base) >= self.max_history else 0
            x_max = x_base[-1]
            
            # 如果x_min和x_max相同，给x_max增加一个微小偏移
            if x_min == x_max:
                x_max += 1e-10  # 或使用 x_max += 0.1，取决于您的数据范围
            
            self.ax.set_xlim(x_min, x_max)
        else:
            # 如果没有数据，设置一个默认范围
            self.ax.set_xlim(0, self.max_history)

        self.canvas.draw()

    def clear_data(self):
        for hist in self.phase_diff_history.values():
            hist.clear()
        self.time_index = 0
        self._update_plot()

    def refresh_if_needed(self):
        if any(self.phase_diff_history.values()):
            self._update_plot()

class CalibratedPhaseTab(DisplayTab):
    """显示校准后的相位热力图"""
    def __init__(self, notebook, title, gui_ref):
        self.gui_ref = gui_ref  # 指向 CSIDataGUI 实例
        self.antennas = ["00", "01", "02", "03", "10", "11", "12", "13"]
        # 定义天线在 2x4 网格中的位置 (row, col)
        self.antenna_positions = {
            "00": (0, 0), "01": (0, 1), "02": (0, 2), "03": (0, 3),
            "10": (1, 0), "11": (1, 1), "12": (1, 2), "13": (1, 3)
        }
        super().__init__(notebook, title)

    def setup_ui(self):
        # 创建图形区域
        self.fig, self.ax = plt.subplots(figsize=(8, 6))
        self.fig.tight_layout(pad=3.0)
        self.ax.set_title('Calibrated Phase Heatmap (Relative to Antenna 00)')
        self.ax.set_xlabel('Horizontal Antenna Position')
        self.ax.set_ylabel('Vertical Antenna Position')
        # 设置坐标轴刻度
        self.ax.set_xticks(np.arange(4))
        self.ax.set_yticks(np.arange(2))
        self.ax.set_xticklabels(['0', '1', '2', '3'])
        self.ax.set_yticklabels(['0', '1'])
        self.ax.grid(True, color='white', linewidth=2)  # 白色网格，增强可读性
        # 创建一个空的 heatmap
        self.im = self.ax.imshow(
            np.zeros((2, 4)), 
            cmap='hsv',  # 改为hsv颜色映射，它本身就是循环的
            vmin=-np.pi, 
            vmax=np.pi,
            interpolation='nearest', 
            aspect='equal'
        )
        # 添加颜色条
        cbar = self.fig.colorbar(self.im, ax=self.ax, shrink=0.8, label='Phase [rad]')
        cbar.set_ticks([-np.pi, -np.pi/2, 0, np.pi/2, np.pi])
        cbar.set_ticklabels([r'$-\pi$', r'$-\pi/2$', '0', r'$\pi/2$', r'$\pi$'])

        # 绘制天线标签
        for aid, (row, col) in self.antenna_positions.items():
            self.ax.text(col, row, aid, ha='center', va='center', fontsize=10, color='black', weight='bold')

        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=(0, 10))

    def update_data(self, full_data):
        # 计算相对于 00 的相位差
        if "00" not in full_data:
            return

        # 使用校准后的相位数据
        ref_phase = full_data["00"]['csi_phase'][15]
        # ref_phase = full_data["00"]['csi_phase']
        phase_diffs = {}
        for aid in self.antennas:
            if aid in full_data:
                # 使用宽带相位（所有子载波相位的平均值）
                # broadband_phase = np.mean(full_data[aid]['csi_phase'])
                broadband_phase = full_data[aid]['csi_phase'][15]
                diff = broadband_phase - np.mean(ref_phase)
                # 归一化到 [-π, π]
                while diff > np.pi:
                    diff -= 2 * np.pi
                while diff < -np.pi:
                    diff += 2 * np.pi
                phase_diffs[aid] = diff

        # 构建 2x4 的相位矩阵
        phase_matrix = np.zeros((2, 4))
        for aid, (row, col) in self.antenna_positions.items():
            if aid in phase_diffs:
                phase_matrix[row, col] = phase_diffs[aid]

        # 更新热力图
        self.im.set_array(phase_matrix)
        
        # 根据是否有校准参考数据来设置标题
        processor = self.gui_ref.receiver.processor
        if processor.h_ref is not None:
            self.ax.set_title('Calibrated Phase Heatmap (Calibrated)')
        else:
            self.ax.set_title('Calibrated Phase Heatmap (Uncalibrated)')
            
        self.canvas.draw()

    def clear_data(self):
        # 清除时显示全零（灰色）
        self.im.set_array(np.zeros((2, 4)))
        self.ax.set_title('Calibrated Phase Heatmap (No Data)')
        self.canvas.draw()

    def refresh_if_needed(self):
        # 如果有缓存数据，则刷新
        if hasattr(self, '_last_full_data'):
            self.update_data(self._last_full_data)

class AOA1DTab(DisplayTab):
    """显示 1D AoA 估计结果（MUSIC 谱）"""
    def __init__(self, notebook, title):
        super().__init__(notebook, title)

    def setup_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.fig.tight_layout(pad=3.0)
        self.ax.set_title('1D AoA Estimation - MUSIC Algorithm')
        self.ax.set_xlabel('Angle (°)')
        self.ax.set_ylabel('MUSIC Spectrum (dB)')
        self.ax.grid(True, alpha=0.3)
        
        # 绘制MUSIC谱线和峰值标记
        self.line, = self.ax.plot([], [], 'b-', linewidth=1.5, label='MUSIC Spectrum')
        self.peak_line = self.ax.axvline(0, color='red', linestyle='--', alpha=0.8, label='Peak Angle')
        self.peak_text = self.ax.text(0.02, 0.98, '', transform=self.ax.transAxes, 
                                     verticalalignment='top', fontsize=12,
                                     bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8))
        self.ax.legend()
        
        # 设置角度范围（-90到90度）
        self.ax.set_xlim(-90, 90)
        self.ax.set_ylim(-40, 0)  # 调整Y轴范围，因为你的谱是20*log10的dB值
        
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def update_data(self, full_data):
        if not full_data.get('music_computed', False):
            return
        # 获取数据（直接从full_data中获取）
        angles_deg = full_data.get('aoa_angles', [])
        spectrum_db = full_data.get('aoa_spectrum', [])  # 你的数据处理输出已经是dB值
        peak_angle = full_data.get('aoa', 0)
        
        if len(angles_deg) == 0 or len(spectrum_db) == 0:
            return
        
        # 更新图形
        self.line.set_data(angles_deg, spectrum_db)
        
        # 设置峰值线
        if peak_angle is not None:
            self.peak_line.set_xdata([peak_angle])
            self.peak_text.set_text(f'Peak: {peak_angle:.1f}°')
        else:
            # 如果没有峰值角度，找到最大值
            peak_idx = np.argmax(spectrum_db)
            peak_angle = angles_deg[peak_idx]
            self.peak_line.set_xdata([peak_angle])
            self.peak_text.set_text(f'Peak: {peak_angle:.1f}°')
        
        # 更新标题
        self.ax.set_title(f'1D AoA MUSIC Estimation: Peak at {peak_angle:.1f}°')
        
        # 自动调整Y轴范围
        if len(spectrum_db) > 0:
            y_min = np.min(spectrum_db)
            y_max = np.max(spectrum_db)
            margin = (y_max - y_min) * 0.1
            self.ax.set_ylim(y_min - margin, y_max + margin)
        
        self.canvas.draw()

    def clear_data(self):
        self.line.set_data([], [])
        self.peak_line.set_xdata([0])
        self.peak_text.set_text('')
        self.ax.set_title('1D AoA Estimation - MUSIC Algorithm (No Data)')
        self.ax.set_ylim(-40, 0)
        self.canvas.draw()

class AOA2DTab(DisplayTab):
    """显示 2D AoA 估计结果（方位角 azimuth + 仰角 elevation）"""
    def __init__(self, notebook, title):
        super().__init__(notebook, title)

    def setup_ui(self):
        # 创建两个子图：方位角谱和仰角谱
        self.fig, (self.ax_azimuth, self.ax_elevation) = plt.subplots(
            1, 2, figsize=(12, 5)
        )
        self.fig.tight_layout(pad=3.0)
        
        # 方位角谱图
        self.ax_azimuth.set_title('Azimuth Spectrum (1D Projection)')
        self.ax_azimuth.set_xlabel('Azimuth (°)')
        self.ax_azimuth.set_ylabel('MUSIC Spectrum (dB)')
        self.ax_azimuth.grid(True, alpha=0.3)
        self.ax_azimuth.set_xlim(-90, 90)
        self.ax_azimuth.set_ylim(-40, 0)
        
        # 仰角谱图
        self.ax_elevation.set_title('Elevation Spectrum (1D Projection)')
        self.ax_elevation.set_xlabel('Elevation (°)')
        self.ax_elevation.set_ylabel('MUSIC Spectrum (dB)')
        self.ax_elevation.grid(True, alpha=0.3)
        self.ax_elevation.set_xlim(-90, 90)
        self.ax_elevation.set_ylim(-40, 0)
        
        # 初始化线条
        self.line_azimuth, = self.ax_azimuth.plot([], [], 'b-', linewidth=2, label='Azimuth Spectrum')
        self.line_elevation, = self.ax_elevation.plot([], [], 'g-', linewidth=2, label='Elevation Spectrum')
        
        # 峰值标记
        self.peak_az_marker = self.ax_azimuth.axvline(0, color='r', linestyle='--', linewidth=2, label='Peak')
        self.peak_el_marker = self.ax_elevation.axvline(0, color='r', linestyle='--', linewidth=2, label='Peak')
        
        # 峰值文本
        self.text_az = self.ax_azimuth.text(
            0.02, 0.98, '', transform=self.ax_azimuth.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        self.text_el = self.ax_elevation.text(
            0.02, 0.98, '', transform=self.ax_elevation.transAxes,
            verticalalignment='top', fontsize=10,
            bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8)
        )
        
        # 标题中的角度信息
        self.title_text = self.fig.suptitle('2D AoA Estimation', fontsize=14)
        
        # 添加图例
        self.ax_azimuth.legend(loc='upper right')
        self.ax_elevation.legend(loc='upper right')
        
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def update_data(self, full_data):
        if not full_data.get('music2d_computed', False):
            return   
        spectrum_2d = np.array(full_data['aoa_spectrum_2d'])  # shape: (n_el, n_az)
        est_az = full_data['aoa_azimuth']
        est_el = full_data['aoa_elevation']
        # 获取角度轴
        az_axis = full_data.get('aoa_azimuths', np.linspace(-90, 90, spectrum_2d.shape[1]))
        el_axis = full_data.get('aoa_elevations', np.linspace(-90, 90, spectrum_2d.shape[0]))
        
        # 计算1D投影谱（沿另一维度取最大值）
        # 方位角谱：固定仰角维度，沿仰角轴取最大值
        azimuth_spectrum = np.max(spectrum_2d, axis=0)  # shape: (n_az,)
        
        # 仰角谱：固定方位角维度，沿方位角轴取最大值
        elevation_spectrum = np.max(spectrum_2d, axis=1)  # shape: (n_el,)
        
        # 更新方位角谱图
        self.line_azimuth.set_data(az_axis, azimuth_spectrum)
        
        # 更新仰角谱图
        self.line_elevation.set_data(el_axis, elevation_spectrum)
        
        # 更新峰值标记和文本
        if est_az is not None:
            self.peak_az_marker.set_xdata([est_az, est_az])
            self.text_az.set_text(f'Peak: {est_az:.1f}°')
        
        if est_el is not None:
            self.peak_el_marker.set_xdata([est_el, est_el])
            self.text_el.set_text(f'Peak: {est_el:.1f}°')
        
        # 更新标题
        self.title_text.set_text(f'2D AoA Estimation: Azimuth = {est_az:.1f}°, Elevation = {est_el:.1f}°')
        
        # 调整Y轴范围以适应数据
        if len(azimuth_spectrum) > 0:
            az_min, az_max = np.min(azimuth_spectrum), np.max(azimuth_spectrum)
            self.ax_azimuth.set_ylim(az_min - 5, az_max + 5)
        
        if len(elevation_spectrum) > 0:
            el_min, el_max = np.min(elevation_spectrum), np.max(elevation_spectrum)
            self.ax_elevation.set_ylim(el_min - 5, el_max + 5)
        
        self.canvas.draw()

    def clear_data(self):
        # 清空数据
        self.line_azimuth.set_data([], [])
        self.line_elevation.set_data([], [])
        self.peak_az_marker.set_xdata([0, 0])
        self.peak_el_marker.set_xdata([0, 0])
        self.text_az.set_text('')
        self.text_el.set_text('')
        self.title_text.set_text('2D AoA Estimation')
        
        # 重置Y轴范围
        self.ax_azimuth.set_ylim(-40, 0)
        self.ax_elevation.set_ylim(-40, 0)
        
        self.canvas.draw()

class TDOATab(DisplayTab):
    """显示每根天线的 TDOA（相对于 00）"""
    def __init__(self, notebook, title):
        super().__init__(notebook, title)

    def setup_ui(self):
        self.fig, self.ax = plt.subplots(figsize=(10, 5))
        self.fig.tight_layout(pad=3.0)
        self.ax.set_title('TDOA per Antenna (relative to 00)')
        self.ax.set_xlabel('Antenna ID')
        self.ax.set_ylabel('TDOA (ns)')
        self.ax.grid(True, alpha=0.3)
        self.bars = self.ax.bar([], [])
        self.ax.set_ylim(-100, 100)  # ±100 ns 足够（对应 ±30 米）
        self.canvas = FigureCanvasTkAgg(self.fig, self.frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

    def update_data(self, full_data):
        if not full_data.get('tdoa_computed', False):
            return
        antennas = ["00", "01", "02", "03", "10", "11", "12", "13"]
        tdoa_vals = [full_data['tdoa_ns'][aid] for aid in antennas]
        self.ax.clear()
        self.ax.bar(antennas, tdoa_vals, color='steelblue')
        self.ax.set_title('TDOA per Antenna (relative to 00)')
        self.ax.set_ylabel('TDOA (ns)')
        self.ax.grid(True, alpha=0.3)
        self.ax.set_ylim(-100, 100)
        self.canvas.draw()

    def clear_data(self):
        self.ax.clear()
        self.ax.set_title('TDOA per Antenna (No Data)')
        self.canvas.draw()


class TabManager:
    """标签页管理器（带显示开关）"""
    
    def __init__(self, notebook, processor):
        self.notebook = notebook
        self.tabs = {}
        self.processor = processor
        self.gui_ref = None  # 由 CSIDataGUI 设置
        
    def create_basic_tabs(self):
        multi_antenna_tab = MultiAntennaDisplayTab(self.notebook, "多天线监控")
        self.tabs["multi_antenna"] = multi_antenna_tab
        tab_enabled["multi_antenna"] = False

        plot_tab = PlotDisplayTab(self.notebook, "CSI可视化")
        self.tabs["csi_plot"] = plot_tab
        tab_enabled["csi_plot"] = False

        phase_time_tab = PhaseTimeDisplayTab(self.notebook, "相位时序")
        self.tabs["phase_time"] = phase_time_tab
        tab_enabled["phase_time"] = False

        phase_stab_tab = PhaseStabilityTab(self.notebook, "相位稳定性")
        self.tabs["phase_stability"] = phase_stab_tab
        tab_enabled["phase_stability"] = False

        cal_phase_tab = CalibratedPhaseTab(self.notebook, "校准后相位", self.gui_ref)
        self.tabs["calibrated_phase"] = cal_phase_tab
        tab_enabled["calibrated_phase"] = False

        aoa_tab = AOA1DTab(self.notebook, "AoA 1D")
        self.tabs["aoa_1d"] = aoa_tab
        tab_enabled["aoa_1d"] = False

        aoa2d_tab = AOA2DTab(self.notebook, "AoA 2D")
        self.tabs["aoa_2d"] = aoa2d_tab
        tab_enabled["aoa_2d"] = False

        tdoa_tab = TDOATab(self.notebook, "TDOA")
        self.tabs["tdoa"] = tdoa_tab
        tab_enabled["tdoa"] = False
    
    def update_all_tabs(self, full_data):
        """仅更新启用的标签页"""
        for tab_name, tab in self.tabs.items():
            if tab_enabled[tab_name]:
                tab.update_data(full_data)
    
    def clear_all_tabs(self):
        for tab in self.tabs.values():
            if hasattr(tab, 'clear_data'):
                tab.clear_data()

    def set_tab_enabled(self, tab_name, enabled):
        """外部可调用：设置 tab 是否启用"""
        tab_enabled[tab_name] = enabled
        if enabled and tab_name in self.tabs:
            tab = self.tabs[tab_name]
            if hasattr(tab, 'refresh_if_needed'):
                tab.refresh_if_needed()


class CSIDataReceiver:
    def __init__(self):
        self.socket = None
        self.connected = False
        self.receiving_data = False
        self.processor = CSIDataProcessor()
        self.recv_buffer = b""

    def connect(self, host, port):
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(5)
            self.socket.connect((host, port))
            self.socket.settimeout(0.1)
            self.connected = True
            self.receiving_data = True
            return True
        except Exception as e:
            messagebox.showerror("连接错误", f"无法连接到设备: {str(e)}")
            return False

    def disconnect(self):
        self.connected = False
        self.receiving_data = False
        if self.socket:
            self.socket.close()
            self.socket = None

    def send_command(self, command_data):
        if not self.connected or not self.socket:
            return False
        try:
            self.socket.send(command_data)
            return True
        except Exception as e:
            messagebox.showerror("发送错误", f"发送命令失败: {str(e)}")
            return False

    def receive_data_loop(self, full_set_callback):
        while self.connected:
            try:
                chunk = self.socket.recv(1024)
                if not chunk:
                    break
                self.recv_buffer += chunk

                while len(self.recv_buffer) >= 420:
                    packet = self.recv_buffer[:420]
                    self.recv_buffer = self.recv_buffer[420:]
                    parsed = self.parse_csi_packet(packet)
                    if parsed is not None:
                        full_data = self.processor.process(parsed)
                        if full_data is not None:
                            full_set_callback(full_data)
            except socket.timeout:
                continue
            except Exception as e:
                print(f"Error: {e}")
                break

    def parse_csi_packet(self, data):
        try:
            sequence_num = data[0]
            csi_data = data[1:420]  # 419 bytes, matches struct size

            timestamp = struct.unpack('<I', csi_data[0:4])[0]
            mac = csi_data[4:10]
            dmac = csi_data[10:16]
            rssi = struct.unpack('<b', csi_data[16:17])[0]
            noise_floor = struct.unpack('<b', csi_data[17:18])[0]
            rate = csi_data[18]
            sgi = csi_data[19]
            sig_mode = csi_data[20]
            mcs = csi_data[21]
            cwb = csi_data[22]
            channel = csi_data[23]
            secondary_channel = csi_data[24]
            rx_state = csi_data[25]
            fft_gain = csi_data[26]
            agc_gain = csi_data[27]
            gain = struct.unpack('<f', csi_data[28:32])[0]
            first_word_invalid = csi_data[32]
            csi_len = struct.unpack('<H', csi_data[33:35])[0]
            csi_buffer = csi_data[35:35+384]  # buf[384]
            return {
                'antenna_index': sequence_num,
                'timestamp': timestamp,
                'rssi': rssi,
                'noise_floor': noise_floor,
                'mac': ':'.join(f'{b:02x}' for b in mac),
                'dmac': ':'.join(f'{b:02x}' for b in dmac),
                'rate': rate,
                'sgi': sgi,
                'sig_mode': sig_mode,
                'mcs': mcs,
                'cwb': cwb,
                'channel': channel,
                'secondary_channel': -1 if secondary_channel==2 else secondary_channel,
                'rx_state': rx_state,
                'fft_gain': fft_gain,
                'agc_gain': agc_gain,
                'gain': gain,
                'first_word_invalid': first_word_invalid,
                'csi_len': csi_len,
                'csi_data': [b if b < 128 else b - 256 for b in csi_buffer],
            }
        
        except Exception as e:
            print(f"Parse error: {e}")
            return None


class CSIDataGUI:
    def __init__(self, root):
        self.root = root
        self.root.title("CSI数据采集系统")
        self.root.geometry("1400x900")

        self.setup_style()
        self.received_packets = 0
        self.receiver = CSIDataReceiver()
        self._last_full_data = None
        self.setup_ui()

    def setup_style(self):
        style = ttk.Style()
        style.theme_use('clam')
        
        self.colors = {
            "primary": "#2c3e50",
            "secondary": "#3498db", 
            "success": "#27ae60",
            "warning": "#e67e22",
            "danger": "#e74c3c",
            "light_bg": "#ecf0f1"
        }

        # style.configure("TFrame", background=self.colors["light_bg"])
        # style.configure("TLabel", background=self.colors["light_bg"], font=("微软雅黑", 9))
        style.configure("TButton", font=("微软雅黑", 9))
        style.configure("TEntry", font=("微软雅黑", 9))
        style.configure("TCombobox", font=("微软雅黑", 9))
        style.configure("TNotebook", background=self.colors["light_bg"])
        style.configure("TNotebook.Tab", font=("微软雅黑", 9))

        for color_name, color_value in [("Primary", self.colors["secondary"]),
                                       ("Success", self.colors["success"]),
                                       ("Warning", self.colors["warning"]),
                                       ("Danger", self.colors["danger"])]:
            style.configure(f"{color_name}.TButton", background=color_value, foreground="white")

    def setup_ui(self):
        self.root.configure(bg=self.colors["light_bg"])
        
        main_paned = ttk.PanedWindow(self.root, orient=tk.VERTICAL)
        main_paned.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        control_frame = ttk.LabelFrame(main_paned, text="控制面板", padding="8")
        main_paned.add(control_frame, weight=0)

        data_frame = ttk.Frame(main_paned)
        main_paned.add(data_frame, weight=1)

        self.setup_control_panel(control_frame)
        self.setup_data_panel(data_frame)

    def setup_control_panel(self, parent):
        conn_frame = ttk.Frame(parent)
        conn_frame.pack(fill=tk.X, pady=2)

        ttk.Label(conn_frame, text="IP:").pack(side=tk.LEFT, padx=(0, 5))
        self.ip_entry = ttk.Entry(conn_frame, width=12)
        self.ip_entry.insert(0, "192.168.2.10")
        self.ip_entry.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(conn_frame, text="端口:").pack(side=tk.LEFT, padx=(0, 5))
        self.port_entry = ttk.Entry(conn_frame, width=8)
        self.port_entry.insert(0, "8000")
        self.port_entry.pack(side=tk.LEFT, padx=(0, 15))

        self.connect_btn = ttk.Button(conn_frame, text="连接", 
                                    command=self.toggle_connection, style="Primary.TButton")
        self.connect_btn.pack(side=tk.LEFT, padx=(0, 15))

        self.clear_btn = ttk.Button(conn_frame, text="清除数据", 
                                  command=self.clear_all_data, style="Warning.TButton")
        self.clear_btn.pack(side=tk.LEFT, padx=(0, 15))
        self.clear_btn.state(['disabled'])

        ttk.Label(conn_frame, text="数据包:").pack(side=tk.LEFT, padx=(0, 5))
        self.packet_count_label = ttk.Label(conn_frame, text="0", 
                                          foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.packet_count_label.pack(side=tk.LEFT, padx=(0, 15))

        self.status_label = ttk.Label(conn_frame, text="未连接", foreground="red", 
                                    font=("微软雅黑", 9, "bold"))
        self.status_label.pack(side=tk.LEFT)

        control_frame = ttk.Frame(parent)
        control_frame.pack(fill=tk.X, pady=5)

        mac_frame = ttk.Frame(control_frame)
        mac_frame.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(mac_frame, text="MAC地址:").pack(side=tk.LEFT, padx=(0, 5))
        self.mac_entries = []
        default_mac = ["00", "55", "66", "77", "88", "88"]
        for i in range(6):
            entry = ttk.Entry(mac_frame, width=3, justify='center', font=("Consolas", 8))
            entry.insert(0, default_mac[i])
            entry.pack(side=tk.LEFT, padx=1)
            self.mac_entries.append(entry)
            if i < 5:
                ttk.Label(mac_frame, text=":").pack(side=tk.LEFT)

        ttk.Button(control_frame, text="普通模式", 
                  command=self.set_normal_mode, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(control_frame, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_var = tk.StringVar(value="1")
        channel_combo = ttk.Combobox(control_frame, textvariable=self.channel_var,
                                   values=[str(i) for i in range(1, 14)], width=3, state="readonly")
        channel_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Label(control_frame, text="带宽:").pack(side=tk.LEFT, padx=(0, 5))
        self.bandwidth_var = tk.StringVar(value="20M")
        bandwidth_combo = ttk.Combobox(control_frame, textvariable=self.bandwidth_var,
                                   values=["20M", "40M"], width=4, state="readonly")
        bandwidth_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(control_frame, text="设置wifi", 
                  command=self.set_wifi, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(control_frame, text="校准模式", 
                  command=self.set_calibration_mode, style="Success.TButton").pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(control_frame, text="开启校准wifi", 
                  command=self.send_wifion_frame, style="Warning.TButton").pack(side=tk.LEFT)
        
        ttk.Button(control_frame, text="关闭校准wifi", 
                  command=self.send_wifioff_frame, style="Warning.TButton").pack(side=tk.LEFT)
        
        ttk.Button(control_frame, text="应用校准", 
                  command=self.apply_calibration, style="Warning.TButton").pack(side=tk.LEFT)
        
        ttk.Button(control_frame, text="清除校准", 
                  command=self.clear_h_ref, style="Warning.TButton").pack(side=tk.LEFT)
        
        ttk.Label(control_frame, text="协议:").pack(side=tk.LEFT, padx=(5, 5))
        self.wifi_protocol_var = tk.StringVar(value="LLTF")
        wifi_protocol_combo = ttk.Combobox(control_frame, textvariable=self.wifi_protocol_var,
                                   values=["LLTF", "HT40"], width=6, state="readonly")
        wifi_protocol_combo.pack(side=tk.LEFT, padx=(0, 10))

        ttk.Button(control_frame, text="设置协议", 
             command=self.set_protocol, style="Primary.TButton").pack(side=tk.LEFT, padx=(0, 10))

        status_frame = ttk.Frame(parent)
        status_frame.pack(fill=tk.X, pady=2)

        ttk.Label(status_frame, text="模式:").pack(side=tk.LEFT, padx=(0, 5))
        self.mode_label = ttk.Label(status_frame, text="普通模式", 
                                  foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mode_label.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(status_frame, text="信道:").pack(side=tk.LEFT, padx=(0, 5))
        self.channel_label = ttk.Label(status_frame, text="1", 
                                     foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.channel_label.pack(side=tk.LEFT, padx=(0, 15))

        ttk.Label(status_frame, text="过滤MAC:").pack(side=tk.LEFT, padx=(0, 5))
        self.mac_label = ttk.Label(status_frame, text="00:00:00:00:00:00", 
                                 foreground=self.colors["primary"], font=("微软雅黑", 9, "bold"))
        self.mac_label.pack(side=tk.LEFT)

        # === 新增：标签页显示开关 ===
        tab_switch_frame = ttk.Frame(parent)
        tab_switch_frame.pack(fill=tk.X)

        self.tab_checkboxes = {}
        tab_names = [
            ("multi_antenna", "多天线监控"),
            ("csi_plot", "CSI可视化"),
            ("phase_time", "相位时序"),
            ("phase_stability", "相位稳定性"),
            ("calibrated_phase", "校准后相位"),
            ("aoa_1d", "AoA 1D"),  
            ("aoa_2d", "AoA 2D"),  
            ("tdoa", "TDOA"),
        ]
        for tab_key, tab_label in tab_names:
            var = tk.BooleanVar(value=False)
            cb = ttk.Checkbutton(
                tab_switch_frame,
                text=tab_label,
                variable=var,
                command=lambda key=tab_key, v=var: self.on_tab_toggle(key, v)
            )
            cb.pack(side=tk.LEFT, padx=10)
            self.tab_checkboxes[tab_key] = var
        # ==========================

    def setup_data_panel(self, parent):
        self.notebook = ttk.Notebook(parent)
        self.notebook.pack(fill=tk.BOTH, expand=True)

        self.tab_manager = TabManager(self.notebook, self.receiver.processor)
        self.tab_manager.gui_ref = self
        self.tab_manager.create_basic_tabs()

    def toggle_connection(self):
        if not self.receiver.connected:
            self.connect_to_device()
        else:
            self.disconnect_from_device()

    def connect_to_device(self):
        ip = self.ip_entry.get()
        try:
            port = int(self.port_entry.get())
        except ValueError:
            messagebox.showerror("错误", "端口号必须是数字")
            return

        if self.receiver.connect(ip, port):
            self.connect_btn.config(text="断开连接", style="Danger.TButton")
            self.clear_btn.state(['!disabled'])
            self.status_label.config(text="已连接", foreground=self.colors["success"])

            self.receive_thread = threading.Thread(
                target=self.receiver.receive_data_loop,
                args=(self.on_full_antenna_set,),
                daemon=True
            )
            self.receive_thread.start()

    def disconnect_from_device(self):
        self.receiver.disconnect()
        self.connect_btn.config(text="连接", style="Primary.TButton")
        self.clear_btn.state(['disabled'])
        self.status_label.config(text="未连接", foreground="red")

    def clear_all_data(self):
        self.tab_manager.clear_all_tabs()
        self.receiver.processor.clear_antenna_data()
        self.received_packets = 0
        self.packet_count_label.config(text="0")

    def set_normal_mode(self):
        try:
            mac_bytes = []
            for entry in self.mac_entries:
                value = entry.get().strip()
                if not value:
                    messagebox.showerror("错误", "请填写完整的MAC地址")
                    return
                mac_bytes.append(int(value, 16))

            command = bytes([0x01] + mac_bytes)
            if self.receiver.send_command(command):
                self.mode_label.config(text="普通模式")
                self.mac_label.config(text=':'.join(f'{x:02X}' for x in mac_bytes))
        except ValueError:
            messagebox.showerror("错误", "MAC地址必须是十六进制数字")

    def set_wifi(self):
        try:
            channel = int(self.channel_var.get())
            if not 1 <= channel <= 13:
                messagebox.showerror("错误", "信道必须在1-13之间")
                return
            bandwidth = 1 if self.bandwidth_var.get() == "20M" else 2
            command = bytes([0x03, channel, bandwidth])
            if self.receiver.send_command(command):
                self.channel_label.config(text=str(channel))
        except ValueError:
            messagebox.showerror("错误", "请选择有效的信道")

    def set_protocol(self):
        try:
            self.receiver.processor.wifi_protocol = self.wifi_protocol_var.get()
        except ValueError:
            messagebox.showerror("错误")

    def set_calibration_mode(self):
        command = bytes([0x02] + [0x00] * 6)
        if self.receiver.send_command(command):
            self.mode_label.config(text="校准模式")
            self.mac_label.config(text="00:11:22:33:44:55")

    def send_wifion_frame(self):
        command = bytes([0xFF])
        if self.receiver.send_command(command):
            messagebox.showinfo("成功", "校准wifi已开启")

    def send_wifioff_frame(self):
        command = bytes([0xFE])
        if self.receiver.send_command(command):
            messagebox.showinfo("成功", "校准wifi已关闭")

    def apply_calibration(self):
        """将最近一次 full_data 作为新的校准参考 h_ref_origin（每次点击都覆盖）"""
        if self._last_full_data is not None:
            # 所有天线都有数据，进行校准
            h_ref_origin = {
                aid: {
                    'csi_complex': data['csi_complex'].copy(),
                    'timestamp': data.get('timestamp', None)
                }
                for aid, data in self._last_full_data.items()
            }
            self.receiver.processor.calculate_pcb_phase_offsets(
                self._last_full_data['00']['channel'], self._last_full_data['00']['secondary_channel']
            )
            self.receiver.processor.set_calibration_reference(h_ref_origin)
            messagebox.showinfo("校准", "校准数据已更新！")
        else:
            messagebox.showwarning("校准", "尚无可用数据！请先接收完整数据包。")

    def clear_h_ref(self):
        self.receiver.processor.clear_calibration_reference()

    def on_full_antenna_set(self, full_data):
        """收齐8个天线后，更新UI"""
        self.root.after(0, self.update_ui_with_full_set, full_data)

    def update_ui_with_full_set(self, full_data):
        self.received_packets += 1
        self.packet_count_label.config(text=str(self.received_packets))
        self._last_full_data = full_data
        self.tab_manager.update_all_tabs(full_data)

    def on_tab_toggle(self, tab_name, var):
        """标签页开关回调"""
        enabled = var.get()
        self.tab_manager.set_tab_enabled(tab_name, enabled)


def main():
    root = tk.Tk()
    app = CSIDataGUI(root)
    root.mainloop()


if __name__ == "__main__":
    main()