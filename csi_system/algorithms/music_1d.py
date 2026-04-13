"""MUSIC 1D 到达角估计算法模块"""
import numpy as np
from numba import jit  

class Music1D:
    """MUSIC 1D 算法类"""
    
    def __init__(self):
        # 1D 扫描角度配置
        self.scanning_angles_deg = np.linspace(-90, 90, 180)
        self.scanning_angles = np.deg2rad(self.scanning_angles_deg)
        
        # 缓存配置
        self.MUSIC_BUFFER_SIZE = 1
        self.music_buffer = []

    def apply(self, full_data, antenna_order):
        """
        应用 MUSIC 1D 算法（滑动窗口模式）
        
        参数:
            full_data: 包含 8 根天线对齐后的数据
            antenna_order: 天线顺序列表
            
        返回:
            full_data: 添加 AoA 估计结果的数据
        """

        csi_matrix = np.array([
            full_data[aid]['csi_complex'] for aid in antenna_order
        ])
        
        # 滑动窗口：添加新数据
        self.music_buffer.append(csi_matrix)
        
        # # 如果超过窗口大小，移除最早的一帧
        # if len(self.music_buffer) > self.MUSIC_BUFFER_SIZE:
        #     self.music_buffer.pop(0)
        
        # 只有当窗口满时才进行计算
        if len(self.music_buffer) < self.MUSIC_BUFFER_SIZE:
            full_data['music_computed'] = False
            return full_data
        
        csi_backlog = np.array(self.music_buffer)  # 形状：(缓冲区大小，天线数量，子载波数量)
        csi_backlog = csi_backlog.reshape(csi_backlog.shape[0], 4, -1, csi_backlog.shape[-1])  # 形状：(缓冲区大小，行天线数量，列天线数量，子载波数量)
        csi_backlog = csi_backlog[:, :, :, 14]  # 选取子载波 14，形状：(缓冲区大小，行天线数量，列天线数量)
        
        # 计算协方差矩阵
        # R = np.einsum("tics,tjcs->ij", csi_backlog, np.conj(csi_backlog))
        R = np.einsum("tic,tjc->ij", csi_backlog, np.conj(csi_backlog))
        
        # 计算特征分解
        eig_val, eig_vec = np.linalg.eig(R)
        order = np.argsort(eig_val)[::-1]
        Qn = eig_vec[:, order][:, 1:]
        
        # 计算导向矢量
        steering_vectors = np.exp(-1.0j * np.outer(np.pi * np.sin(self.scanning_angles), np.arange(R.shape[0])))
        
        # 计算 MUSIC 谱
        QnH = np.conj(Qn).T
        QnH_a = QnH @ steering_vectors.T
        norms = np.linalg.norm(QnH_a, axis=0)
        spatial_spectrum_linear = 1 / (norms ** 2)
        
        # 转换为对数刻度
        spatial_spectrum_log = 20 * np.log10(spatial_spectrum_linear + 1e-10)
        peak_idx = np.argmax(spatial_spectrum_log)
        aoa_angle = self.scanning_angles_deg[peak_idx]

        # 结果写入
        full_data.update({
            'aoa': aoa_angle,
            'aoa_spectrum': spatial_spectrum_log.tolist(),
            'aoa_angles': self.scanning_angles_deg.tolist(),
            'music_computed': True,
        })
        
        # 滑动窗口模式：不清空 buffer，保持窗口大小
        self.music_buffer = [] 
        
        return full_data
    
    def clear(self):
        """清空算法状态"""
        self.music_buffer = []