# algorithms/music_2d.py
"""MUSIC 2D 到达角估计算法模块 - 性能优化版"""
import numpy as np
from numba import jit

class Music2D:
    """MUSIC 2D 算法类"""
    
    def __init__(self):
        self.azimuth_angles_deg = np.linspace(-90, 90, 180)   # 方位角
        self.elevation_angles_deg = np.linspace(-90, 90, 180)   # 仰角
        self.azimuth_angles = np.deg2rad(self.azimuth_angles_deg)
        self.elevation_angles = np.deg2rad(self.elevation_angles_deg)

        # 缓存配置
        self.MUSIC_BUFFER_SIZE = 1
        self.music_buffer = []

    def music_algorithm(self,R, angles):
        """
		基于MUSIC算法计算空间频谱
		参数:
			R: numpy.ndarray - 阵列协方差矩阵
		返回:
			numpy.ndarray - 空间频谱（dB单位）
		"""
        steering_vectors = np.exp(-1.0j * np.outer(np.pi * np.sin(angles), np.arange(R.shape[0])))  # 计算导向矢量

		# 使用MUSIC算法基于R计算空间频谱
        eig_val, eig_vec = np.linalg.eig(R)  # 特征值分解
        order = np.argsort(eig_val)[::-1]  # 特征值降序排列
        Qn = eig_vec[:,order][:,1:]  # 噪声子空间
        spatial_spectrum = 1 / np.linalg.norm(np.einsum("ae,ra->er", np.conj(Qn), steering_vectors), axis = 0)  # 计算空间频谱
        spatial_spectrum /= spatial_spectrum.max()  # 归一化

        return 20 * np.log10(spatial_spectrum + 1e-10)  # 转换为dB单位

    def apply(self, full_data, antenna_order):
        """
        应用 MUSIC 2D 算法
        
        参数:
            full_data: 包含 8 根天线对齐后的数据
            antenna_order: 天线顺序列表
            
        返回:
            full_data: 添加 AoA 估计结果的数据
        """
        csi_matrix = np.array([full_data[aid]['csi_complex'] for aid in antenna_order])
        self.music_buffer.append(csi_matrix)
        if len(self.music_buffer) < self.MUSIC_BUFFER_SIZE:
            full_data['music2d_computed'] = False
            return full_data
        csi_backlog = np.array(self.music_buffer) #(缓冲区大小, 天线数量, 子载波数量)
        csi_backlog = csi_backlog.reshape(csi_backlog.shape[0], 4, -1, csi_backlog.shape[-1])#(缓冲区大小, 行数量, 列数量, 子载波数量)
    
        # 计算协方差矩阵
        R_h = np.einsum("tris,trjs->ij", csi_backlog, np.conj(csi_backlog))
        R_v = np.einsum("tics,tjcs->ij", csi_backlog, np.conj(csi_backlog))

        spectrum_azimuth = self.music_algorithm(R_h, self.azimuth_angles)
        spectrum_elevation = self.music_algorithm(R_v, self.elevation_angles)

        spectrum_azimuth = np.roll(spectrum_azimuth, shift=len(spectrum_azimuth) // 2)  # 不知道为什么中心位置不对，需要额外将谱中心滚动对齐

        peak_idx = np.argmax(spectrum_azimuth)
        aoa_azimuth = self.azimuth_angles_deg[peak_idx]
        peak_idx = np.argmax(spectrum_elevation)
        aoa_elevation = self.elevation_angles_deg[peak_idx]

        full_data.update({
            'aoa_spectrum_azimuth': spectrum_azimuth.tolist(),
            'aoa_spectrum_elevation': spectrum_elevation.tolist(),
            'aoa_azimuth': aoa_azimuth,
            'aoa_elevation': aoa_elevation,
            'aoa_azimuths': (self.azimuth_angles_deg).tolist(),
            'aoa_elevations': self.elevation_angles_deg.tolist(),
            'music2d_computed': True
        })

        self.music_buffer.clear()  # 处理完一批数据后清空缓存
        return full_data                
   
    def clear(self):
        """清空算法状态"""
        self.music_buffer.clear()      
        pass