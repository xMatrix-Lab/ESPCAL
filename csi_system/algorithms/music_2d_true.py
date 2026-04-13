"""MUSIC 2D 到达角估计算法模块 - 真二维联合估计版"""
import numpy as np
from numba import jit, prange

class Music2D_True:
    """真二维 MUSIC 算法类 - 基于均匀平面阵列(UPA) 4行2列 8天线"""
    
    def __init__(self, 
                 az_range=(-90, 90), el_range=(-90, 90),
                 az_points=128, el_points=128,
                 n_sources=1
                 ):        
        """
        初始化真2D MUSIC算法
        阵列：4 行 × 2 列 UPA (8天线)
        总天线数：4×2 = 8
        """
        # 角度网格
        self.az_points = az_points
        self.el_points = el_points
        
        self.azimuth_angles_deg = np.linspace(*az_range, az_points)
        self.elevation_angles_deg = np.linspace(*el_range, el_points)
        self.azimuth_angles = np.deg2rad(self.azimuth_angles_deg)
        self.elevation_angles = np.deg2rad(self.elevation_angles_deg)
        
        # 4行2列阵列配置
        self.N_y = 4    # 行数 (y 方向)
        self.N_x = 2    # 列数 (x 方向)
        self.M = self.N_y * self.N_x    # 8 阵元
        self.d_lambda = 0.5
        self.n_sources = n_sources
        
        # 角度网格
        self.AZ_GRID, self.EL_GRID = np.meshgrid(
            self.azimuth_angles, self.elevation_angles, indexing='ij')
        
        # 导向矢量
        self.steering_matrix_2d = self._build_2d_steering_matrix().astype(np.complex128)
        
        # 缓冲
        self.MUSIC_BUFFER_SIZE = 10
        self.music_buffer = []

    def _build_2d_steering_matrix(self):
        """4行 × 2列 UPA 正确导向矢量"""
        az = self.AZ_GRID.ravel()
        el = self.EL_GRID.ravel()
        
        cos_el = np.cos(el)
        cos_az = np.cos(az)
        sin_az = np.sin(az)
        
        nx, ny = np.meshgrid(np.arange(self.N_x), np.arange(self.N_y))
        nx = nx.flatten()
        ny = ny.flatten()
        
        phase = 2 * np.pi * self.d_lambda * (
            nx[None, :] * cos_el[:, None] * cos_az[:, None] +
            ny[None, :] * cos_el[:, None] * sin_az[:, None]
        )
        
        steering = np.exp(1j * phase)
        return steering

    @staticmethod
    @jit(nopython=True, parallel=True, fastmath=True)
    def _music_spectrum_2d_numba(Qn, steering_matrix, az_points, el_points):
        grid_num = steering_matrix.shape[0]
        spectrum = np.zeros(grid_num, dtype=np.float32)
        
        for g in prange(grid_num):
            a = steering_matrix[g]
            norm = 0.0
            for k in range(Qn.shape[1]):
                q_vec = Qn[:, k]
                real_sum = 0.0
                imag_sum = 0.0
                for i in range(q_vec.shape[0]):
                    real_q = q_vec[i].real
                    imag_q = q_vec[i].imag
                    real_a = a[i].real
                    imag_a = a[i].imag
                    real_sum += real_q * real_a + imag_q * imag_a
                    imag_sum += real_q * imag_a - imag_q * real_a
                norm += real_sum**2 + imag_sum**2

            spectrum[g] = 1.0 / (norm + 1e-10)
        
        # 【关键修复】固定维度
        return spectrum.reshape(az_points, el_points)

    def music_algorithm_2d(self, R):
        eig_val, eig_vec = np.linalg.eigh(R)
        idx = np.argsort(eig_val)[::-1]
        eig_val = eig_val[idx]
        eig_vec = eig_vec[:, idx]
        
        Qn = eig_vec[:, self.n_sources:]
        
        # 传入正确维度
        spec_linear = self._music_spectrum_2d_numba(
            np.ascontiguousarray(Qn),
            np.ascontiguousarray(self.steering_matrix_2d),
            self.az_points,
            self.el_points
        )
        
        spec_linear = spec_linear / (spec_linear.max() + 1e-10)
        spectrum_dB = 10 * np.log10(spec_linear + 1e-10)
        
        peak_az, peak_el = np.unravel_index(np.argmax(spectrum_dB), spectrum_dB.shape)
        return spectrum_dB, self.azimuth_angles_deg[peak_az], self.elevation_angles_deg[peak_el]

    def apply(self, full_data, antenna_order):
        try:
            csi_vectors = [full_data[aid]['csi_complex'] for aid in antenna_order[:self.M]]
            csi_matrix = np.array(csi_vectors, dtype=np.complex128)
            
            self.music_buffer.append(csi_matrix)
            if len(self.music_buffer) < self.MUSIC_BUFFER_SIZE:
                full_data['music2d_computed'] = False
                return full_data
            
            X = np.array(self.music_buffer)
            R = np.zeros((self.M, self.M), dtype=np.complex128)
            for t in range(X.shape[0]):
                x = X[t]
                R += x @ x.conj().T
            R /= (X.shape[0] * X.shape[2])
            
            spec2d, az, el = self.music_algorithm_2d(R)
            
            full_data.update({
                'aoa_spectrum_2d': spec2d.tolist(),
                'aoa_azimuth': float(az),
                'aoa_elevation': float(el),
                'aoa_azimuths': self.azimuth_angles_deg.tolist(),
                'aoa_elevations': self.elevation_angles_deg.tolist(),
                'music2d_computed': True,
            })
            
            self.music_buffer.pop(0)
            
        except Exception as e:
            full_data['music2d_computed'] = False
            print(f"2D MUSIC 计算错误: {e}")
        
        return full_data
                
    def clear(self):
        self.music_buffer.clear()