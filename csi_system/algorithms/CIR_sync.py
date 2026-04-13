"""CIR 峰值同步滤波算法模块"""
import numpy as np

class CIRSync:
    """基于 CIR 第一个峰值的全局同步算法类"""
    
    def __init__(self, max_delay_taps=10, search_res=20, peak_threshold=0.5):
        """
        :param max_delay_taps: 搜索的最大负时延点数 (例如 -10 ~ 0)
        :param search_res: 搜索分辨率 (在范围内取多少个点)
        :param peak_threshold: 峰值检测阈值 (0.0 ~ 1.0)，相对于最大功率的比率
        """
        self.CIR_MAX_DELAY_TAPS = max_delay_taps
        self.CIR_SEARCH_RES = search_res
        self.CIR_PEAK_THRESHOLD = peak_threshold

    @staticmethod
    def _shift_to_firstpeak_sync_single(csi_frame, max_delay_taps, search_res, peak_threshold):
        """ 
        对 *单帧* CSI 数据进行全局 CIR 峰值同步（所有天线使用相同移位）
        """
        M, K = csi_frame.shape
        
        # 1. 生成时延候选和移位向量
        # 搜索范围：从 -max_delay_taps 到 0
        shifts = np.linspace(-max_delay_taps, 0, search_res)  # (D,)
        subcarrier_range = np.arange(K) - (K // 2)  # (K,)
        
        # 移位向量: 对每个时延 d，生成频域相位旋转因子 exp(j * 2*pi * d * k / K)
        # outer 形状: (D, K)
        shift_vectors = np.exp(1.0j * np.outer(shifts, 2 * np.pi * subcarrier_range / K))

        # 2. 计算每个时延 d 下的 *总接收功率*
        # 应用移位：(M, K) * (D, K) -> 需要广播
        # shifted[d, m, k] = csi_frame[m, k] * shift_vectors[d, k]
        # 使用 einsum 或者 broadcasting
        # broadcasting 方式: csi_frame[None, :, :] * shift_vectors[:, None, :] -> (D, M, K)
        shifted = csi_frame[None, :, :] * shift_vectors[:, None, :]
        
        # 计算能量：对每个 d，求所有 m (天线) 和 k (子载波) 的能量和
        total_power = np.sum(np.abs(shifted) ** 2, axis=(1, 2))  # (D,)

        # 3. 找到第一个超过阈值的峰值
        if np.max(total_power) == 0:
            best_delay_idx = 0
        else:
            max_power = np.max(total_power)
            threshold = peak_threshold * max_power
            
            # 从左到右找第一个超过阈值的 (对应最大的负时延，即最早的信号)
            valid = total_power > threshold
            if np.any(valid):
                # argmax 返回第一个最大值的索引，对于布尔数组，True=1, False=0
                # 所以 argmax 会返回第一个 True 的位置
                best_delay_idx = np.argmax(valid)
            else:
                best_delay_idx = 0  # 无显著峰值，不移位

        # 4. 应用最佳移位（广播到所有天线）
        best_shift = shift_vectors[best_delay_idx]  # (K,)
        csi_sync = csi_frame * best_shift  # (M, K) * (K,) → (M, K)

        return csi_sync
    
    def apply(self, full_data, antenna_order):
        """应用全局 CIR 同步"""
        if not antenna_order:
            return full_data
            
        try:
            csi_matrix = np.array([
                full_data[aid]['csi_complex'] for aid in antenna_order
            ], dtype=np.complex64)
            
            # 3. 执行同步
            csi_synced = self._shift_to_firstpeak_sync_single(
                csi_matrix, 
                self.CIR_MAX_DELAY_TAPS, 
                self.CIR_SEARCH_RES, 
                self.CIR_PEAK_THRESHOLD
            )
            
            # 4. 写回数据
            for idx, aid in enumerate(antenna_order):
                data = full_data[aid]
                h_vec = csi_synced[idx]
                data['csi_complex'] = h_vec
                # data['csi_magnitude'] = np.abs(h_vec)
                # data['csi_phase'] = np.angle(h_vec)
                
            return full_data
            
        except Exception as e:
            print(f"[CIRSyncFilter] 处理出错：{e}")
            return full_data

    def clear(self):
        """重置状态 (此类无状态)"""
        pass