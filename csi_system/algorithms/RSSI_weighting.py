"""RSSI 加权算法模块"""
import numpy as np

class RSSIWeighting:
    def __init__(self):
        pass

    @staticmethod
    def _apply_rssi_weighting(csi_frame, rssi_vals):
        """核心算法：CSI * 10^(RSSI/20)"""
        weights = 10 ** (rssi_vals / 20.0)
        return csi_frame * weights[:, np.newaxis]

    def apply(self, full_data, antenna_order):
        # 1. 收集数据
        csi_list = []
        rssi_list = []
        for aid in antenna_order:
            data = full_data[aid]
            csi_list.append(data['csi_complex'])
            rssi_list.append(data.get('rssi', 0))
        
        csi_matrix = np.array(csi_list)
        rssi_vals = np.array(rssi_list)

        # 2. 计算
        csi_weighted = self._apply_rssi_weighting(csi_matrix, rssi_vals)

        # 3. 写回
        for i, aid in enumerate(antenna_order):
            data = full_data[aid]
            data['csi_complex'] = csi_weighted[i]
            data['csi_magnitude'] = np.abs(csi_weighted[i])
            data['csi_phase'] = np.angle(csi_weighted[i])
            
        return full_data

    def clear(self):
        pass