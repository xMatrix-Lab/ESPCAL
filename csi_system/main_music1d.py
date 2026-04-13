"""
CSI 数据采集系统 - 主入口
"""
import tkinter as tk
import sys  
from core.csi_core_single import CSIDataGUI
from algorithms.iterative_filter import IterativeFilter
from algorithms.eigenvec_filter import EigenvecPerSubcarrierFilter
from algorithms.music_1d import Music1D
from algorithms.CIR_sync import CIRSync
from algorithms.RSSI_weighting import RSSIWeighting
from algorithms.sinusoid_filter import ComplexSinusoidFitFilter
from tabs.tab_music_1d import AOA1DTab
from tabs.tab_data_collect import DataCollectTab


class MyCSIApp(CSIDataGUI):  
    def __init__(self, root):
        super().__init__(root, title="CSI MUSIC AOA1D", window_size="1400x900")
        self._init_algorithms()
        self._init_tabs()
    
    def _init_algorithms(self):
        """初始化算法模块"""

        #开启这个会好一点，不要使用esp32自带的补偿因子，会导致乱跳
        rssi_weighting = RSSIWeighting()
        self.register_algorithm('rssi_weighting', rssi_weighting)

        #似乎开了之后效果还差一点，又似乎对music没影响
        # cir_sync = CIRSync(
        #     max_delay_taps=3,
        #     search_res=40,
        #     peak_threshold=0.5
        # )
        # self.register_algorithm('cir_sync', cir_sync)

        #不知道有什么用，怎么用
        # sinusoid_fit = ComplexSinusoidFitFilter(
        #     sliding_window_size=30
        # )
        # self.register_algorithm('sinusoid_fit', sinusoid_fit)

        # iterative_filter = IterativeFilter(
        #     sliding_window_size=30,
        #     iterations=10
        # )
        # self.register_algorithm('iterative_filter', iterative_filter)

        #协方差滤波似乎比迭代滤波好一点
        eigenvec_filter = EigenvecPerSubcarrierFilter(
            sliding_window_size=30,
        )
        self.register_algorithm('eigenvec_filter', eigenvec_filter)

        music_1d = Music1D()
        self.register_algorithm('aoa_1d', music_1d)
    
    def _init_tabs(self):
        """初始化 Tab 页"""
        self.register_tab('AoA 1D', AOA1DTab, enabled=True)
        self.register_tab('Data Collection', DataCollectTab, enabled=True)
        



def main():
    root = tk.Tk()
    
    def on_closing():
        root.quit()    # 停止Tkinter主循环
        root.destroy() # 销毁所有UI组件
        sys.exit(0)    # 强制退出Python进程，杜绝残留
    
    root.protocol("WM_DELETE_WINDOW", on_closing)
    
    app = MyCSIApp(root)
    root.mainloop()

if __name__ == "__main__":
    main()