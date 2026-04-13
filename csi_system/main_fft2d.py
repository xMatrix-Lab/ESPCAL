"""
CSI 数据采集系统 - 主入口
"""
import tkinter as tk
import sys  
from core.csi_core_single import CSIDataGUI
from algorithms.iterative_filter import IterativeFilter
from algorithms.eigenvec_filter import EigenvecPerSubcarrierFilter
from algorithms.fft_2d import FFTBeamspace
from algorithms.CIR_sync import CIRSync
from algorithms.RSSI_weighting import RSSIWeighting
from tabs.tab_fft_2d import FFTBeamspaceTab
from tabs.tab_fft_2d_camera import FFTBeamspaceCameraTab

class MyCSIApp(CSIDataGUI):  
    def __init__(self, root):
        super().__init__(root, title="CSI MUSIC AOA2D", window_size="1400x900")
        self._init_algorithms()
        self._init_tabs()
    
    def _init_algorithms(self):
        """初始化算法模块"""
        rssi_weighting = RSSIWeighting()
        self.register_algorithm('rssi_weighting', rssi_weighting)

        # cir_sync = CIRSync(
        #     max_delay_taps=3,
        #     search_res=40,
        #     peak_threshold=0.5
        # )
        # self.register_algorithm('cir_sync', cir_sync)

        # iterative_filter = IterativeFilter(
        #     sliding_window_size=5,
        #     iterations=5
        # )
        # self.register_algorithm('iterative_filter', iterative_filter)

        eigenvec_filter = EigenvecPerSubcarrierFilter(
            sliding_window_size=30,
        )
        self.register_algorithm('eigenvec_filter', eigenvec_filter)

        fft_2d = FFTBeamspace()
        self.register_algorithm('fft_2d', fft_2d)

    def _init_tabs(self):
        """初始化 Tab 页"""
        # 注册 FFT 2D 显示 Tab
        self.register_tab('FFT 2D Camera', FFTBeamspaceCameraTab, enabled=True)
        # self.register_tab('FFT 2D', FFTBeamspaceTab, enabled=False)
    


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