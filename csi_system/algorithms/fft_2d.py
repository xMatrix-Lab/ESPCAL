"""FFT 波束空间到达角估计算法模块"""
import numpy as np

class FFTBeamspace:
    """FFT波束空间算法"""
    
    def __init__(self, 
                 resolution_azimuth=128, 
                 resolution_elevation=128, 
                 array_config=(4, 2),
                 roll_azimuth_half=True,
                 flip_azimuth=False,
                 flip_elevation=True
                 ):
        self.res_az = resolution_azimuth
        self.res_el = resolution_elevation
        self.N_y, self.N_x = array_config
        self.M = self.N_y * self.N_x
        
        self.roll_az_half = roll_azimuth_half
        self.flip_az = flip_azimuth
        self.flip_el = flip_elevation
        
        self.azimuth_angles_deg = np.linspace(-90, 90, self.res_az)
        self.elevation_angles_deg = np.linspace(-90, 90, self.res_el)
        
        if self.roll_az_half:
            self.azimuth_angles_deg = np.roll(self.azimuth_angles_deg, self.res_az // 2)

    def apply(self, full_data, antenna_order):
        try:
            csi_matrix = np.array([
                full_data[aid]['csi_complex'] for aid in antenna_order[:self.M]
            ])
            csi_combined = csi_matrix.reshape(self.N_y, self.N_x, csi_matrix.shape[-1])
            
            csi_tdomain = np.fft.ifftshift(
                np.fft.ifft(
                    np.fft.fftshift(csi_combined, axes=-1), 
                    axis=-1
                ), 
                axes=-1
            )
            
            tap_count = csi_tdomain.shape[-1]
            start_idx = tap_count // 2 + 1 - 16
            end_idx = tap_count // 2 + 1 + 17
            csi_tdomain_cut = csi_tdomain[..., start_idx:end_idx]
            
            csi_fdomain_cut = np.fft.ifftshift(
                np.fft.fft(
                    np.fft.fftshift(csi_tdomain_cut, axes=-1), 
                    axis=-1
                ), 
                axes=-1
            )
            
            subs_cut = csi_fdomain_cut.shape[-1]
            csi_zeropadded = np.zeros(
                (self.res_az, self.res_el, subs_cut), 
                dtype=csi_fdomain_cut.dtype
            )
            
            real_cols_half = csi_fdomain_cut.shape[1] // 2
            real_rows_half = csi_fdomain_cut.shape[0] // 2
            zeropadded_cols_half = csi_zeropadded.shape[0] // 2
            zeropadded_rows_half = csi_zeropadded.shape[1] // 2
            
            csi_input = np.swapaxes(csi_fdomain_cut, 0, 1)
            csi_zeropadded[
                zeropadded_cols_half - real_cols_half : zeropadded_cols_half + real_cols_half,
                zeropadded_rows_half - real_rows_half : zeropadded_rows_half + real_rows_half,
                :
            ] = csi_input
            
            csi_zeropadded = np.fft.ifftshift(csi_zeropadded, axes=(0, 1))
            beam_frequency_space = np.fft.fft2(csi_zeropadded, axes=(0, 1))
            beam_frequency_space = np.fft.fftshift(beam_frequency_space, axes=(0, 1))
            
            raw_power_spectrum = np.sum(np.abs(beam_frequency_space)**2, axis=-1).T
            
            if self.roll_az_half:
                raw_power_spectrum = np.roll(raw_power_spectrum, self.res_az // 2, axis=1)
            if self.flip_az:
                raw_power_spectrum = np.fliplr(raw_power_spectrum)
            if self.flip_el:
                raw_power_spectrum = np.flipud(raw_power_spectrum)
            
            max_power = np.max(raw_power_spectrum)
            power_norm = raw_power_spectrum / (max_power + 1e-10) if max_power > 0 else raw_power_spectrum
            spectrum_db = 10 * np.log10(power_norm + 1e-10)
            
            peak_el_idx, peak_az_idx = np.unravel_index(np.argmax(raw_power_spectrum), raw_power_spectrum.shape)
            est_az = self.azimuth_angles_deg[peak_az_idx]
            est_el = self.elevation_angles_deg[peak_el_idx]
            
            full_data.update({
                'aoa_spectrum_2d': spectrum_db.tolist(),
                'raw_power_spectrum': raw_power_spectrum.tolist(),
                'aoa_azimuth': float(est_az),
                'aoa_elevation': float(est_el),
                'aoa_azimuths': self.azimuth_angles_deg.tolist(),
                'aoa_elevations': self.elevation_angles_deg.tolist(),
                'fft2d_computed': True, 
            })
            
        except Exception as e:
            full_data['fft2d_computed'] = False
            print(f"FFT Beamspace 计算错误: {e}")
        
        return full_data

    def clear(self):
        pass