# v0.1.0
This is the first release version for esp-radar component in Espressif Component Registry, more detailed descriptions about the project.

# v0.2.0
1. Added Wi Fi fft gain and agc gain acquisition and settings
2. Added CSI data gain compensation function to improve detection accuracy

# v0.3.0
1. Refactored the `esp_radar` library for better modularity and maintainability.
2. Open-sourced the main source code, allowing users to customize and extend functionalities.
3. Added support for ESP32-C5, ESP32-C6, and ESP32-C61 chips.
4. Improved CSI data handling and radar detection performance.
5. Optimized API interfaces for easier integration in user projects.

# v0.3.1
1. Add support for esp32c61

# v0.3.2
1. Reduce warnings.
2. Change the default LTF type to LLTF.
3. Fix threshold exception issue when calling radar_stop.