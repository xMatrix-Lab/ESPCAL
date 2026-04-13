idf_component_register(SRC_DIRS "src"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_wifi esp_timer esp_csi_gain_ctrl esp_radar_motion_dec)
message(STATUS "Building esp-radar")
include(package_manager)
cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
message(STATUS "Using local esp-radar library")
target_compile_options(${COMPONENT_LIB} PRIVATE -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG "-Wno-format")
