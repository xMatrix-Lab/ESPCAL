if(EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/esp_radar_motion_dec.c")
set(sources "src/esp_radar_motion_dec.c")
else()
set(sources "")
endif()

idf_component_register( SRCS "${sources}"
                        INCLUDE_DIRS "include"
                        REQUIRES esp_common)

if(NOT EXISTS "${CMAKE_CURRENT_SOURCE_DIR}/src/esp_radar_motion_dec.c")
    add_prebuilt_library(esp_radar_motion_dec_lib "${CMAKE_CURRENT_SOURCE_DIR}/${IDF_VERSION_MAJOR}.${IDF_VERSION_MINOR}/${CONFIG_IDF_TARGET}/libesp_radar_motion_dec.a")
    target_include_directories(${COMPONENT_LIB} INTERFACE "${CMAKE_CURRENT_SOURCE_DIR}/include")
    target_link_libraries(${COMPONENT_LIB} INTERFACE esp_radar_motion_dec_lib)
else()
    include(package_manager)
    cu_pkg_define_version(${CMAKE_CURRENT_LIST_DIR})
    target_compile_options(${COMPONENT_LIB} PRIVATE -DLOG_LOCAL_LEVEL=ESP_LOG_DEBUG  -Wignored-qualifiers)
endif()
