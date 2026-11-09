cmake_minimum_required(VERSION 3.16)

# 必须在 project.cmake 之前设置：本固件只跑 ESP32-S3。
# 不写这一行时，全新克隆后 `idf.py build` 会按 IDF 内置默认值走 esp32，
# 于是 battery_adc.cpp 里的 adc_cali_curve_fitting_* （esp32s3 才有）编译不过。
# sdkconfig.defaults 里的 CONFIG_IDF_TARGET 也能兜底，但那一步 IDF 只是
# “guess”，会打印 "-- IDF_TARGET is not set, guessed '..." 的告警。
set(IDF_TARGET "esp32s3")

include($ENV{IDF_PATH}/tools/cmake/project.cmake)
project(leap_ros)

if(TARGET merge-bin)
    add_custom_target(merged_bin ALL DEPENDS merge-bin)
endif()
