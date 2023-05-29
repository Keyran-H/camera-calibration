cmake_minimum_required(VERSION 3.10)
project(intrinsic_and_extrinsic_optimise_real_data)

add_compile_options(-std=c++11)

find_package(Ceres REQUIRED)

find_package(OpenCV REQUIRED PATHS "/usr/local/share/OpenCV") # TODO: add in commands to install this locally.

#Add the executable files
add_executable(nlo-camera-calibration src/camera_calibration_real_data.cpp)

target_link_libraries(nlo-camera-calibration
   ${CERES_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


