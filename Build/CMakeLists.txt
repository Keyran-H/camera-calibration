cmake_minimum_required(VERSION 2.8.3)
project(intrinsic_and_extrinsic_optimise_real_data)

#Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

find_package(Ceres REQUIRED PATHS "/usr/local/lib/cmake/Ceres/")

find_package(OpenCV REQUIRED PATHS "/usr/local/share/OpenCV")

#Bring the header files into the build environment. They are located in the folder include
include_directories(
  include
  ${CERES_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

#Add the executable files
add_executable(nlo_eg ../src/camera_calibration_real_data.cpp)


target_link_libraries(nlo_eg

   ${CERES_LIBRARIES}
   ${OpenCV_LIBRARIES}
)


